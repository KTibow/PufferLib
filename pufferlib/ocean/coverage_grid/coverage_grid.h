#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "raylib.h"

const unsigned char ACTION_GO_FORWARD = 0;
const unsigned char ACTION_GO_LEFT = 1;
const unsigned char ACTION_GO_RIGHT = 2;

const unsigned char DIRECTION_NORTH = 0;
const unsigned char DIRECTION_EAST = 1;
const unsigned char DIRECTION_SOUTH = 2;
const unsigned char DIRECTION_WEST = 3;

const int CELL_UNVISITED = 0;
const int CELL_VISITED = 1;

#define GRID_WIDTH 8
#define GRID_HEIGHT 6
#define GRID_CELLS (GRID_WIDTH * GRID_HEIGHT)
const int MAX_STEPS = GRID_CELLS;

typedef struct {
    float coverage_percentage;
    float collisions;
    float episode_return;
    float n; // Required as the last field
} Log;

typedef struct {
    Log log;                     // Required field
    float* observations;         // Required field - 2D
    int* actions;                // Required field - 3 discrete actions
    float* rewards;              // Required field
    unsigned char* terminals;    // Required field

    // Environment state
    int x, y;                    // Agent position (integer grid coordinates)
    unsigned char direction;     // Agent facing direction (0=North, 1=East, 2=South, 3=West)
    unsigned char visited[GRID_CELLS]; // 0 = unvisited, 1 = visited
    int visited_count;           // Number of cells visited
    int tick;

    // Wall collision tracking
    int wall_collisions;

    // Episode return tracking
    float episode_return;
} CoverageGrid;

void add_log(CoverageGrid* env) {
    env->log.coverage_percentage = (
        (float)env->visited_count / (float)GRID_CELLS * 100.0f
    );
    env->log.collisions += env->wall_collisions;
    env->log.episode_return += env->episode_return;
    env->log.n++;
}

void update_observations(CoverageGrid* env) {
    env->observations[0] = env->y == 0 ? 1.0f : 0.0f;
    env->observations[1] = env->y == GRID_HEIGHT - 1 ? 1.0f : 0.0f;
}

void c_reset(CoverageGrid* env) {
    // Reset location
    env->x = 0;
    env->y = 1;
    env->direction = DIRECTION_SOUTH;
    env->tick = 0;
    env->wall_collisions = 0;

    // Reset visited grid
    memset(env->visited, CELL_UNVISITED, GRID_CELLS);
    env->visited_count = 0;

    // Mark starting position as visited
    env->visited[env->y * GRID_WIDTH + env->x] = CELL_VISITED;
    env->visited_count++;

    // Reset episode return
    env->episode_return = 0.0f;

    // Update initial observations
    update_observations(env);
}

void c_step(CoverageGrid* env) {
    env->tick += 1;
    env->rewards[0] = 0.0f;
    env->terminals[0] = 0;

    // Process action
    int action = env->actions[0];
    // int action = env->y == 0 ? ACTION_GO_RIGHT : env->y == (GRID_HEIGHT - 1) ? ACTION_GO_LEFT : ACTION_GO_FORWARD;
    int new_x = env->x;
    int new_y = env->y;

    switch (action) {
        case ACTION_GO_FORWARD:
            // Just go forward in current direction
            break;
        case ACTION_GO_LEFT:
            // Turn left then go forward
            env->direction = (env->direction + 3) % 4;
            break;
        case ACTION_GO_RIGHT:
            // Turn right then go forward
            env->direction = (env->direction + 1) % 4;
            break;
        default:
            // Invalid action, stay in place
            break;
    }

    // Always move forward
    switch (env->direction) {
        case DIRECTION_NORTH:
            new_y = env->y - 1;
            break;
        case DIRECTION_EAST:
            new_x = env->x + 1;
            break;
        case DIRECTION_SOUTH:
            new_y = env->y + 1;
            break;
        case DIRECTION_WEST:
            new_x = env->x - 1;
            break;
    }

    // Check if new position is within bounds
    if (new_x >= 0 && new_x < GRID_WIDTH &&
        new_y >= 0 && new_y < GRID_HEIGHT) {

        // Move to new position
        env->x = new_x;
        env->y = new_y;

        // Check if this cell was already visited
        int cell_idx = env->y * GRID_WIDTH + env->x;
        if (env->visited[cell_idx] == CELL_UNVISITED) {
            // Reward for visiting new cell
            env->visited[cell_idx] = CELL_VISITED;
            env->visited_count++;
            env->rewards[0] += 0.1f;
        } else {
            // Small penalty for revisiting cell
            env->rewards[0] -= 0.05f;
        }
    } else {
        // Wall collision - no movement, penalty
        env->wall_collisions++;
        env->rewards[0] -= 0.1f;
    }

    // Accumulate episode return
    env->episode_return += env->rewards[0];

    // Update observations
    update_observations(env);

    // Check for episode termination
    if (env->tick >= MAX_STEPS) {
        env->terminals[0] = 1;
        add_log(env);
        c_reset(env);
    }
}

void c_render(CoverageGrid* env) {
    const int CELL_SIZE = 40;
    const int WINDOW_WIDTH = fmaxf(GRID_WIDTH * CELL_SIZE, 600);
    const int WINDOW_HEIGHT = (GRID_HEIGHT + 2) * CELL_SIZE;
    const int offset_x = 0.5 * (WINDOW_WIDTH - (GRID_WIDTH * CELL_SIZE));
    const int offset_y = WINDOW_HEIGHT - (GRID_HEIGHT * CELL_SIZE);

    if (!IsWindowReady()) {
        InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PufferLib Coverage Grid");
        SetTargetFPS(10);
    }


    if (WindowShouldClose() || IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});

    // Draw grid cells
    for (int y = 0; y < GRID_HEIGHT; y++) {
        for (int x = 0; x < GRID_WIDTH; x++) {
            int cell_idx = y * GRID_WIDTH + x;
            int pixel_x = offset_x + x * CELL_SIZE;
            int pixel_y = offset_y + y * CELL_SIZE;

            Color cell_color;
            if (env->visited[cell_idx] == CELL_VISITED) {
                cell_color = (Color){0, 100, 0, 255};  // Green for visited
            } else {
                cell_color = (Color){50, 50, 50, 255}; // Dark gray for unvisited
            }

            DrawRectangle(pixel_x, pixel_y, CELL_SIZE, CELL_SIZE, cell_color);
            DrawRectangleLines(pixel_x, pixel_y, CELL_SIZE, CELL_SIZE, (Color){200, 200, 200, 255});
        }
    }

    // Draw agent with direction indicator
    int agent_pixel_x = offset_x + env->x * CELL_SIZE + CELL_SIZE / 2;
    int agent_pixel_y = offset_y + env->y * CELL_SIZE + CELL_SIZE / 2;
    DrawCircle(agent_pixel_x, agent_pixel_y, CELL_SIZE / 3, (Color){255, 255, 0, 255});

    // Draw direction arrow
    int arrow_length = CELL_SIZE / 4;
    int arrow_end_x = agent_pixel_x;
    int arrow_end_y = agent_pixel_y;

    switch (env->direction) {
        case DIRECTION_NORTH:
            arrow_end_y -= arrow_length;
            break;
        case DIRECTION_EAST:
            arrow_end_x += arrow_length;
            break;
        case DIRECTION_SOUTH:
            arrow_end_y += arrow_length;
            break;
        case DIRECTION_WEST:
            arrow_end_x -= arrow_length;
            break;
    }

    DrawLineEx(
        (Vector2){agent_pixel_x, agent_pixel_y},
        (Vector2){arrow_end_x, arrow_end_y},
        4.0f,
        (Color){255, 0, 0, 255}
    );

    // Draw status information
    char status_text[200];
    snprintf(status_text, sizeof(status_text),
        "%d/%d | %d collisions | %.2f/%.2f reward | Step %d",
        env->visited_count, GRID_CELLS,
        env->wall_collisions,
        env->rewards[0], env->episode_return,
        env->tick);
    DrawText(status_text, 10, 10, 20, (Color){241, 241, 241, 255});

    EndDrawing();
}

void c_close(CoverageGrid* env) {
    if (IsWindowReady()) {
        CloseWindow();
    }
}
