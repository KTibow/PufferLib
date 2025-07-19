#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "raylib.h"

const unsigned char ACTION_UP = 0;
const unsigned char ACTION_DOWN = 1;
const unsigned char ACTION_LEFT = 2;
const unsigned char ACTION_RIGHT = 3;

const int CELL_UNVISITED = 0;
const int CELL_VISITED = 1;

const int OBS_TYPE_POSITION = 0;
const int OBS_TYPE_DISTANCES = 1;

typedef struct {
    float coverage_percentage;
    float total_reward;
    float collisions;
    float episode_return;
    float episode_length;
    float n; // Required as the last field
} Log;

typedef struct {
    Log log;                     // Required field
    float* observations;         // Required field - 2D: [x, y] or 4D: [dist_left, dist_right, dist_up, dist_down]
    int* actions;                // Required field - 4 discrete actions: UP, DOWN, LEFT, RIGHT  
    float* rewards;              // Required field
    unsigned char* terminals;    // Required field

    // Environment state
    int x, y;                    // Agent position (integer grid coordinates)
    int grid_width, grid_height; // Grid dimensions
    unsigned char* visited;      // 0 = unvisited, 1 = visited
    int total_cells;             // Total cells in grid
    int visited_count;           // Number of cells visited
    int max_steps;
    int tick;
    
    // Configuration
    int obs_type;                // 0 = position, 1 = distances
    int obs_size;                // 2 or 4 depending on obs_type
    
    // Wall collision tracking
    int wall_collisions;
} CoverageGrid;

void add_log(CoverageGrid* env) {
    env->log.coverage_percentage = (env->total_cells > 0) ? 
        (float)env->visited_count / (float)env->total_cells * 100.0f : 0.0f;
    env->log.total_reward += env->rewards[0];
    env->log.collisions += env->wall_collisions;
    env->log.episode_length += env->tick;
    env->log.episode_return += env->rewards[0];
    env->log.n++;
}

void update_observations(CoverageGrid* env) {
    if (env->obs_type == OBS_TYPE_POSITION) {
        // Normalize position to [0, 1]
        env->observations[0] = (float)env->x / (float)(env->grid_width - 1);
        env->observations[1] = (float)env->y / (float)(env->grid_height - 1);
    } else if (env->obs_type == OBS_TYPE_DISTANCES) {
        // Distance to walls, normalized to [0, 1]
        float max_dist = fmaxf(env->grid_width, env->grid_height);
        env->observations[0] = (float)env->x / max_dist;           // distance to left wall
        env->observations[1] = (float)(env->grid_width - 1 - env->x) / max_dist;  // distance to right wall
        env->observations[2] = (float)env->y / max_dist;           // distance to top wall  
        env->observations[3] = (float)(env->grid_height - 1 - env->y) / max_dist; // distance to bottom wall
    }
}

void c_reset(CoverageGrid* env) {
    // Reset agent to corner of grid
    env->x = 0;
    env->y = 0;
    env->tick = 0;
    env->wall_collisions = 0;
    
    // Reset visited grid
    memset(env->visited, CELL_UNVISITED, env->grid_width * env->grid_height);
    env->visited_count = 0;
    
    // Mark starting position as visited
    env->visited[env->y * env->grid_width + env->x] = CELL_VISITED;
    env->visited_count = 1;
    
    // Update initial observations
    update_observations(env);
}

void c_step(CoverageGrid* env) {
    env->tick += 1;
    env->rewards[0] = 0.0f;
    env->terminals[0] = 0;
    
    // Process action
    int action = env->actions[0];
    int new_x = env->x;
    int new_y = env->y;
    
    switch (action) {
        case ACTION_UP:
            new_y = env->y - 1;
            break;
        case ACTION_DOWN:
            new_y = env->y + 1;
            break;
        case ACTION_LEFT:
            new_x = env->x - 1;
            break;
        case ACTION_RIGHT:
            new_x = env->x + 1;
            break;
        default:
            // Invalid action, stay in place
            break;
    }
    
    // Check if new position is within bounds
    if (new_x >= 0 && new_x < env->grid_width && 
        new_y >= 0 && new_y < env->grid_height) {
        
        // Move to new position
        env->x = new_x;
        env->y = new_y;
        
        // Check if this cell was already visited
        int cell_idx = env->y * env->grid_width + env->x;
        if (env->visited[cell_idx] == CELL_UNVISITED) {
            // Reward for visiting new cell
            env->visited[cell_idx] = CELL_VISITED;
            env->visited_count++;
            env->rewards[0] += 1.0f;
        } else {
            // Small penalty for revisiting cell
            env->rewards[0] -= 0.01f;
        }
        
        // Check if all cells have been visited
        if (env->visited_count >= env->total_cells) {
            env->rewards[0] += 10.0f;  // Large bonus for complete coverage
            env->terminals[0] = 1;
            add_log(env);
            c_reset(env);
            return;
        }
        
    } else {
        // Wall collision - no movement, penalty
        env->wall_collisions++;
        env->rewards[0] -= 0.1f;
    }
    
    // Update observations
    update_observations(env);
    
    // Check for episode termination
    if (env->tick >= env->max_steps) {
        env->terminals[0] = 1;
        add_log(env);
        c_reset(env);
    }
}

void c_render(CoverageGrid* env) {
    const int WINDOW_WIDTH = 640;
    const int WINDOW_HEIGHT = 480;
    const int CELL_SIZE = 40;
    
    if (!IsWindowReady()) {
        InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PufferLib Coverage Grid");
        SetTargetFPS(30);
    }
    
    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }
    
    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});
    
    // Calculate offset to center the grid
    int grid_pixel_width = env->grid_width * CELL_SIZE;
    int grid_pixel_height = env->grid_height * CELL_SIZE;
    int offset_x = (WINDOW_WIDTH - grid_pixel_width) / 2;
    int offset_y = (WINDOW_HEIGHT - grid_pixel_height) / 2;
    
    // Draw grid cells
    for (int y = 0; y < env->grid_height; y++) {
        for (int x = 0; x < env->grid_width; x++) {
            int cell_idx = y * env->grid_width + x;
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
    
    // Draw agent
    int agent_pixel_x = offset_x + env->x * CELL_SIZE + CELL_SIZE / 2;
    int agent_pixel_y = offset_y + env->y * CELL_SIZE + CELL_SIZE / 2;
    DrawCircle(agent_pixel_x, agent_pixel_y, CELL_SIZE / 3, (Color){255, 255, 0, 255});
    
    // Draw status information
    float coverage_pct = (env->total_cells > 0) ? 
        (float)env->visited_count / (float)env->total_cells * 100.0f : 0.0f;
    char status_text[200];
    snprintf(status_text, sizeof(status_text), 
        "Pos: (%d,%d) | Coverage: %d/%d (%.1f%%) | Collisions: %d | Step: %d",
        env->x, env->y, env->visited_count, env->total_cells, coverage_pct, 
        env->wall_collisions, env->tick);
    DrawText(status_text, 10, 10, 20, (Color){241, 241, 241, 255});
    
    EndDrawing();
}

void c_close(CoverageGrid* env) {
    if (env->visited) {
        free(env->visited);
        env->visited = NULL;
    }
    if (IsWindowReady()) {
        CloseWindow();
    }
}