#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "raylib.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_TRAIL 500
#define PIXEL_SCALE 1.0f

// Environment constants (in mm)
#define ROOM_SIZE 1500.0f       // Fixed square room for simplicity
#define ROOMBA_OUTER_RADIUS 150.0f  // Outer radius - bumper activation zone
#define ROOMBA_INNER_RADIUS 120.0f  // Inner radius - movement boundary
#define MAX_SPEED 250.0f        // mm/s max wheel speed
#define WHEELBASE 100.0f        // Distance between wheels
// #define SENSOR_RANGE 500.0f     // How far sensors can see
#define dt 0.05f                // Time step

typedef struct {
    float episode_return;
    float episode_length;
    float coverage_percentage;
    float n;
} Log;

typedef struct {
    float x, y;
} Point;

typedef struct {
    Log log;
    float* observations;     // 2D: [left_bumper, right_bumper] binary 0/1
    float* actions;          // 2D: [left_wheel, right_wheel] in [-1, 1]
    float* rewards;
    unsigned char* terminals;

    // Robot state
    float x, y, angle;       // Position and orientation
    float vx, vy, vangle;    // Velocities (for smooth movement)

    // Bumper sensors
    int left_bumper;         // 1 if left bumper pressed, 0 otherwise
    int right_bumper;        // 1 if right bumper pressed, 0 otherwise

    // Environment
    int tick;
    int max_steps;
    float episode_return;
    float last_coverage;     // Track coverage for reward calculation

    // Visualization
    Point trail[MAX_TRAIL];
    int trail_idx;
    int trail_count;
    float coverage_grid[30][30];  // Track where robot has been
} Roomba;

// Helper: Normalize angle to [-PI, PI]
float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// Helper: Check if point is near wall
// float distance_to_wall(float x, float y, float angle) {
//     // Cast a ray from (x,y) in direction angle and find distance to nearest wall
//     float dx = cosf(angle);
//     float dy = sinf(angle);
//     float dist = 0;

//     while (dist < SENSOR_RANGE) {
//         float test_x = x + dx * dist;
//         float test_y = y + dy * dist;

//         // Check if we hit a wall
//         if (test_x <= ROOMBA_RADIUS || test_x >= ROOM_SIZE - ROOMBA_RADIUS ||
//             test_y <= ROOMBA_RADIUS || test_y >= ROOM_SIZE - ROOMBA_RADIUS) {
//             return dist;
//         }
//         dist += 1.0f;
//     }
//     return SENSOR_RANGE;
// }

// Helper: Estimate coverage percentage
float estimate_coverage(Roomba* env) {
    int covered = 0;
    int total = 0;
    for (int i = 0; i < 30; i++) {
        for (int j = 0; j < 30; j++) {
            total++;
            if (env->coverage_grid[i][j] > 0) covered++;
        }
    }
    return (float)covered / total;
}

void c_reset(Roomba* env) {
    // Make the robot start at bottom left
    float margin = ROOMBA_INNER_RADIUS + 5; // Small margin from wall
    env->x = margin;
    env->y = ROOM_SIZE - margin;
    env->angle = -M_PI / 2.0f; // Facing down
    env->vx = 0;
    env->vy = 0;
    env->vangle = 0;

    // Reset bumpers
    env->left_bumper = 0;
    env->right_bumper = 0;

    // Reset counters
    env->tick = 0;
    env->episode_return = 0;
    env->trail_idx = 0;
    env->trail_count = 0;
    env->last_coverage = 0;

    // Clear coverage grid
    memset(env->coverage_grid, 0, sizeof(env->coverage_grid));

    // Set initial observations: [left_bumper, right_bumper]
    env->observations[0] = 0.0f;  // Left bumper not pressed initially
    env->observations[1] = 0.0f;  // Right bumper not pressed initially
}

void c_step(Roomba* env) {
    env->tick++;

    // Reset bumpers each step
    env->left_bumper = 0;
    env->right_bumper = 0;

    // Get wheel speeds from actions
    float left_speed = env->actions[0] * MAX_SPEED;
    float right_speed = env->actions[1] * MAX_SPEED;

    // Calculate robot motion (differential drive)
    float linear_vel = (left_speed + right_speed) / 2.0f;
    float angular_vel = (right_speed - left_speed) / WHEELBASE;

    // Update velocities with some damping for smoother motion
    float damping = 0.9f;  // Increased from 0.8 for more responsive control
    env->vx = damping * (linear_vel * cosf(env->angle));
    env->vy = damping * (linear_vel * sinf(env->angle));
    env->vangle = damping * angular_vel;

    // Update position
    float new_x = env->x + env->vx * dt;
    float new_y = env->y + env->vy * dt;
    float new_angle = normalize_angle(env->angle + env->vangle * dt);

    // Bumper collision detection - check if bumper points hit walls (outer radius)
    int hit_wall = 0;

    // Check left bumper (front-left of roomba) - uses outer radius
    // Cast a range of points along the left bumper arc
    for (float offset = 0; offset <= M_PI/2; offset += M_PI/16) {
        float angle = new_angle + offset;
        float left_bump_x = new_x + ROOMBA_OUTER_RADIUS * cosf(angle);
        float left_bump_y = new_y + ROOMBA_OUTER_RADIUS * sinf(angle);
        if (left_bump_x <= 0 || left_bump_x >= ROOM_SIZE ||
            left_bump_y <= 0 || left_bump_y >= ROOM_SIZE) {
            env->left_bumper = 1;
            break;
        }
    }

    // Check right bumper (front-right of roomba) - uses outer radius
    // Cast a range of points along the right bumper arc
    for (float offset = 0; offset <= M_PI/2; offset += M_PI/16) {
        float angle = new_angle - offset;
        float right_bump_x = new_x + ROOMBA_OUTER_RADIUS * cosf(angle);
        float right_bump_y = new_y + ROOMBA_OUTER_RADIUS * sinf(angle);
        if (right_bump_x <= 0 || right_bump_x >= ROOM_SIZE ||
            right_bump_y <= 0 || right_bump_y >= ROOM_SIZE) {
            env->right_bumper = 1;
            break;
        }
    }

    // Movement collision detection - robot stops at inner radius
    if (new_x < ROOMBA_INNER_RADIUS) {
        new_x = ROOMBA_INNER_RADIUS;
        env->vx = 0;
        hit_wall = 1;
    }
    if (new_x > ROOM_SIZE - ROOMBA_INNER_RADIUS) {
        new_x = ROOM_SIZE - ROOMBA_INNER_RADIUS;
        env->vx = 0;
        hit_wall = 1;
    }
    if (new_y < ROOMBA_INNER_RADIUS) {
        new_y = ROOMBA_INNER_RADIUS;
        env->vy = 0;
        hit_wall = 1;
    }
    if (new_y > ROOM_SIZE - ROOMBA_INNER_RADIUS) {
        new_y = ROOM_SIZE - ROOMBA_INNER_RADIUS;
        env->vy = 0;
        hit_wall = 1;
    }

    env->x = new_x;
    env->y = new_y;
    env->angle = new_angle;

    // Update coverage grid - mark all cells covered by the brush
    // The brush is as wide as the robot inner diameter
    float brush_radius = ROOMBA_INNER_RADIUS;
    int min_grid_x = (int)((env->x - brush_radius) / ROOM_SIZE * 30);
    int max_grid_x = (int)((env->x + brush_radius) / ROOM_SIZE * 30);
    int min_grid_y = (int)((env->y - brush_radius) / ROOM_SIZE * 30);
    int max_grid_y = (int)((env->y + brush_radius) / ROOM_SIZE * 30);

    // Clamp to grid bounds and mark covered
    if (min_grid_x < 0) min_grid_x = 0;
    if (max_grid_x >= 30) max_grid_x = 29;
    if (min_grid_y < 0) min_grid_y = 0;
    if (max_grid_y >= 30) max_grid_y = 29;

    for (int gx = min_grid_x; gx <= max_grid_x; gx++) {
        for (int gy = min_grid_y; gy <= max_grid_y; gy++) {
            // Check if this grid cell is actually within brush radius
            float cell_center_x = (gx + 0.5f) * ROOM_SIZE / 30;
            float cell_center_y = (gy + 0.5f) * ROOM_SIZE / 30;
            float dx = cell_center_x - env->x;
            float dy = cell_center_y - env->y;
            if (dx*dx + dy*dy <= brush_radius*brush_radius) {
                env->coverage_grid[gy][gx] = 1;
            }
        }
    }

    // Update trail for visualization
    if (env->tick % 2 == 0) {  // Record every other tick
        env->trail[env->trail_idx].x = env->x;
        env->trail[env->trail_idx].y = env->y;
        env->trail_idx = (env->trail_idx + 1) % MAX_TRAIL;
        if (env->trail_count < MAX_TRAIL) env->trail_count++;
    }

    // Calculate reward
    float reward = 0;

    // Main goal: explore new area
    float coverage = estimate_coverage(env);
    float new_coverage = coverage - env->last_coverage;
    reward += new_coverage * 1.0f;  // Reward for new coverage (as fraction of grid)
    env->last_coverage = coverage;

    // Learn boundaries: small wall penalty
    if (hit_wall) {
        reward -= 0.01f;
    }

    env->rewards[0] = reward;
    env->episode_return += reward;

    // Update observations: [left_bumper, right_bumper]
    env->observations[0] = env->left_bumper ? 1.0f : 0.0f;
    env->observations[1] = env->right_bumper ? 1.0f : 0.0f;

    // Check termination: 100% coverage or max steps
    if (env->tick >= env->max_steps || coverage >= 0.999f) {
        env->terminals[0] = 1;

        // Update logs
        env->log.episode_return += env->episode_return;
        env->log.episode_length += env->tick;
        env->log.coverage_percentage += coverage * 100.0f;  // Convert to percentage
        env->log.n++;

        c_reset(env);
    } else {
        env->terminals[0] = 0;
    }
}

void c_render(Roomba* env) {
    int window_size = ROOM_SIZE * PIXEL_SCALE;

    if (!IsWindowReady()) {
        InitWindow(window_size, window_size, "Simple Roomba");
        SetTargetFPS((int)(1.0f / dt));
    }

    if (WindowShouldClose() || IsKeyPressed(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground(BLACK);

    // Helper function to flip Y coordinate
    #define FLIP_Y(y) (window_size - (y))

    // Draw room
    DrawRectangleLines(0, 0, window_size, window_size, WHITE);

    // Draw coverage grid
    for (int i = 0; i < 30; i++) {
        for (int j = 0; j < 30; j++) {
            if (env->coverage_grid[i][j] > 0) {
                int x = j * window_size / 30;
                int y = FLIP_Y((i + 1) * window_size / 30);  // Flip and adjust
                DrawRectangle(x, y, window_size/30, window_size/30, (Color){0, 100, 0, 50});
            }
        }
    }

    // Draw trail
    for (int i = 1; i < env->trail_count; i++) {
        int prev = (env->trail_idx - env->trail_count + i - 1 + MAX_TRAIL) % MAX_TRAIL;
        int curr = (env->trail_idx - env->trail_count + i + MAX_TRAIL) % MAX_TRAIL;

        float alpha = (float)i / MAX_TRAIL;
        Color c = (Color){0, 200, 200, (int)(alpha * 150)};

        DrawLine(
            env->trail[prev].x * PIXEL_SCALE,
            FLIP_Y(env->trail[prev].y * PIXEL_SCALE),
            env->trail[curr].x * PIXEL_SCALE,
            FLIP_Y(env->trail[curr].y * PIXEL_SCALE),
            c
        );
    }

    // Draw roomba
    float screen_x = env->x * PIXEL_SCALE;
    float screen_y = FLIP_Y(env->y * PIXEL_SCALE);

    DrawCircle(screen_x, screen_y, ROOMBA_INNER_RADIUS * PIXEL_SCALE, DARKGREEN);
    DrawCircleLines(screen_x, screen_y, ROOMBA_OUTER_RADIUS * PIXEL_SCALE, GRAY);

    // Highlight bumper zones if active
    if (env->left_bumper || env->right_bumper) {
        Color bumper_color = RED;
        if (env->left_bumper) {
            // Adjust angles for flipped Y
            float start_angle = (-env->angle) * 180 / M_PI;
            float end_angle = (-env->angle - M_PI/2) * 180 / M_PI;
            DrawRing((Vector2){screen_x, screen_y},
                     ROOMBA_INNER_RADIUS * PIXEL_SCALE,
                     ROOMBA_OUTER_RADIUS * PIXEL_SCALE,
                     start_angle, end_angle, 16, bumper_color);
        }
        if (env->right_bumper) {
            float start_angle = (-env->angle + M_PI/2) * 180 / M_PI;
            float end_angle = (-env->angle) * 180 / M_PI;
            DrawRing((Vector2){screen_x, screen_y},
                     ROOMBA_INNER_RADIUS * PIXEL_SCALE,
                     ROOMBA_OUTER_RADIUS * PIXEL_SCALE,
                     start_angle, end_angle, 16, bumper_color);
        }
    }

    // Draw direction indicator (adjust for flipped Y)
    float end_x = env->x + cosf(env->angle) * ROOMBA_INNER_RADIUS * 0.8f;
    float end_y = env->y + sinf(env->angle) * ROOMBA_INNER_RADIUS * 0.8f;
    DrawLineEx(
        (Vector2){screen_x, screen_y},
        (Vector2){end_x * PIXEL_SCALE, FLIP_Y(end_y * PIXEL_SCALE)},
        3, WHITE
    );

    // UI text (unchanged)
    char text[256];
    snprintf(text, sizeof(text), "Coverage: %.1f%% | Reward: %.2f | Step: %d",
             estimate_coverage(env) * 100,
             env->episode_return, env->tick);
    DrawText(text, 10, 10, 20, WHITE);

    char bumper_text[128];
    snprintf(bumper_text, sizeof(bumper_text), "Bumpers: L:%s R:%s",
             env->left_bumper ? "HIT" : "OK",
             env->right_bumper ? "HIT" : "OK");
    DrawText(bumper_text, 10, 35, 16, env->left_bumper || env->right_bumper ? RED : GREEN);

    #undef FLIP_Y
    EndDrawing();
}

void c_close(Roomba* env) {
    if (IsWindowReady()) {
        CloseWindow();
    }
}
