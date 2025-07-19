#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "raylib.h"

const unsigned char STOP = 0;
const unsigned char FORWARD = 1;
const unsigned char BACKWARD = 2;
const unsigned char TURN_LEFT = 3;
const unsigned char TURN_RIGHT = 4;

const float ROOMBA_RADIUS = 0.2f;
const float MAX_SPEED = 1.0f;
const float TURN_RATE = 2.0f;
const float FRICTION = 0.95f;

typedef struct {
    float coverage_percentage;
    float dirt_cleaned;
    float collisions;
    float episode_return;
    float episode_length;
    float n; // Required as the last field
} Log;

typedef struct {
    Log log;                     // Required field
    float* observations;         // Required field. 5D: [bump, front_dist, right_dist, back_dist, left_dist]
    int* actions;                // Required field. Discrete actions 0-4
    float* rewards;              // Required field
    unsigned char* terminals;    // Required field

    // Roomba state
    float x, y;                  // Position in continuous space
    float theta;                 // Orientation in radians
    float speed;                 // Current forward speed
    float angular_velocity;      // Current turning rate

    // Environment parameters
    float room_width;
    float room_height;
    int max_steps;
    int tick;

    // Sensors
    int bump_sensor;             // 1 if collision this step, 0 otherwise
    float wall_distances[4];     // front, right, back, left distances to walls
    
    // Coverage tracking (simple grid)
    int grid_width, grid_height; // Grid dimensions for coverage tracking
    unsigned char* coverage_grid; // 0 = dirty, 1 = cleaned
    int total_dirt;              // Total dirt tiles at start
    int cleaned_dirt;            // Number of dirt tiles cleaned
} Roomba;

void add_log(Roomba* env) {
    env->log.coverage_percentage = (env->total_dirt > 0) ? 
        (float)env->cleaned_dirt / (float)env->total_dirt * 100.0f : 0.0f;
    env->log.dirt_cleaned += env->cleaned_dirt;
    env->log.collisions += env->bump_sensor;
    env->log.episode_length += env->tick;
    env->log.episode_return += env->rewards[0];
    env->log.n++;
}

void update_wall_distances(Roomba* env) {
    // Calculate distances to walls in 4 directions relative to roomba orientation
    float cos_theta = cosf(env->theta);
    float sin_theta = sinf(env->theta);
    
    // Front distance (direction roomba is facing)
    float front_x = env->x + cos_theta * env->room_width;
    float front_y = env->y + sin_theta * env->room_height;
    env->wall_distances[0] = fminf(
        fminf(env->room_width - env->x, env->x),
        fminf(env->room_height - env->y, env->y)
    );
    
    // Simplified: distance to nearest wall in each cardinal direction
    env->wall_distances[0] = env->room_width - env->x;  // front (right wall)
    env->wall_distances[1] = env->room_height - env->y; // right (top wall)  
    env->wall_distances[2] = env->x;                    // back (left wall)
    env->wall_distances[3] = env->y;                    // left (bottom wall)
    
    // Clamp to reasonable sensor range
    for (int i = 0; i < 4; i++) {
        env->wall_distances[i] = fmaxf(0.0f, fminf(env->wall_distances[i], 5.0f));
    }
}

void init_dirt_grid(Roomba* env) {
    // Create a grid for tracking dirt/coverage
    env->grid_width = (int)(env->room_width * 2.0f);  // 0.5 unit resolution
    env->grid_height = (int)(env->room_height * 2.0f);
    
    if (!env->coverage_grid) {
        env->coverage_grid = (unsigned char*)calloc(env->grid_width * env->grid_height, sizeof(unsigned char));
    }
    
    // Reset all tiles to dirty
    memset(env->coverage_grid, 0, env->grid_width * env->grid_height);
    env->total_dirt = env->grid_width * env->grid_height;
    env->cleaned_dirt = 0;
}

void c_reset(Roomba* env) {
    // Reset roomba to center of room with random orientation
    env->x = env->room_width / 2.0f;
    env->y = env->room_height / 2.0f;
    env->theta = (float)(rand()) / RAND_MAX * 2.0f * 3.14159265359f;
    env->speed = 0.0f;
    env->angular_velocity = 0.0f;
    env->tick = 0;
    env->bump_sensor = 0;
    
    // Initialize dirt grid
    init_dirt_grid(env);
    
    // Update initial sensor readings
    update_wall_distances(env);
    
    // Set observations
    env->observations[0] = (float)env->bump_sensor;
    for (int i = 0; i < 4; i++) {
        env->observations[i + 1] = env->wall_distances[i] / 5.0f; // Normalize to [0,1]
    }
}

void c_step(Roomba* env) {
    env->tick += 1;
    env->bump_sensor = 0;
    env->rewards[0] = 0.0f;
    env->terminals[0] = 0;
    
    // Process action
    int action = env->actions[0];
    float target_speed = 0.0f;
    float target_angular_vel = 0.0f;
    
    switch (action) {
        case STOP:
            target_speed = 0.0f;
            target_angular_vel = 0.0f;
            break;
        case FORWARD:
            target_speed = MAX_SPEED;
            target_angular_vel = 0.0f;
            break;
        case BACKWARD:
            target_speed = -MAX_SPEED * 0.5f;
            target_angular_vel = 0.0f;
            break;
        case TURN_LEFT:
            target_speed = MAX_SPEED * 0.3f;
            target_angular_vel = TURN_RATE;
            break;
        case TURN_RIGHT:
            target_speed = MAX_SPEED * 0.3f;
            target_angular_vel = -TURN_RATE;
            break;
    }
    
    // Update velocities with simple dynamics
    env->speed = env->speed * FRICTION + target_speed * (1.0f - FRICTION);
    env->angular_velocity = env->angular_velocity * FRICTION + target_angular_vel * (1.0f - FRICTION);
    
    // Update position and orientation
    float dt = 0.1f;  // Fixed time step
    env->x += env->speed * cosf(env->theta) * dt;
    env->y += env->speed * sinf(env->theta) * dt;
    env->theta += env->angular_velocity * dt;
    
    // Normalize angle to [0, 2π]
    while (env->theta < 0) env->theta += 2.0f * 3.14159265359f;
    while (env->theta >= 2.0f * 3.14159265359f) env->theta -= 2.0f * 3.14159265359f;
    
    // Check if roomba cleaned any dirt at current position
    int grid_x = (int)(env->x * 2.0f);  // Convert to grid coordinates
    int grid_y = (int)(env->y * 2.0f);
    if (grid_x >= 0 && grid_x < env->grid_width && grid_y >= 0 && grid_y < env->grid_height) {
        int grid_idx = grid_y * env->grid_width + grid_x;
        if (env->coverage_grid[grid_idx] == 0) {  // If tile is dirty
            env->coverage_grid[grid_idx] = 1;     // Clean it
            env->cleaned_dirt++;
            env->rewards[0] += 0.1f;  // Reward for cleaning dirt
        }
    }
    
    // Check wall collisions
    if (env->x - ROOMBA_RADIUS <= 0.0f || env->x + ROOMBA_RADIUS >= env->room_width ||
        env->y - ROOMBA_RADIUS <= 0.0f || env->y + ROOMBA_RADIUS >= env->room_height) {
        
        env->bump_sensor = 1;
        env->rewards[0] -= 0.1f;  // Penalty for hitting wall
        
        // Push roomba back inside bounds
        env->x = fmaxf(ROOMBA_RADIUS, fminf(env->room_width - ROOMBA_RADIUS, env->x));
        env->y = fmaxf(ROOMBA_RADIUS, fminf(env->room_height - ROOMBA_RADIUS, env->y));
        
        // Stop forward movement
        env->speed = 0.0f;
    } else if (env->speed > 0.1f) {
        env->rewards[0] += 0.01f;  // Small reward for moving
    }
    
    // Update sensor readings
    update_wall_distances(env);
    
    // Update observations
    env->observations[0] = (float)env->bump_sensor;
    for (int i = 0; i < 4; i++) {
        env->observations[i + 1] = env->wall_distances[i] / 5.0f; // Normalize to [0,1]
    }
    
    // Check for episode termination
    if (env->tick >= env->max_steps) {
        env->terminals[0] = 1;
        add_log(env);
        c_reset(env);
    }
}

void c_render(Roomba* env) {
    const int WINDOW_WIDTH = 800;
    const int WINDOW_HEIGHT = 600;
    const float SCALE = 80.0f;  // Pixels per unit
    
    if (!IsWindowReady()) {
        InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PufferLib Roomba");
        SetTargetFPS(30);
    }
    
    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }
    
    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});
    
    // Draw room boundaries
    float room_pixel_width = env->room_width * SCALE;
    float room_pixel_height = env->room_height * SCALE;
    float offset_x = (WINDOW_WIDTH - room_pixel_width) / 2.0f;
    float offset_y = (WINDOW_HEIGHT - room_pixel_height) / 2.0f;
    
    DrawRectangleLines(offset_x, offset_y, room_pixel_width, room_pixel_height, (Color){241, 241, 241, 255});
    
    // Draw roomba
    float roomba_x = offset_x + env->x * SCALE;
    float roomba_y = offset_y + env->y * SCALE;
    float roomba_radius = ROOMBA_RADIUS * SCALE;
    
    Color roomba_color = env->bump_sensor ? (Color){187, 0, 0, 255} : (Color){0, 187, 187, 255};
    DrawCircle(roomba_x, roomba_y, roomba_radius, roomba_color);
    
    // Draw orientation indicator
    float indicator_length = roomba_radius * 0.8f;
    float end_x = roomba_x + indicator_length * cosf(env->theta);
    float end_y = roomba_y + indicator_length * sinf(env->theta);
    DrawLine(roomba_x, roomba_y, end_x, end_y, (Color){255, 255, 255, 255});
    
    // Draw dirt grid
    float grid_scale = SCALE / 2.0f;  // Grid cells are 0.5 units
    for (int gy = 0; gy < env->grid_height; gy++) {
        for (int gx = 0; gx < env->grid_width; gx++) {
            int grid_idx = gy * env->grid_width + gx;
            if (env->coverage_grid[grid_idx] == 0) {  // Dirty tile
                float dirt_x = offset_x + gx * grid_scale;
                float dirt_y = offset_y + gy * grid_scale;
                DrawRectangle(dirt_x, dirt_y, grid_scale, grid_scale, (Color){100, 50, 0, 100});
            }
        }
    }
    
    // Draw sensor information
    float coverage_pct = (env->total_dirt > 0) ? 
        (float)env->cleaned_dirt / (float)env->total_dirt * 100.0f : 0.0f;
    char sensor_text[200];
    snprintf(sensor_text, sizeof(sensor_text), 
        "Bump: %d | Pos: (%.1f,%.1f) | θ: %.1f° | Speed: %.2f | Coverage: %.1f%%",
        env->bump_sensor, env->x, env->y, env->theta * 180.0f / 3.14159265359f, env->speed, coverage_pct);
    DrawText(sensor_text, 10, 10, 20, (Color){241, 241, 241, 255});
    
    EndDrawing();
}

void c_close(Roomba* env) {
    if (env->coverage_grid) {
        free(env->coverage_grid);
        env->coverage_grid = NULL;
    }
    if (IsWindowReady()) {
        CloseWindow();
    }
}
