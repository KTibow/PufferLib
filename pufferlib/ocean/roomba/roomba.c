#include "roomba.h"

void add_log(Roomba* env) {
    env->log.coverage += (env->speed > 0.1f) ? 1.0f : 0.0f;  // Moving = exploring
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

void c_reset(Roomba* env) {
    // Reset roomba to center of room with random orientation
    env->x = env->room_width / 2.0f;
    env->y = env->room_height / 2.0f;
    env->theta = (float)(rand()) / RAND_MAX * 2.0f * PI;
    env->speed = 0.0f;
    env->angular_velocity = 0.0f;
    env->tick = 0;
    env->bump_sensor = 0;
    
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
    while (env->theta < 0) env->theta += 2.0f * PI;
    while (env->theta >= 2.0f * PI) env->theta -= 2.0f * PI;
    
    // Check wall collisions
    if (env->x - ROOMBA_RADIUS <= 0.0f || env->x + ROOMBA_RADIUS >= env->room_width ||
        env->y - ROOMBA_RADIUS <= 0.0f || env->y + ROOMBA_RADIUS >= env->room_height) {
        
        env->bump_sensor = 1;
        env->rewards[0] = -0.1f;  // Penalty for hitting wall
        
        // Push roomba back inside bounds
        env->x = fmaxf(ROOMBA_RADIUS, fminf(env->room_width - ROOMBA_RADIUS, env->x));
        env->y = fmaxf(ROOMBA_RADIUS, fminf(env->room_height - ROOMBA_RADIUS, env->y));
        
        // Stop forward movement
        env->speed = 0.0f;
    } else if (env->speed > 0.1f) {
        env->rewards[0] = 0.01f;  // Small reward for moving
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
    
    // Draw sensor information
    char sensor_text[200];
    snprintf(sensor_text, sizeof(sensor_text), 
        "Bump: %d | Pos: (%.1f,%.1f) | θ: %.1f° | Speed: %.2f",
        env->bump_sensor, env->x, env->y, env->theta * 180.0f / PI, env->speed);
    DrawText(sensor_text, 10, 10, 20, (Color){241, 241, 241, 255});
    
    EndDrawing();
}

void c_close(Roomba* env) {
    if (IsWindowReady()) {
        CloseWindow();
    }
}