#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "raylib.h"

const float ROOMBA_RADIUS = 17.425f; // cm
const float MAX_WHEEL_SPEED = 50.0f; // cm/s
const float WHEELBASE = 23.5f; // cm
const float FRICTION = 0.01f;
const float BUMP_CLEARANCE = 1.0f; // cm - distance to back away before bumper clears
const float BRUSH_WIDTH = 16.51f; // cm - 6.5" brush width
const float BRUSH_DEPTH = 7.62f; // cm - 3" brush depth
const float dt = 0.1f;

typedef struct {
    float collisions;
    float episode_return;
    float episode_length;
    float dirt_collected;
    float n; // Required as the last field
} Log;

typedef struct {
    Log log;                     // Required field
    float* observations;         // Required field. 4D: [left_bumper_importance, right_bumper_importance, left_wheel_speed, right_wheel_speed]
    float* actions;              // Required field. 2D: [left_wheel_speed, right_wheel_speed] in cm/s
    float* rewards;              // Required field
    unsigned char* terminals;    // Required field

    // Roomba state
    float x, y;                  // Position in cm
    float theta;                 // Orientation in radians
    float left_wheel_speed;      // Current left wheel speed in cm/s
    float right_wheel_speed;     // Current right wheel speed in cm/s

    // Environment parameters
    float room_width;            // Room width in cm
    float room_height;           // Room height in cm
    int max_steps;
    int tick;

    // Sensors
    int left_bumper;             // 1 if left bumper pressed, 0 otherwise
    int right_bumper;            // 1 if right bumper pressed, 0 otherwise
    float left_bumper_importance;     // Importance of left bumper hit: 2.0=just hit, 0.0=not hit recently
    float right_bumper_importance;    // Importance of right bumper hit: 2.0=just hit, 0.0=not hit recently
} Roomba;

void add_log(Roomba* env) {
    env->log.collisions += (env->left_bumper || env->right_bumper) ? 1 : 0;
    env->log.episode_length += env->tick;
    env->log.episode_return += env->rewards[0];
    env->log.n++;
}

void c_reset(Roomba* env) {
    // Reset roomba to center of room with random orientation
    env->x = env->room_width / 2.0f;
    env->y = env->room_height / 2.0f;
    env->theta = (float)(rand()) / RAND_MAX * 2.0f * PI;
    env->left_wheel_speed = 0.0f;
    env->right_wheel_speed = 0.0f;
    env->tick = 0;
    env->left_bumper = 0;
    env->right_bumper = 0;
    env->left_bumper_importance = 0.0f;
    env->right_bumper_importance = 0.0f;

    // Set initial observations: [left_bumper_importance, right_bumper_importance, left_wheel_speed, right_wheel_speed]
    env->observations[0] = env->left_bumper_importance;
    env->observations[1] = env->right_bumper_importance;
    env->observations[2] = env->left_wheel_speed;
    env->observations[3] = env->right_wheel_speed;
}

void c_step(Roomba* env) {
    env->tick += 1;
    env->left_bumper = 0;
    env->right_bumper = 0;
    env->rewards[0] = 0.0f;
    env->terminals[0] = 0;

    // Decrement bumper importance over time (minimum 0.0)
    env->left_bumper_importance = fmaxf(0.0f, env->left_bumper_importance - dt);
    env->right_bumper_importance = fmaxf(0.0f, env->right_bumper_importance - dt);

    // Get wheel speed commands from actions (clamped to [-50, 50] cm/s)
    float target_left = fmaxf(-MAX_WHEEL_SPEED, fminf(MAX_WHEEL_SPEED, env->actions[0]));
    float target_right = fmaxf(-MAX_WHEEL_SPEED, fminf(MAX_WHEEL_SPEED, env->actions[1]));

    // Apply wheel speed commands with friction
    env->left_wheel_speed = env->left_wheel_speed * FRICTION + target_left * (1.0f - FRICTION);
    env->right_wheel_speed = env->right_wheel_speed * FRICTION + target_right * (1.0f - FRICTION);

    // Convert wheel speeds to linear and angular velocity
    // Differential drive kinematics: all units in cm and cm/s
    float linear_velocity = (env->left_wheel_speed + env->right_wheel_speed) / 2.0f; // cm/s
    float angular_velocity = (env->right_wheel_speed - env->left_wheel_speed) / WHEELBASE; // rad/s

    // Check wall proximity for realistic collision constraints
    int near_wall = (env->x <= ROOMBA_RADIUS - BUMP_CLEARANCE ||
                     env->x >= env->room_width - ROOMBA_RADIUS + BUMP_CLEARANCE ||
                     env->y <= ROOMBA_RADIUS - BUMP_CLEARANCE ||
                     env->y >= env->room_height - ROOMBA_RADIUS + BUMP_CLEARANCE);

    // Prevent going into walls
    if (near_wall) {
      // Calculate direction toward room center
      float center_x = env->room_width / 2.0f;
      float center_y = env->room_height / 2.0f;
      float to_center_x = center_x - env->x;
      float to_center_y = center_y - env->y;
      float to_center_dist = sqrtf(to_center_x * to_center_x + to_center_y * to_center_y);

      if (to_center_dist > 0) {
          to_center_x /= to_center_dist;
          to_center_y /= to_center_dist;

          // Project intended movement onto "away from wall" direction
          float intended_x = linear_velocity * cosf(env->theta);
          float intended_y = linear_velocity * sinf(env->theta);
          float toward_center_component = intended_x * to_center_x + intended_y * to_center_y;

          // Only allow movement if it's toward center, otherwise block it
          if (toward_center_component <= 0.01) {
              linear_velocity = 0; // Don't allow movement toward walls
          }
      }
    }

    // Update position and orientation
    env->x += linear_velocity * cosf(env->theta) * dt; // cm/s * s = cm
    env->y += linear_velocity * sinf(env->theta) * dt; // cm/s * s = cm
    env->theta += angular_velocity * dt; // rad/s * s = rad

    // Prevent getting stuck in walls
    if (env->x < ROOMBA_RADIUS - BUMP_CLEARANCE) env->x = ROOMBA_RADIUS - BUMP_CLEARANCE;
    if (env->x > env->room_width - ROOMBA_RADIUS + BUMP_CLEARANCE) env->x = env->room_width - ROOMBA_RADIUS + BUMP_CLEARANCE;
    if (env->y < ROOMBA_RADIUS - BUMP_CLEARANCE) env->y = ROOMBA_RADIUS;
    if (env->y > env->room_height - ROOMBA_RADIUS + BUMP_CLEARANCE) env->y = env->room_height - ROOMBA_RADIUS + BUMP_CLEARANCE;

    // Normalize angle to [0, 2π]
    while (env->theta < 0) env->theta += 2.0f * PI;
    while (env->theta >= 2.0f * PI) env->theta -= 2.0f * PI;

    // Set bump sensors based on front bumper collisions
    // Check left bumper arc
    int left_hit = 0;
    float angle_offset_start = env->theta - (PI/36);
    float angle_offset_end = env->theta - (PI/2);
    for (float angle_offset = angle_offset_start; angle_offset > angle_offset_end; angle_offset -= PI/36) {
        float sample_x = env->x + ROOMBA_RADIUS * cosf(angle_offset);
        float sample_y = env->y + ROOMBA_RADIUS * sinf(angle_offset);
        if (sample_x <= 0 || sample_x >= env->room_width ||
            sample_y <= 0 || sample_y >= env->room_height) {
            left_hit = 1;
            break;
        }
    }

    // Check right bumper arc
    int right_hit = 0;
    angle_offset_start = env->theta + (PI/36);
    angle_offset_end = env->theta + (PI/2);
    for (float angle_offset = angle_offset_start; angle_offset < angle_offset_end; angle_offset += PI/36) {
        float sample_x = env->x + ROOMBA_RADIUS * cosf(angle_offset);
        float sample_y = env->y + ROOMBA_RADIUS * sinf(angle_offset);
        if (sample_x <= 0 || sample_x >= env->room_width ||
            sample_y <= 0 || sample_y >= env->room_height) {
            right_hit = 1;
            break;
        }
    }

    env->left_bumper = left_hit;
    env->right_bumper = right_hit;

    if (left_hit) env->left_bumper_importance = 2.0f;
    if (right_hit) env->right_bumper_importance = 2.0f;

    // Proportional rewards
    float movement_reward;
    if (linear_velocity < 0) {
        movement_reward = fminf(1.0f, fabsf(linear_velocity));
    } else {
        movement_reward = fabsf(linear_velocity);
    }
    movement_reward /= 50.0f;
    movement_reward *= 0.5f;
    env->rewards[0] = movement_reward;

    // Update observations: [left_bumper_importance, right_bumper_importance, left_wheel_speed, right_wheel_speed]
    env->observations[0] = env->left_bumper_importance;
    env->observations[1] = env->right_bumper_importance;
    env->observations[2] = env->left_wheel_speed;
    env->observations[3] = env->right_wheel_speed;

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
    const float SCALE = 1.0f;  // Pixels per cm - reasonable room size on screen

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

    // Draw room boundaries with thick lines for visibility
    DrawRectangleLines(offset_x, offset_y, room_pixel_width, room_pixel_height, (Color){241, 241, 241, 255});
    DrawRectangleLines(offset_x-1, offset_y-1, room_pixel_width+2, room_pixel_height+2, (Color){241, 241, 241, 255});
    DrawRectangleLines(offset_x+1, offset_y+1, room_pixel_width-2, room_pixel_height-2, (Color){241, 241, 241, 255});

    // Draw roomba
    float roomba_x = offset_x + env->x * SCALE;
    float roomba_y = offset_y + env->y * SCALE;
    float roomba_radius = ROOMBA_RADIUS * SCALE;

    Color roomba_color = (env->left_bumper || env->right_bumper) ? (Color){187, 0, 0, 255} : (Color){0, 187, 187, 255};
    DrawCircle(roomba_x, roomba_y, roomba_radius, roomba_color);

    // Draw brush collection area
    float brush_center_x = offset_x + (env->x - (BRUSH_DEPTH / 2.0f) * cosf(env->theta)) * SCALE;
    float brush_center_y = offset_y + (env->y - (BRUSH_DEPTH / 2.0f) * sinf(env->theta)) * SCALE;

    float brush_width = BRUSH_WIDTH * SCALE;
    float brush_depth = BRUSH_DEPTH * SCALE;

    // Calculate brush rectangle corners
    float cos_theta = cosf(env->theta);
    float sin_theta = sinf(env->theta);
    float half_width = brush_width / 2.0f;
    float half_depth = brush_depth / 2.0f;

    Vector2 corners[4] = {
        {brush_center_x + (-half_depth * cos_theta - (-half_width) * sin_theta),
         brush_center_y + (-half_depth * sin_theta + (-half_width) * cos_theta)},
        {brush_center_x + (half_depth * cos_theta - (-half_width) * sin_theta),
         brush_center_y + (half_depth * sin_theta + (-half_width) * cos_theta)},
        {brush_center_x + (half_depth * cos_theta - half_width * sin_theta),
         brush_center_y + (half_depth * sin_theta + half_width * cos_theta)},
        {brush_center_x + (-half_depth * cos_theta - half_width * sin_theta),
         brush_center_y + (-half_depth * sin_theta + half_width * cos_theta)}
    };

    // Draw brush area outline
    for (int i = 0; i < 4; i++) {
        DrawLineEx(corners[i], corners[(i+1)%4], 2.0f, (Color){255, 255, 0, 128}); // Semi-transparent yellow
    }

    // Draw orientation indicator
    float indicator_length = roomba_radius * 0.8f;
    float end_x = roomba_x + indicator_length * cosf(env->theta);
    float end_y = roomba_y + indicator_length * sinf(env->theta);
    DrawLineEx(
        (Vector2){roomba_x, roomba_y},
        (Vector2){end_x, end_y},
        6.0f,
        (Color){255, 255, 255, 255}
    );

    // Draw sensor information
    char sensor_text[300];
    snprintf(sensor_text, sizeof(sensor_text),
        "(%.0f,%.0f) facing %.1f° | %.0f/s, %.0f/s | bump %d%d",
        env->x, env->y, env->theta * 180.0f / PI,
        env->left_wheel_speed, env->right_wheel_speed,
        env->left_bumper, env->right_bumper);
    DrawText(sensor_text, 10, 10, 20, (Color){241, 241, 241, 255});

    EndDrawing();
}

void c_close(Roomba* env) {
    if (IsWindowReady()) {
        CloseWindow();
    }
}
