#include <stdlib.h>
#include <math.h>
#include "raylib.h"

const Color PUFF_RED = (Color){187, 0, 0, 255};
const Color PUFF_CYAN = (Color){0, 187, 187, 255};
const Color PUFF_GREEN = (Color){0, 187, 0, 255};
const Color PUFF_WHITE = (Color){241, 241, 241, 241};
const Color PUFF_BACKGROUND = (Color){6, 24, 24, 255};

// Environment constants (all distances in millimeters)
#define ARENA_WIDTH 2000.0f       // 2 meters
#define ARENA_HEIGHT 1500.0f      // 1.5 meters
#define ROBOT_RADIUS 100.0f       // Robot body radius (10 cm)
#define GOAL_RADIUS 150.0f        // Goal marker radius (15 cm)
#define SUCCESS_DISTANCE 30.0f    // Position threshold (3 cm precision)
#define SUCCESS_ANGLE 0.087f      // ~5 degrees in radians (tight alignment)

// Time-based simulation parameters
#define SIMULATION_FPS 10.0f           // 10 Hz control loop
#define EPISODE_DURATION_SEC 30.0f     // 30 second episode timeout
#define MAX_STEPS ((int)(SIMULATION_FPS * EPISODE_DURATION_SEC))  // 300 steps
#define DT (1.0f / SIMULATION_FPS)     // 0.1 seconds per step

// Robot dynamics (speeds are mm/second, scaled by DT to get mm/step)
#define WHEEL_BASE 200.0f              // Distance between wheels (20 cm)
#define MAX_WHEEL_SPEED (500.0f * DT)  // Max 500 mm/s = 50 mm/step at 10 Hz

// Only use floats!
typedef struct {
    float score;
    float n; // Required as the last field
} Log;

typedef struct {
    Log log;                     // Required field
    float* observations;         // Required field. Ensure type matches in .py and .c
    float* actions;              // Required field. Ensure type matches in .py and .c
    float* rewards;              // Required field
    unsigned char* terminals;    // Required field

    // Robot state (game coordinates: origin top-left, +y down, +angle clockwise)
    float x, y;          // Position in pixels
    float theta;         // Heading angle in radians, 0=right, PI/2=down, PI=left, -PI/2=up

    // Goal state
    float goal_x, goal_y;
    float goal_theta;    // Desired heading at goal

    // Episode tracking
    int step_count;
    float prev_distance;     // For potential-based distance rewards
    float prev_angle_error;  // For potential-based angle rewards
} Docking;

// Helper: random float in [min, max]
float randf(float min, float max) {
    return min + (max - min) * ((float)rand() / RAND_MAX);
}

// Helper: normalize angle to [-PI, PI]
float normalize_angle(float angle) {
    while (angle > PI) angle -= 2.0f * PI;
    while (angle < -PI) angle += 2.0f * PI;
    return angle;
}

// Helper: calculate angle error between current heading and goal heading
float calculate_angle_error(Docking* env) {
    return normalize_angle(env->goal_theta - env->theta);
}

// Helper: update observations with ego-centric measurements
void update_observations(Docking* env) {
    // Calculate distance and bearing to goal
    float dx = env->goal_x - env->x;
    float dy = env->goal_y - env->y;
    float distance = sqrtf(dx*dx + dy*dy);

    // Normalize distance by arena diagonal
    float max_distance = sqrtf(ARENA_WIDTH * ARENA_WIDTH + ARENA_HEIGHT * ARENA_HEIGHT);
    float distance_norm = distance / max_distance;

    // Bearing: angle from robot's heading to goal (in robot's reference frame)
    float bearing = normalize_angle(atan2f(dy, dx) - env->theta);

    // Heading error: how much to rotate to match goal orientation
    float heading_error = calculate_angle_error(env);

    // Set observations: [distance_norm, sin(bearing), cos(bearing), heading_error]
    env->observations[0] = distance_norm;
    env->observations[1] = sinf(bearing);
    env->observations[2] = cosf(bearing);
    env->observations[3] = heading_error;
}

void c_reset(Docking* env) {
    // Random robot start position (away from edges, with guaranteed space)
    float margin = ROBOT_RADIUS + 10.0f;
    float safe_zone = 400.0f;  // Keep robot at least 40cm from walls initially

    env->x = randf(safe_zone, ARENA_WIDTH - safe_zone);
    env->y = randf(safe_zone, ARENA_HEIGHT - safe_zone);
    env->theta = randf(-PI, PI);

    // Place goal on one of the four walls with perpendicular approach
    // Wall selection: 0=top, 1=right, 2=bottom, 3=left
    int wall = rand() % 4;
    float dock_margin = GOAL_RADIUS * 2.0f;  // Keep docks away from corners
    float dock_size = GOAL_RADIUS;

    if (wall == 0) {
        // Top wall: robot faces UP toward wall
        env->goal_x = randf(dock_margin, ARENA_WIDTH - dock_margin);
        env->goal_y = dock_size;
        env->goal_theta = -PI / 2.0f;  // Pointing up (toward wall)
    } else if (wall == 1) {
        // Right wall: robot faces RIGHT toward wall
        env->goal_x = ARENA_WIDTH - dock_size;
        env->goal_y = randf(dock_margin, ARENA_HEIGHT - dock_margin);
        env->goal_theta = 0.0f;  // Pointing right (toward wall)
    } else if (wall == 2) {
        // Bottom wall: robot faces DOWN toward wall
        env->goal_x = randf(dock_margin, ARENA_WIDTH - dock_margin);
        env->goal_y = ARENA_HEIGHT - dock_size;
        env->goal_theta = PI / 2.0f;  // Pointing down (toward wall)
    } else {
        // Left wall: robot faces LEFT toward wall
        env->goal_x = dock_size;
        env->goal_y = randf(dock_margin, ARENA_HEIGHT - dock_margin);
        env->goal_theta = PI;  // Pointing left (toward wall)
    }

    // Calculate initial distance for reward tracking
    float dx = env->goal_x - env->x;
    float dy = env->goal_y - env->y;
    env->prev_distance = sqrtf(dx*dx + dy*dy);
    env->step_count = 0;

    // Initialize tracking for potential-based rewards
    env->prev_angle_error = calculate_angle_error(env);

    // Set initial observations
    update_observations(env);
}

void c_step(Docking* env) {
    env->rewards[0] = 0;
    env->terminals[0] = 0;
    env->step_count++;

    // Get wheel actions (continuous in [-1, 1])
    float left_wheel = env->actions[0];
    float right_wheel = env->actions[1];

    // Clamp to [-1, 1] for safety
    if (left_wheel > 1.0f) left_wheel = 1.0f;
    if (left_wheel < -1.0f) left_wheel = -1.0f;
    if (right_wheel > 1.0f) right_wheel = 1.0f;
    if (right_wheel < -1.0f) right_wheel = -1.0f;

    // Differential drive kinematics
    float vL = left_wheel * MAX_WHEEL_SPEED;
    float vR = right_wheel * MAX_WHEEL_SPEED;
    float v = (vL + vR) / 2.0f;              // Linear velocity
    float omega = (vR - vL) / WHEEL_BASE;    // Angular velocity

    // Update robot pose (game coordinates: theta=0 is right, positive is clockwise)
    env->x += v * cosf(env->theta);
    env->y += v * sinf(env->theta);
    env->theta += omega;
    env->theta = normalize_angle(env->theta);

    // Calculate distance and angle errors
    float dx = env->goal_x - env->x;
    float dy = env->goal_y - env->y;
    float distance = sqrtf(dx*dx + dy*dy);
    float angle_error = calculate_angle_error(env);

    // Potential-based reward shaping (both distance and angle)
    float reward = 0.0f;

    // Reward for decreasing distance (scaled down so success bonus dominates)
    // Typical improvement: ~50 mm/step → scale to ~0.005 per step
    reward += (env->prev_distance - distance) / 10000.0f;

    // Reward for decreasing angle error (scaled to match distance importance)
    // Max angle error is PI, max change is ~0.1 rad/step → scale similarly
    reward += (fabsf(env->prev_angle_error) - fabsf(angle_error)) * 0.5f / 1000.0f;

    // When near dock, penalize turning to force smooth, pre-aligned approach
    if (distance < SUCCESS_DISTANCE * 3.0f) {  // Within ~10cm of dock
        // Penalize angular velocity (turning action) when close
        // Forces agent to align early and approach smoothly, not make jerky corrections
        // Direct penalty on turning near dock
        reward -= fabsf(omega) / 100.0f;
    }

    // Step penalty to encourage efficiency
    reward -= 0.01f;

    env->rewards[0] = reward;

    // Update tracking for next step
    env->prev_distance = distance;
    env->prev_angle_error = angle_error;

    // Check success condition
    int success = (distance < SUCCESS_DISTANCE) && (fabsf(angle_error) < SUCCESS_ANGLE);

    // Check failure conditions
    int out_of_bounds = (env->x < 0 || env->x > ARENA_WIDTH ||
                         env->y < 0 || env->y > ARENA_HEIGHT);
    int timeout = (env->step_count >= MAX_STEPS);

    if (success) {
        env->rewards[0] += 1.0f;
        env->terminals[0] = 1;
        env->log.score += 1.0f;
        env->log.n += 1.0f;
        c_reset(env);
        return;
    }

    if (out_of_bounds || timeout) {
        env->rewards[0] -= 1.0f;
        env->terminals[0] = 1;
        env->log.n += 1.0f;
        c_reset(env);
        return;
    }

    // Update observations
    update_observations(env);
}

void c_render(Docking* env) {
    if (!IsWindowReady()) {
        InitWindow(ARENA_WIDTH, ARENA_HEIGHT, "PufferLib Docking");
        SetTargetFPS((int)SIMULATION_FPS);
    }

    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground(PUFF_BACKGROUND);

    // Draw arena walls (thicker line)
    DrawRectangleLinesEx((Rectangle){0, 0, ARENA_WIDTH, ARENA_HEIGHT}, 5, PUFF_WHITE);

    // Draw goal/dock position (circle with heading indicator)
    DrawCircle(env->goal_x, env->goal_y, GOAL_RADIUS, PUFF_RED);

    // Draw docking approach direction (thicker, longer line showing required heading)
    float goal_dir_x = env->goal_x + cosf(env->goal_theta) * GOAL_RADIUS * 1.5f;
    float goal_dir_y = env->goal_y + sinf(env->goal_theta) * GOAL_RADIUS * 1.5f;
    DrawLineEx((Vector2){env->goal_x, env->goal_y},
               (Vector2){goal_dir_x, goal_dir_y}, 5, PUFF_WHITE);

    // Draw wall segment behind dock for visual clarity
    float wall_length = GOAL_RADIUS * 3.0f;
    if (env->goal_x < GOAL_RADIUS * 2) {
        // Left wall
        DrawLineEx((Vector2){0, env->goal_y - wall_length},
                   (Vector2){0, env->goal_y + wall_length}, 8, PUFF_GREEN);
    } else if (env->goal_x > ARENA_WIDTH - GOAL_RADIUS * 2) {
        // Right wall
        DrawLineEx((Vector2){ARENA_WIDTH, env->goal_y - wall_length},
                   (Vector2){ARENA_WIDTH, env->goal_y + wall_length}, 8, PUFF_GREEN);
    } else if (env->goal_y < GOAL_RADIUS * 2) {
        // Top wall
        DrawLineEx((Vector2){env->goal_x - wall_length, 0},
                   (Vector2){env->goal_x + wall_length, 0}, 8, PUFF_GREEN);
    } else {
        // Bottom wall
        DrawLineEx((Vector2){env->goal_x - wall_length, ARENA_HEIGHT},
                   (Vector2){env->goal_x + wall_length, ARENA_HEIGHT}, 8, PUFF_GREEN);
    }

    // Draw robot (circle with direction indicator)
    DrawCircle(env->x, env->y, ROBOT_RADIUS, PUFF_CYAN);
    float robot_dir_x = env->x + cosf(env->theta) * ROBOT_RADIUS * 1.5f;
    float robot_dir_y = env->y + sinf(env->theta) * ROBOT_RADIUS * 1.5f;
    DrawLineEx((Vector2){env->x, env->y},
               (Vector2){robot_dir_x, robot_dir_y}, 3, PUFF_WHITE);

    // Draw info text
    DrawText(TextFormat("Step: %d/%d", env->step_count, MAX_STEPS), 10, 10, 20, PUFF_WHITE);
    float dx = env->goal_x - env->x;
    float dy = env->goal_y - env->y;
    float dist = sqrtf(dx*dx + dy*dy);
    DrawText(TextFormat("Distance: %.1f", dist), 10, 35, 20, PUFF_WHITE);
    float angle_err = calculate_angle_error(env);
    DrawText(TextFormat("Angle Error: %.2f", angle_err), 10, 60, 20, PUFF_WHITE);

    EndDrawing();
}

void c_close(Docking* env) {
    if (IsWindowReady()) {
        CloseWindow();
    }
}
