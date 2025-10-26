#include <stdlib.h>
#include <math.h>
#include "raylib.h"

const Color PUFF_RED = (Color){187, 0, 0, 255};
const Color PUFF_CYAN = (Color){0, 187, 187, 255};
const Color PUFF_GREEN = (Color){0, 187, 0, 255};
const Color PUFF_WHITE = (Color){241, 241, 241, 241};
const Color PUFF_BACKGROUND = (Color){6, 24, 24, 255};

// Environment constants (all distances in millimeters)
#define ARENA_WIDTH 1000.0f
#define ARENA_HEIGHT 1000.0f
#define GOAL_RADIUS 150.0f        // Goal marker radius (15 cm)
#define FF_RANGE 380.0f           // 15 in force field
#define BUOY_RANGE 1000.0f        // 1 meter range buoys
#define BUOY_INWARD_ANGLE 0.17f   // ~10 degrees in radians
#define BUOY_OUTWARD_ANGLE 0.7f   // ~40 degrees in radians
#define ANGLED_SENSOR_MIN 0.17f   // ~10 degrees - inner edge of directional sensor FOV
#define ANGLED_SENSOR_MAX 1.05f   // ~60 degrees - outer edge of directional sensor FOV
#define WALL_DETECT_DISTANCE 50.0f // Light bumper detection distance (5 cm)
#define SUCCESS_DISTANCE 30.0f    // Position threshold (3 cm precision)
#define SUCCESS_ANGLE 0.087f      // ~5 degrees in radians (tight alignment)

// Time-based simulation parameters
#define SIMULATION_FPS 30.0f           // Control loop
#define EPISODE_DURATION_SEC 30.0f     // 30 second episode timeout
#define MAX_STEPS ((int)(SIMULATION_FPS * EPISODE_DURATION_SEC))
#define DT (1.0f / SIMULATION_FPS)     // Seconds per step

// Robot dynamics (speeds are mm/second, scaled by DT to get mm/step)
#define ROBOT_RADIUS 174.25f
#define WHEEL_BASE 235.0f
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
    float goal_wall_x, goal_wall_y;
    float goal_robot_x, goal_robot_y;
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

// Helper: get buoy emission cone angles (in radians, world frame)
void get_buoy_angles(Docking* env, float* green_left, float* green_right,
                     float* red_left, float* red_right) {
    // Buoys emit from the wall, opposite the goal approach direction
    float emission_dir = env->goal_theta + PI;

    // Green buoy (left when facing dock): emits left-outward
    *green_left = emission_dir + BUOY_OUTWARD_ANGLE;
    *green_right = emission_dir - BUOY_INWARD_ANGLE;

    // Red buoy (right when facing dock): emits right-outward
    *red_left = emission_dir - BUOY_OUTWARD_ANGLE;
    *red_right = emission_dir + BUOY_INWARD_ANGLE;
}

// Helper: check if angle is within a circular sector [sector_min, sector_max]
int is_in_sector(float angle, float sector_min, float sector_max) {
    // Normalize all angles to [-PI, PI]
    angle = normalize_angle(angle);
    sector_min = normalize_angle(sector_min);
    sector_max = normalize_angle(sector_max);

    // Calculate angular span and offset from min
    float span = normalize_angle(sector_max - sector_min);
    float delta = normalize_angle(angle - sector_min);

    // Check if delta is within the span (handles wraparound)
    return (delta >= 0 && delta <= span);
}

// Helper: update observations with IR-like sensor measurements
void update_observations(Docking* env) {
    // All sensors positioned at front-center of robot
    float sensor_x = env->x + cosf(env->theta) * ROBOT_RADIUS;
    float sensor_y = env->y + sinf(env->theta) * ROBOT_RADIUS;

    // Calculate position from sensor to dock
    float dx_from_dock = sensor_x - env->goal_wall_x;
    float dy_from_dock = sensor_y - env->goal_wall_y;
    float distance_from_dock = sqrtf(dx_from_dock * dx_from_dock + dy_from_dock * dy_from_dock);

    // Angle from dock to sensor (world frame) - for checking buoy emission cones
    float angle_to_sensor = atan2f(dy_from_dock, dx_from_dock);

    // 1. Force field detection (omni sensor, 360° coverage)
    float ff_detected = (distance_from_dock <= FF_RANGE) ? 1.0f : 0.0f;

    // 2. Buoy detections (omni sensor, but only within emission cones)
    float green_left, green_right, red_left, red_right;
    get_buoy_angles(env, &green_left, &green_right, &red_left, &red_right);

    int in_green_cone = is_in_sector(angle_to_sensor, green_right, green_left);
    int in_red_cone = is_in_sector(angle_to_sensor, red_left, red_right);

    float green_detected = (distance_from_dock <= BUOY_RANGE && in_green_cone) ? 1.0f : 0.0f;
    float red_detected = (distance_from_dock <= BUOY_RANGE && in_red_cone) ? 1.0f : 0.0f;

    // 3. Directional sensors (front-left and front-right, narrow FOV)
    // Calculate bearing from sensor to dock in robot's reference frame
    float dx_to_dock = env->goal_wall_x - sensor_x;
    float dy_to_dock = env->goal_wall_y - sensor_y;
    float bearing_to_dock = normalize_angle(atan2f(dy_to_dock, dx_to_dock) - env->theta);

    // Front-left sensor: FOV from min to max (left side)
    // Front-right sensor: FOV from -min to -max (right side)
    int left_sees_signal = (bearing_to_dock > ANGLED_SENSOR_MIN && bearing_to_dock < ANGLED_SENSOR_MAX) &&
                           (green_detected || red_detected);
    int right_sees_signal = (bearing_to_dock < -ANGLED_SENSOR_MIN && bearing_to_dock > -ANGLED_SENSOR_MAX) &&
                            (green_detected || red_detected);

    float angled_active = (left_sees_signal || right_sees_signal) ? 1.0f : 0.0f;

    // 4. Wall proximity sensor (light bumper simulation)
    // Check if position 5cm ahead of robot's front would be out of bounds
    float check_distance = ROBOT_RADIUS + WALL_DETECT_DISTANCE;
    float check_x = env->x + cosf(env->theta) * check_distance;
    float check_y = env->y + sinf(env->theta) * check_distance;

    int wall_nearby = (check_x < 0 || check_x > ARENA_WIDTH ||
                       check_y < 0 || check_y > ARENA_HEIGHT) ? 1.0f : 0.0f;

    // Set observations: [force_field, green_buoy, red_buoy, angled_active, wall_nearby]
    env->observations[0] = ff_detected;
    env->observations[1] = green_detected;
    env->observations[2] = red_detected;
    env->observations[3] = angled_active;
    env->observations[4] = wall_nearby;
}

void c_reset(Docking* env) {
    // Place goal on one of the four walls with perpendicular approach
    // Wall selection: 0=top, 1=right, 2=bottom, 3=left
    int wall = rand() % 4;
    float dock_margin = GOAL_RADIUS * 2.0f;  // Keep docks away from corners
    float dock_size = GOAL_RADIUS;

    if (wall == 0) {
        // Top wall: robot faces UP toward wall
        env->goal_robot_x = env->goal_wall_x = randf(dock_margin, ARENA_WIDTH - dock_margin);
        env->goal_robot_y = env->goal_wall_y = 0;
        env->goal_robot_y += ROBOT_RADIUS;
        env->goal_theta = -PI / 2.0f;  // Pointing up (toward wall)
    } else if (wall == 1) {
        // Right wall: robot faces RIGHT toward wall
        env->goal_robot_x = env->goal_wall_x = ARENA_WIDTH;
        env->goal_robot_y = env->goal_wall_y = randf(dock_margin, ARENA_HEIGHT - dock_margin);
        env->goal_robot_x -= ROBOT_RADIUS;
        env->goal_theta = 0.0f;  // Pointing right (toward wall)
    } else if (wall == 2) {
        // Bottom wall: robot faces DOWN toward wall
        env->goal_robot_x = env->goal_wall_x = randf(dock_margin, ARENA_WIDTH - dock_margin);
        env->goal_robot_y = env->goal_wall_y = ARENA_HEIGHT;
        env->goal_robot_y -= ROBOT_RADIUS;
        env->goal_theta = PI / 2.0f;  // Pointing down (toward wall)
    } else {
        // Left wall: robot faces LEFT toward wall
        env->goal_robot_x = env->goal_wall_x = 0;
        env->goal_robot_y = env->goal_wall_y = randf(dock_margin, ARENA_HEIGHT - dock_margin);
        env->goal_robot_x += ROBOT_RADIUS;
        env->goal_theta = PI;  // Pointing left (toward wall)
    }

    float robot_safe_zone = 400.0f;  // Keep robot at least 40cm from walls initially
    env->x = randf(robot_safe_zone, ARENA_WIDTH - robot_safe_zone);
    env->y = randf(robot_safe_zone, ARENA_HEIGHT - robot_safe_zone);
    env->theta = normalize_angle(env->goal_theta + randf(-PI/2.0f, PI/2.0f));

    // Calculate initial distance for reward tracking
    float dx = env->goal_robot_x - env->x;
    float dy = env->goal_robot_y - env->y;
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
    float dx = env->goal_robot_x - env->x;
    float dy = env->goal_robot_y - env->y;
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
        float bonus = 1.0f;
        bonus -= fabsf(angle_error) / SUCCESS_ANGLE * 0.5f;
        // float speeding_min = 100.0f * DT;
        // float speeding_max = MAX_WHEEL_SPEED;
        // if (v > speeding_min) {
        //     bonus -= (v - speeding_min) / (speeding_max - speeding_min) * 0.5f;
        // }
        env->rewards[0] += bonus;
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

    // Draw dock
    float wall_length = GOAL_RADIUS;
    Vector2 dock_point_1 = {env->goal_wall_x + cosf(env->goal_theta - PI*0.5) * wall_length, env->goal_wall_y + sinf(env->goal_theta - PI*0.5) * wall_length};
    Vector2 dock_point_2 = {env->goal_wall_x + cosf(env->goal_theta + PI*0.5) * wall_length, env->goal_wall_y + sinf(env->goal_theta + PI*0.5) * wall_length};
    DrawLineEx(dock_point_1, dock_point_2, 10, (Color){128,128,128,255});
    DrawCircleLines(env->goal_wall_x, env->goal_wall_y, FF_RANGE, (Color){128,128,255,255});

    // Draw buoy emission cones
    float green_left, green_right, red_left, red_right;
    get_buoy_angles(env, &green_left, &green_right, &red_left, &red_right);
    DrawCircleSectorLines((Vector2){env->goal_wall_x, env->goal_wall_y}, BUOY_RANGE,
                         green_left * RAD2DEG, green_right * RAD2DEG, 20, (Color){0,255,0,255});
    DrawCircleSectorLines((Vector2){env->goal_wall_x, env->goal_wall_y}, BUOY_RANGE,
                         red_left * RAD2DEG, red_right * RAD2DEG, 20, (Color){255,0,0,255});

    // Draw robot (circle with direction indicator)
    DrawCircle(env->x, env->y, ROBOT_RADIUS, PUFF_CYAN);
    float robot_dir_x = env->x + cosf(env->theta) * ROBOT_RADIUS * 1.5f;
    float robot_dir_y = env->y + sinf(env->theta) * ROBOT_RADIUS * 1.5f;
    DrawLineEx((Vector2){env->x, env->y},
               (Vector2){robot_dir_x, robot_dir_y}, 3, PUFF_WHITE);

    // Draw info text
    DrawText(TextFormat("Step: %d/%d", env->step_count, MAX_STEPS), 10, 10, 20, PUFF_WHITE);
    DrawText(TextFormat("Distance: %.1f", env->prev_distance), 10, 35, 20, PUFF_WHITE);
    float angle_err = calculate_angle_error(env);
    DrawText(TextFormat("Angle Error: %.2f", angle_err), 10, 60, 20, PUFF_WHITE);

    // Draw IR sensor states
    DrawText(TextFormat("FF:%.0f Green:%.0f Red:%.0f Angled:%.0f Wall:%.0f",
                       env->observations[0], env->observations[1],
                       env->observations[2], env->observations[3],
                       env->observations[4]),
             10, 85, 20, PUFF_WHITE);

    EndDrawing();
}

void c_close(Docking* env) {
    if (IsWindowReady()) {
        CloseWindow();
    }
}
