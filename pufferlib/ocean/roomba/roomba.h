#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "raylib.h"
#include "box2d/box2d.h"

#define MAX_DIRT_PIECES 20
#define MAX_HISTORY_POINTS 1000
#define SCALE 2.0f  // Pixels per cm - reasonable room size on screen

const float ROOMBA_RADIUS = 17.425f; // cm
const float LIGHT_BUMPER_RADIUS = 22.425f; // cm - 5cm larger than roomba radius
const float MAX_WHEEL_SPEED = 25.0f; // cm/s
const float WHEELBASE = 23.5f; // cm
const float FRICTION = 0.01f;
const float BUMP_CLEARANCE = 1.0f; // cm - distance to back away before bumper clears
const float BRUSH_WIDTH = 16.51f; // cm - 6.5" brush width
const float BRUSH_DEPTH = 7.62f; // cm - 3" brush depth
const float DIRT_DISPLAY_RADIUS = 2.5f; // cm - visual radius for dirt
const float dt = 0.05f;

typedef struct {
    float collisions;
    float episode_return;
    float episode_length;
    float dirt_collected;
    float n; // Required as the last field
} Log;

typedef struct {
    float x, y;
    int active; // 1 if dirt exists, 0 if collected
} Dirt;

typedef struct {
    float x, y;
} HistoryPoint;

typedef struct {
    Log log;                     // Required field
    float* observations;         // Required field. 4D: [left_bumper_importance, right_bumper_importance, left_light_bumper_strength, right_light_bumper_strength]
    float* actions;              // Required field. 2D: [left_wheel_speed, right_wheel_speed] in cm/s
    float* rewards;              // Required field
    unsigned char* terminals;    // Required field

    // Box2D components
    b2WorldId world_id;
    b2BodyId roomba_body_id;

    // Roomba state
    float left_wheel_speed;      // Current left wheel speed in cm/s
    float right_wheel_speed;     // Current right wheel speed in cm/s

    // Environment parameters
    float room_width;            // Room width in cm
    float room_height;           // Room height in cm
    int max_steps;
    int tick;
    float episode_return;        // Accumulated episode return

    // Sensors
    int left_bumper;             // 1 if left bumper pressed, 0 otherwise
    int right_bumper;            // 1 if right bumper pressed, 0 otherwise
    float left_bumper_importance;     // Importance of left bumper hit: 2.0=just hit, 0.0=not hit recently
    float right_bumper_importance;    // Importance of right bumper hit: 2.0=just hit, 0.0=not hit recently

    // Light bumper sensors (6 discrete sensors at specific angles)
    // Angles relative to front of robot: Left (-39.6°), Front Left (-18°), Center Left (-6°),
    // Center Right (15°), Front Right (30.5°), Right (64°)
    float light_bumper_distances[6];  // Distance to closest wall for each sensor in cm

    // Aggregated light bumper sensors for observations
    float left_light_bumper_strength;   // Strength from 0.0-1.0 based on piecewise function
    float right_light_bumper_strength;  // Strength from 0.0-1.0 based on piecewise function

    // Dirt system
    Dirt dirt_pieces[MAX_DIRT_PIECES];
    int dirt_collected_this_episode;

    // History tracking for visualization
    HistoryPoint history[MAX_HISTORY_POINTS];
    int history_count;
    int history_index;
} Roomba;

void spawn_dirt(Roomba* env) {
    for (int i = 0; i < MAX_DIRT_PIECES; i++) {
        env->dirt_pieces[i].x = (float)(rand()) / RAND_MAX * (env->room_width - 2 * ROOMBA_RADIUS) + ROOMBA_RADIUS;
        env->dirt_pieces[i].y = (float)(rand()) / RAND_MAX * (env->room_height - 2 * ROOMBA_RADIUS) + ROOMBA_RADIUS;
        env->dirt_pieces[i].active = 1;
    }
}

int check_dirt_collection(Roomba* env) {
    int collected = 0;

    // Get current roomba position and orientation from Box2D
    b2Transform transform = b2Body_GetTransform(env->roomba_body_id);
    float roomba_x = transform.p.x;
    float roomba_y = transform.p.y;
    float roomba_theta = b2Rot_GetAngle(transform.q);

    // The brush is positioned slightly behind the roomba's center point
    // This calculation finds the center of the brush area in world coordinates
    float brush_center_x = roomba_x - (BRUSH_DEPTH / 2.0f) * cosf(roomba_theta);
    float brush_center_y = roomba_y - (BRUSH_DEPTH / 2.0f) * sinf(roomba_theta);

    // Brush rectangle dimensions
    float half_width = BRUSH_WIDTH / 2.0f;
    float half_depth = BRUSH_DEPTH / 2.0f;

    for (int i = 0; i < MAX_DIRT_PIECES; i++) {
        if (!env->dirt_pieces[i].active) continue;

        // Transform dirt position to be relative to the brush's center
        float dx = env->dirt_pieces[i].x - brush_center_x;
        float dy = env->dirt_pieces[i].y - brush_center_y;

        // Rotate the relative dirt position into the brush's coordinate system
        // using the inverse of the roomba's rotation
        float cos_theta = cosf(-roomba_theta);
        float sin_theta = sinf(-roomba_theta);
        float local_x = dx * cos_theta - dy * sin_theta;
        float local_y = dx * sin_theta + dy * cos_theta;

        // Check if the transformed point is inside the brush's rectangular area
        if (local_x >= -half_depth && local_x <= half_depth &&
            local_y >= -half_width && local_y <= half_width) {
            env->dirt_pieces[i].active = 0;
            collected++;
            env->dirt_collected_this_episode++;
        }
    }

    return collected;
}


float calculate_light_bumper_strength(float distance) {
    if (distance > 10.0f) {
        return 0.0f;  // No detection beyond 10cm
    } else if (distance > 2.0f) {
        // Linear rise from 0 to 1 between 10cm and 2cm
        return (10.0f - distance) / 8.0f;  // (10-2) = 8cm range
    } else {
        // Linear fall from 1 to 0 between 2cm and 0cm
        return distance / 2.0f;
    }
}

void add_log(Roomba* env) {
    env->log.collisions += (env->left_bumper || env->right_bumper) ? 1 : 0;
    env->log.episode_length += env->tick;
    env->log.episode_return += env->episode_return;
    env->log.dirt_collected += env->dirt_collected_this_episode;
    env->log.n++;
}

void c_reset(Roomba* env) {
    // Initialize Box2D world if not already done
    if (!b2World_IsValid(env->world_id)) {
        b2WorldDef world_def = b2DefaultWorldDef();
        world_def.gravity = (b2Vec2){0, 0}; // No gravity for top-down 2D simulation
        env->world_id = b2CreateWorld(&world_def);

        // Create room walls
        b2BodyDef wall_def = b2DefaultBodyDef();
        wall_def.type = b2_staticBody;
        b2ShapeDef wall_shape_def = b2DefaultShapeDef();
        float wall_thickness = 1.0f;

        // Wall data: {x, y, half_width, half_height}
        float walls[4][4] = {
            {env->room_width / 2.0f, 0, env->room_width / 2.0f, wall_thickness / 2.0f}, // bottom
            {env->room_width / 2.0f, env->room_height, env->room_width / 2.0f, wall_thickness / 2.0f}, // top
            {0, env->room_height / 2.0f, wall_thickness / 2.0f, env->room_height / 2.0f}, // left
            {env->room_width, env->room_height / 2.0f, wall_thickness / 2.0f, env->room_height / 2.0f} // right
        };

        for (int i = 0; i < 4; i++) {
            wall_def.position = (b2Vec2){walls[i][0], walls[i][1]};
            b2BodyId wall = b2CreateBody(env->world_id, &wall_def);
            b2Polygon box = b2MakeBox(walls[i][2], walls[i][3]);
            b2CreatePolygonShape(wall, &wall_shape_def, &box);
        }

        // Create roomba body
        b2BodyDef roomba_def = b2DefaultBodyDef();
        roomba_def.type = b2_dynamicBody;
        roomba_def.position = (b2Vec2){env->room_width / 2.0f, env->room_height / 2.0f};
        roomba_def.linearDamping = 2.0f;
        roomba_def.angularDamping = 2.0f;
        env->roomba_body_id = b2CreateBody(env->world_id, &roomba_def);

        b2Circle roomba_circle;
        roomba_circle.center = (b2Vec2){0, 0};
        roomba_circle.radius = ROOMBA_RADIUS - BUMP_CLEARANCE;

        b2ShapeDef roomba_shape_def = b2DefaultShapeDef();
        roomba_shape_def.density = 1.0f;
        b2CreateCircleShape(env->roomba_body_id, &roomba_shape_def, &roomba_circle);
    }

    // Reset roomba to center of room with random orientation
    b2Vec2 center_pos = {env->room_width / 2.0f, env->room_height / 2.0f};
    float random_angle = (float)(rand()) / RAND_MAX * 2.0f * PI;

    b2Body_SetTransform(env->roomba_body_id, center_pos, b2MakeRot(random_angle));
    b2Body_SetLinearVelocity(env->roomba_body_id, (b2Vec2){0, 0});
    b2Body_SetAngularVelocity(env->roomba_body_id, 0);

    env->left_wheel_speed = 0.0f;
    env->right_wheel_speed = 0.0f;
    env->tick = 0;
    env->episode_return = 0.0f;
    env->left_bumper = 0;
    env->right_bumper = 0;
    env->left_bumper_importance = 0.0f;
    env->right_bumper_importance = 0.0f;
    for (int i = 0; i < 6; i++) {
        env->light_bumper_distances[i] = 100.0f;  // Initialize to large distance
    }
    env->left_light_bumper_strength = 0.0f;
    env->right_light_bumper_strength = 0.0f;
    env->dirt_collected_this_episode = 0;

    // Reset history tracking
    env->history_count = 0;
    env->history_index = 0;

    // Spawn new dirt pieces
    spawn_dirt(env);

    // Set initial observations: [left_bumper_importance, right_bumper_importance, left_light_bumper_strength, right_light_bumper_strength]
    env->observations[0] = env->left_bumper_importance;
    env->observations[1] = env->right_bumper_importance;
    env->observations[2] = env->left_light_bumper_strength;
    env->observations[3] = env->right_light_bumper_strength;
}

void c_step(Roomba* env) {
    env->tick += 1;
    env->left_bumper = 0;
    env->right_bumper = 0;
    for (int i = 0; i < 6; i++) {
        env->light_bumper_distances[i] = 100.0f;  // Initialize to large distance
    }
    env->left_light_bumper_strength = 0.0f;
    env->right_light_bumper_strength = 0.0f;
    env->rewards[0] = 0.0f;
    env->terminals[0] = 0;

    // Decrement bumper importance over time (minimum 0.0)
    env->left_bumper_importance = fmaxf(0.0f, env->left_bumper_importance - dt);
    env->right_bumper_importance = fmaxf(0.0f, env->right_bumper_importance - dt);

    // Scale normalized actions to wheel speeds
    float target_left = env->actions[0] * MAX_WHEEL_SPEED;
    float target_right = env->actions[1] * MAX_WHEEL_SPEED;

    // Apply wheel speed commands with friction
    env->left_wheel_speed = env->left_wheel_speed * FRICTION + target_left * (1.0f - FRICTION);
    env->right_wheel_speed = env->right_wheel_speed * FRICTION + target_right * (1.0f - FRICTION);

    // Convert wheel speeds to linear and angular velocity
    float linear_velocity = (env->left_wheel_speed + env->right_wheel_speed) / 2.0f; // cm/s
    float angular_velocity = (env->right_wheel_speed - env->left_wheel_speed) / WHEELBASE; // rad/s

    // Get current transform and set Box2D velocities
    b2Transform transform = b2Body_GetTransform(env->roomba_body_id);
    float current_angle = b2Rot_GetAngle(transform.q);

    b2Vec2 velocity = {
        linear_velocity * cosf(current_angle),
        linear_velocity * sinf(current_angle)
    };
    b2Body_SetLinearVelocity(env->roomba_body_id, velocity);
    b2Body_SetAngularVelocity(env->roomba_body_id, angular_velocity);

    // Step Box2D simulation
    b2World_Step(env->world_id, dt, 4);

    // Record position in history every 3 ticks for smoother trail
    if (env->tick % 3 == 0) {
        b2Vec2 current_pos = b2Body_GetPosition(env->roomba_body_id);
        env->history[env->history_index].x = current_pos.x;
        env->history[env->history_index].y = current_pos.y;
        env->history_index = (env->history_index + 1) % MAX_HISTORY_POINTS;
        if (env->history_count < MAX_HISTORY_POINTS) {
            env->history_count++;
        }
    }

    // Bump detection: check if bumper points are outside room
    b2Vec2 pos = b2Body_GetPosition(env->roomba_body_id);

    // Check left bumper arc (theta - PI/36 to theta - PI/2)
    for (float angle_offset = current_angle - PI/36; angle_offset > current_angle - PI/2; angle_offset -= PI/36) {
        float sample_x = pos.x + ROOMBA_RADIUS * cosf(angle_offset);
        float sample_y = pos.y + ROOMBA_RADIUS * sinf(angle_offset);
        if (sample_x <= 0 || sample_x >= env->room_width ||
            sample_y <= 0 || sample_y >= env->room_height) {
            env->left_bumper = 1;
            env->left_bumper_importance = 2.0f;
            break;
        }
    }

    // Check right bumper arc (theta + PI/36 to theta + PI/2)
    for (float angle_offset = current_angle + PI/36; angle_offset < current_angle + PI/2; angle_offset += PI/36) {
        float sample_x = pos.x + ROOMBA_RADIUS * cosf(angle_offset);
        float sample_y = pos.y + ROOMBA_RADIUS * sinf(angle_offset);
        if (sample_x <= 0 || sample_x >= env->room_width ||
            sample_y <= 0 || sample_y >= env->room_height) {
            env->right_bumper = 1;
            env->right_bumper_importance = 2.0f;
            break;
        }
    }

    // Check for wall collision using Box2D contacts
    int wall_collision = 0;
    int contact_count = b2Body_GetContactCapacity(env->roomba_body_id);
    if (contact_count > 0) {
        b2ContactData* contacts = malloc(contact_count * sizeof(b2ContactData));
        int actual_count = b2Body_GetContactData(env->roomba_body_id, contacts, contact_count);

        for (int i = 0; i < actual_count; i++) {
            if (contacts[i].manifold.pointCount > 0) {
                wall_collision = 1;
                break;
            }
        }
        free(contacts);
    }

    // Light bumper detection: 6 discrete sensors at specific angles with distance calculation
    // Sensor angles in degrees relative to front of robot
    float sensor_angles[6] = {-39.6f, -18.0f, -6.0f, 15.0f, 30.5f, 64.0f};

    for (int i = 0; i < 6; i++) {
        float sensor_angle = current_angle + sensor_angles[i] * PI / 180.0f;
        float cos_angle = cosf(sensor_angle);
        float sin_angle = sinf(sensor_angle);

        // Ray cast from robot position outward in sensor direction
        float step_size = 0.5f;  // cm per step
        float max_range = 50.0f; // cm - enough to detect walls
        float distance = max_range;

        for (float d = step_size; d <= max_range; d += step_size) {
            float sample_x = pos.x + d * cos_angle;
            float sample_y = pos.y + d * sin_angle;

            // Check if sample point is outside room boundaries
            if (sample_x <= 0 || sample_x >= env->room_width ||
                sample_y <= 0 || sample_y >= env->room_height) {
                distance = d;
                break;
            }
        }

        // Convert from center-to-wall distance to edge-to-wall distance
        if (distance >= max_range) {
            // No wall found within range - robot edge is far from any wall
            env->light_bumper_distances[i] = 100.0f;  // Large distance indicating no nearby walls
        } else {
            // Wall found - calculate edge-to-wall distance
            float edge_distance = distance - ROOMBA_RADIUS;
            env->light_bumper_distances[i] = fmaxf(0.0f, edge_distance);
        }
    }

    // Aggregate sensors into left/right groups using the piecewise strength function
    float left_strengths[3] = {
        calculate_light_bumper_strength(env->light_bumper_distances[0]),
        calculate_light_bumper_strength(env->light_bumper_distances[1]),
        calculate_light_bumper_strength(env->light_bumper_distances[2])
    };
    float right_strengths[3] = {
        calculate_light_bumper_strength(env->light_bumper_distances[3]),
        calculate_light_bumper_strength(env->light_bumper_distances[4]),
        calculate_light_bumper_strength(env->light_bumper_distances[5])
    };

    // Take maximum strength from each side
    env->left_light_bumper_strength = fmaxf(fmaxf(left_strengths[0], left_strengths[1]), left_strengths[2]);
    env->right_light_bumper_strength = fmaxf(fmaxf(right_strengths[0], right_strengths[1]), right_strengths[2]);

    // Reward forward movement only
    b2Vec2 actual_velocity = b2Body_GetLinearVelocity(env->roomba_body_id);
    float forward_velocity = actual_velocity.x * cosf(current_angle) + actual_velocity.y * sinf(current_angle);

    env->rewards[0] = 0.0f;
    if (forward_velocity >= -1.0f) {
      env->rewards[0] += fabsf(forward_velocity) / MAX_WHEEL_SPEED * 0.05f;
    }
    if (wall_collision) {
      env->rewards[0] -= 0.5f;
    }

    // Check for dirt collection and add rewards
    int dirt_collected = check_dirt_collection(env);
    env->rewards[0] += dirt_collected * 0.5f;

    // Accumulate reward into episode return
    env->episode_return += env->rewards[0];

    // Update observations: [left_bumper_importance, right_bumper_importance, left_light_bumper_strength, right_light_bumper_strength]
    env->observations[0] = env->left_bumper_importance;
    env->observations[1] = env->right_bumper_importance;
    env->observations[2] = env->left_light_bumper_strength;
    env->observations[3] = env->right_light_bumper_strength;

    // Check for episode termination
    if (env->tick >= env->max_steps) {
        env->terminals[0] = 1;
        add_log(env);
        c_reset(env);
    }
}

// Helper function to map simulation coordinates (cm, y-up) to screen coordinates (pixels, y-down)
Vector2 map_to_screen(float sim_x, float sim_y, float room_height, float offset_x, float offset_y) {
    float screen_x = offset_x + sim_x * SCALE;
    // Invert the y-axis: (room_height - sim_y)
    float screen_y = offset_y + (room_height - sim_y) * SCALE;
    return (Vector2){screen_x, screen_y};
}

void c_render(Roomba* env) {
    float room_pixel_width = env->room_width * SCALE;
    float room_pixel_height = env->room_height * SCALE;

    int WINDOW_WIDTH = fmaxf(800, room_pixel_width);
    int WINDOW_HEIGHT = fmaxf(600, room_pixel_height);

    if (!IsWindowReady()) {
        InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PufferLib Roomba");
        SetTargetFPS(1 / dt);
    }

    if (WindowShouldClose() || IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});

    // Calculate rendering offsets to center the room in the window
    float offset_x = (WINDOW_WIDTH - room_pixel_width) / 2.0f;
    float offset_y = (WINDOW_HEIGHT - room_pixel_height) / 2.0f;

    // Draw room boundaries (uses screen coordinates, so no mapping needed)
    DrawRectangleLinesEx((Rectangle){offset_x, offset_y, room_pixel_width, room_pixel_height}, 2.0f, (Color){241, 241, 241, 255});

    // Draw roomba movement history trail
    if (env->history_count > 1) {
        for (int i = 1; i < env->history_count; i++) {
            int prev_idx = (env->history_index - env->history_count + i - 1 + MAX_HISTORY_POINTS) % MAX_HISTORY_POINTS;
            int curr_idx = (env->history_index - env->history_count + i + MAX_HISTORY_POINTS) % MAX_HISTORY_POINTS;

            // Map simulation history points to screen coordinates
            Vector2 prev_pos = map_to_screen(env->history[prev_idx].x, env->history[prev_idx].y, env->room_height, offset_x, offset_y);
            Vector2 curr_pos = map_to_screen(env->history[curr_idx].x, env->history[curr_idx].y, env->room_height, offset_x, offset_y);

            // Fade older trail segments
            int i_bonus = env->history_count < MAX_HISTORY_POINTS ? (MAX_HISTORY_POINTS - env->history_count) : 0;
            float alpha = (float)(i+i_bonus) / MAX_HISTORY_POINTS;
            Color trail_color = Fade((Color){0, 150, 150, 255}, alpha * 0.7f);

            DrawLineEx(prev_pos, curr_pos, 2.0f, trail_color);
        }
    }

    // Get roomba transform from Box2D
    b2Transform transform = b2Body_GetTransform(env->roomba_body_id);
    float roomba_angle = b2Rot_GetAngle(transform.q);

    // Map roomba's center from simulation to screen coordinates
    Vector2 roomba_pos = map_to_screen(transform.p.x, transform.p.y, env->room_height, offset_x, offset_y);
    float roomba_radius = ROOMBA_RADIUS * SCALE;

    Color roomba_color = (env->left_bumper || env->right_bumper) ? (Color){187, 0, 0, 255} : (Color){0, 187, 187, 255};
    DrawCircle(roomba_pos.x, roomba_pos.y, roomba_radius, roomba_color);

    // Draw dirt pieces
    for (int i = 0; i < MAX_DIRT_PIECES; i++) {
        if (env->dirt_pieces[i].active) {
            // Map each dirt piece from simulation to screen coordinates
            Vector2 dirt_pos = map_to_screen(env->dirt_pieces[i].x, env->dirt_pieces[i].y, env->room_height, offset_x, offset_y);
            DrawCircle(dirt_pos.x, dirt_pos.y, DIRT_DISPLAY_RADIUS * SCALE, (Color){139, 69, 19, 255}); // Brown dirt
        }
    }

    // Draw brush collection area
    {
        // Define brush rectangle corners in the Roomba's local coordinate system
        float half_width = BRUSH_WIDTH / 2.0f;
        float half_depth = BRUSH_DEPTH / 2.0f;
        // The center of the brush is offset behind the Roomba's center
        float brush_center_offset_x = -BRUSH_DEPTH / 2.0f;

        Vector2 local_corners[4] = {
            {brush_center_offset_x - half_depth, -half_width}, // Back-left
            {brush_center_offset_x + half_depth, -half_width}, // Front-left
            {brush_center_offset_x + half_depth,  half_width}, // Front-right
            {brush_center_offset_x - half_depth,  half_width}  // Back-right
        };

        Vector2 screen_corners[4];
        for (int i = 0; i < 4; i++) {
            // Rotate the local corner around the Roomba's origin
            float rotated_x = local_corners[i].x * cosf(roomba_angle) - local_corners[i].y * sinf(roomba_angle);
            float rotated_y = local_corners[i].x * sinf(roomba_angle) + local_corners[i].y * cosf(roomba_angle);

            // Translate to the Roomba's world position to get the corner in simulation coordinates
            float sim_corner_x = transform.p.x + rotated_x;
            float sim_corner_y = transform.p.y + rotated_y;

            // Map the final simulation coordinate to a screen coordinate
            screen_corners[i] = map_to_screen(sim_corner_x, sim_corner_y, env->room_height, offset_x, offset_y);
        }

        // Draw the brush outline by connecting the screen corners
        for (int i = 0; i < 4; i++) {
            DrawLineEx(screen_corners[i], screen_corners[(i + 1) % 4], 2.0f, (Color){255, 255, 0, 128}); // Semi-transparent yellow
        }
    }


    // Draw orientation indicator
    float indicator_length = roomba_radius * 0.8f;
    // Calculate the endpoint of the indicator line
    // We subtract the sine component because the y-axis is inverted on the screen
    float end_x = roomba_pos.x + indicator_length * cosf(roomba_angle);
    float end_y = roomba_pos.y - indicator_length * sinf(roomba_angle);
    DrawLineEx(
        roomba_pos,
        (Vector2){end_x, end_y},
        6.0f,
        (Color){255, 255, 255, 255}
    );

    // Draw sensor information text
    char sensor_text[300];
    snprintf(sensor_text, sizeof(sensor_text),
        "%.2f/%.2f | %.0f,%.0f going %.0f/%.0f facing %.0f | Bump: %.2f,%.2f/%d,%d | Dirt: %d/%d",
        env->rewards[0], env->episode_return,
        transform.p.x, transform.p.y, fmodf(roomba_angle * 180.0f / PI, 360.0f),
        env->left_wheel_speed, env->right_wheel_speed,
        env->left_light_bumper_strength, env->right_light_bumper_strength,
        env->left_bumper, env->right_bumper,
        env->dirt_collected_this_episode, MAX_DIRT_PIECES);
    DrawText(sensor_text, 10, 10, 20, (Color){241, 241, 241, 255});

    EndDrawing();
}

void c_close(Roomba* env) {
    if (b2World_IsValid(env->world_id)) {
        b2DestroyWorld(env->world_id);
        env->world_id = b2_nullWorldId;
    }

    if (IsWindowReady()) {
        CloseWindow();
    }
}
