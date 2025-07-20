#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#include "raylib.h"
#include "box2d/box2d.h"

#define MAX_DIRT_PIECES 20

const float ROOMBA_RADIUS = 17.425f; // cm
const float MAX_WHEEL_SPEED = 50.0f; // cm/s
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
    Log log;                     // Required field
    float* observations;         // Required field. 2D: [left_bumper_importance, right_bumper_importance]
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

    // Dirt system
    Dirt dirt_pieces[MAX_DIRT_PIECES];
    int dirt_collected_this_episode;
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

    // Calculate brush rectangle corners relative to roomba center
    float brush_center_x = roomba_x - (BRUSH_DEPTH / 2.0f) * cosf(roomba_theta);
    float brush_center_y = roomba_y - (BRUSH_DEPTH / 2.0f) * sinf(roomba_theta);

    // Brush rectangle corners (local coordinates)
    float half_width = BRUSH_WIDTH / 2.0f;
    float half_depth = BRUSH_DEPTH / 2.0f;

    for (int i = 0; i < MAX_DIRT_PIECES; i++) {
        if (!env->dirt_pieces[i].active) continue;

        // Transform dirt position to brush-relative coordinates
        float dx = env->dirt_pieces[i].x - brush_center_x;
        float dy = env->dirt_pieces[i].y - brush_center_y;

        // Rotate to brush coordinate system
        float cos_theta = cosf(-roomba_theta);
        float sin_theta = sinf(-roomba_theta);
        float local_x = dx * cos_theta - dy * sin_theta;
        float local_y = dx * sin_theta + dy * cos_theta;

        // Check if point is inside brush rectangle
        if (local_x >= -half_depth && local_x <= half_depth &&
            local_y >= -half_width && local_y <= half_width) {
            env->dirt_pieces[i].active = 0;
            collected++;
            env->dirt_collected_this_episode++;
        }
    }

    return collected;
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
            {env->room_width / 2.0f, -wall_thickness / 2.0f, env->room_width / 2.0f, wall_thickness / 2.0f}, // top
            {env->room_width / 2.0f, env->room_height + wall_thickness / 2.0f, env->room_width / 2.0f, wall_thickness / 2.0f}, // bottom
            {-wall_thickness / 2.0f, env->room_height / 2.0f, wall_thickness / 2.0f, env->room_height / 2.0f}, // left
            {env->room_width + wall_thickness / 2.0f, env->room_height / 2.0f, wall_thickness / 2.0f, env->room_height / 2.0f} // right
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
    env->dirt_collected_this_episode = 0;

    // Spawn new dirt pieces
    spawn_dirt(env);

    // Set initial observations: [left_bumper_importance, right_bumper_importance]
    env->observations[0] = env->left_bumper_importance;
    env->observations[1] = env->right_bumper_importance;
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

    // Scale normalized actions [-1, 1] to wheel speeds [-50, 50] cm/s
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

    // Bump detection: check if bumper points are outside room (simplified from original)
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
    b2ContactData* contacts = malloc(contact_count * sizeof(b2ContactData));
    int actual_count = b2Body_GetContactData(env->roomba_body_id, contacts, contact_count);

    for (int i = 0; i < actual_count; i++) {
        if (contacts[i].manifold.pointCount > 0) {
            wall_collision = 1;
            break;
        }
    }
    free(contacts);

    // Reward forward movement only
    b2Vec2 actual_velocity = b2Body_GetLinearVelocity(env->roomba_body_id);
    float forward_velocity = actual_velocity.x * cosf(current_angle) + actual_velocity.y * sinf(current_angle);

    env->rewards[0] = 0.0f;
    if (forward_velocity >= -1.0f) {
      env->rewards[0] += fabsf(forward_velocity) / 50.0f * 0.5f;
    }
    if (wall_collision) {
      env->rewards[0] -= 0.5f;
    }

    // Check for dirt collection and add rewards
    int dirt_collected = check_dirt_collection(env);
    env->rewards[0] += dirt_collected * 0.5f;

    // Accumulate reward into episode return
    env->episode_return += env->rewards[0];

    // Update observations: [left_bumper_importance, right_bumper_importance]
    env->observations[0] = env->left_bumper_importance;
    env->observations[1] = env->right_bumper_importance;

    // Check for episode termination
    if (env->tick >= env->max_steps) {
        env->terminals[0] = 1;
        add_log(env);
        c_reset(env);
    }
}

void c_render(Roomba* env) {
    const float SCALE = 2.0f;  // Pixels per cm - reasonable room size on screen
    float room_pixel_width = env->room_width * SCALE;
    float room_pixel_height = env->room_height * SCALE;

    int WINDOW_WIDTH = fmaxf(800, room_pixel_width);
    int WINDOW_HEIGHT = fmaxf(600, room_pixel_height);

    if (!IsWindowReady()) {
        InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "PufferLib Roomba");
        SetTargetFPS(1 / dt);
    }

    if (IsKeyDown(KEY_ESCAPE)) {
        exit(0);
    }

    BeginDrawing();
    ClearBackground((Color){6, 24, 24, 255});

    // Draw room boundaries
    float offset_x = (WINDOW_WIDTH - room_pixel_width) / 2.0f;
    float offset_y = (WINDOW_HEIGHT - room_pixel_height) / 2.0f;

    // Draw room boundaries with thick lines for visibility
    DrawRectangleLines(offset_x, offset_y, room_pixel_width, room_pixel_height, (Color){241, 241, 241, 255});
    DrawRectangleLines(offset_x-1, offset_y-1, room_pixel_width+2, room_pixel_height+2, (Color){241, 241, 241, 255});
    DrawRectangleLines(offset_x+1, offset_y+1, room_pixel_width-2, room_pixel_height-2, (Color){241, 241, 241, 255});

    // Get roomba position and orientation from Box2D
    b2Transform transform = b2Body_GetTransform(env->roomba_body_id);
    float roomba_x = offset_x + transform.p.x * SCALE;
    float roomba_y = offset_y + transform.p.y * SCALE;
    float roomba_angle = b2Rot_GetAngle(transform.q);
    float roomba_radius = ROOMBA_RADIUS * SCALE;

    Color roomba_color = (env->left_bumper || env->right_bumper) ? (Color){187, 0, 0, 255} : (Color){0, 187, 187, 255};
    DrawCircle(roomba_x, roomba_y, roomba_radius, roomba_color);

    // Draw dirt pieces
    for (int i = 0; i < MAX_DIRT_PIECES; i++) {
        if (env->dirt_pieces[i].active) {
            float dirt_x = offset_x + env->dirt_pieces[i].x * SCALE;
            float dirt_y = offset_y + env->dirt_pieces[i].y * SCALE;
            DrawCircle(dirt_x, dirt_y, DIRT_DISPLAY_RADIUS * SCALE, (Color){139, 69, 19, 255}); // Brown dirt
        }
    }

    // Draw brush collection area
    float brush_center_x = offset_x + (transform.p.x - (BRUSH_DEPTH / 2.0f) * cosf(roomba_angle)) * SCALE;
    float brush_center_y = offset_y + (transform.p.y - (BRUSH_DEPTH / 2.0f) * sinf(roomba_angle)) * SCALE;

    float brush_width = BRUSH_WIDTH * SCALE;
    float brush_depth = BRUSH_DEPTH * SCALE;

    // Calculate brush rectangle corners
    float cos_theta = cosf(roomba_angle);
    float sin_theta = sinf(roomba_angle);
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
    float end_x = roomba_x + indicator_length * cosf(roomba_angle);
    float end_y = roomba_y + indicator_length * sinf(roomba_angle);
    DrawLineEx(
        (Vector2){roomba_x, roomba_y},
        (Vector2){end_x, end_y},
        6.0f,
        (Color){255, 255, 255, 255}
    );

    // Draw sensor information
    char sensor_text[300];
    snprintf(sensor_text, sizeof(sensor_text),
        "%.2f/%.2f | (%.0f,%.0f) facing %.1f° | %.0f/s, %.0f/s | bump %d%d | Dirt: %d/%d",
        env->rewards[0], env->episode_return,
        transform.p.x, transform.p.y, roomba_angle * 180.0f / PI,
        env->left_wheel_speed, env->right_wheel_speed,
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
