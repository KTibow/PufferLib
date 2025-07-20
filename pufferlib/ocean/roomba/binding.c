#include "roomba.h"

#define Env Roomba
#include "../env_binding.h"

static int my_init(Env* env, PyObject* args, PyObject* kwargs) {
    env->room_width = unpack(kwargs, "room_width");
    env->room_height = unpack(kwargs, "room_height");
    env->max_steps = (int)unpack(kwargs, "max_steps");
    return 0;
}

static int my_log(PyObject* dict, Log* log) {
    assign_to_dict(dict, "collisions", log->collisions);
    assign_to_dict(dict, "episode_return", log->episode_return);
    assign_to_dict(dict, "episode_length", log->episode_length);
    assign_to_dict(dict, "dirt_collected", log->dirt_collected);
    return 0;
}