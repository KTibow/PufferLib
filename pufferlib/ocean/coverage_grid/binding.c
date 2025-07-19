#include "coverage_grid.h"

#define Env CoverageGrid
#include "../env_binding.h"

static int my_init(Env* env, PyObject* args, PyObject* kwargs) {
    env->grid_width = (int)unpack(kwargs, "grid_width");
    env->grid_height = (int)unpack(kwargs, "grid_height");
    env->max_steps = (int)unpack(kwargs, "max_steps");
    
    // Parse obs_type string to integer
    PyObject* obs_type_obj = PyDict_GetItemString(kwargs, "obs_type");
    if (obs_type_obj && PyUnicode_Check(obs_type_obj)) {
        const char* obs_type_str = PyUnicode_AsUTF8(obs_type_obj);
        if (strcmp(obs_type_str, "position") == 0) {
            env->obs_type = 0;  // OBS_TYPE_POSITION
            env->obs_size = 2;
        } else if (strcmp(obs_type_str, "distances") == 0) {
            env->obs_type = 1;  // OBS_TYPE_DISTANCES
            env->obs_size = 4;
        } else {
            env->obs_type = 0;  // Default to position
            env->obs_size = 2;
        }
    } else {
        env->obs_type = 0;  // Default to position
        env->obs_size = 2;
    }
    
    // Initialize grid state
    env->total_cells = env->grid_width * env->grid_height;
    env->visited = (unsigned char*)calloc(env->total_cells, sizeof(unsigned char));
    
    return 0;
}

static int my_log(PyObject* dict, Log* log) {
    assign_to_dict(dict, "coverage_percentage", log->coverage_percentage);
    assign_to_dict(dict, "total_reward", log->total_reward);
    assign_to_dict(dict, "collisions", log->collisions);
    assign_to_dict(dict, "episode_return", log->episode_return);
    assign_to_dict(dict, "episode_length", log->episode_length);
    return 0;
}