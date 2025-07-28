#include "coverage_grid.h"

#define Env CoverageGrid
#include "../env_binding.h"

static int my_init(Env* env, PyObject* args, PyObject* kwargs) {
    return 0;
}

static int my_log(PyObject* dict, Log* log) {
    assign_to_dict(dict, "coverage_percentage", log->coverage_percentage);
    assign_to_dict(dict, "collisions", log->collisions);
    assign_to_dict(dict, "episode_return", log->episode_return);
    return 0;
}
