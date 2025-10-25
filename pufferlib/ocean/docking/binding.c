#include "docking.h"

#define Env Docking
#include "../env_binding.h"

static int my_init(Env *env, PyObject *args, PyObject *kwargs) {
  // No custom initialization needed
  return 0;
}

static int my_log(PyObject *dict, Log *log) {
  assign_to_dict(dict, "score", log->score);
  return 0;
}
