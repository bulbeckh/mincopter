#include <experimental/mincopter/ExperimentalRuntime.h>

namespace experimental_mincopter {

bool initialize_runtime(ExperimentalRuntime &runtime) {
    if (!runtime.init_hal()) {
        return false;
    }
    if (!runtime.assemble_runtime()) {
        return false;
    }
    if (!runtime.create_tasks()) {
        return false;
    }
    return true;
}

}  // namespace experimental_mincopter
