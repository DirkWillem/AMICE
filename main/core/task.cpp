#include "task.h"

namespace core {

void TaskBase::_StaticRun(void* ptr) {
    static_cast<TaskBase*>(ptr)->Run();
}

}
