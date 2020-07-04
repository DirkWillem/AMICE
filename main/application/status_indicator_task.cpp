#include "status_indicator_task.h"

#include <core/utils.h>
#include <io/gpo.h>

StatusIndicatorTask::StatusIndicatorTask()
    : Task<>{"StatusIndicator"} {}

[[noreturn]] void StatusIndicatorTask::Run() {
    io::GPO status_led{GPIO_NUM_2};

    // Blink LED at 300 ms
    while(true) {
        status_led = true;
        core::WaitMs(300);
        status_led = false;
        core::WaitMs(300);
    }
}