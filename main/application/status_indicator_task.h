#ifndef AMICE_STATUS_INDICATOR_TASK_H
#define AMICE_STATUS_INDICATOR_TASK_H

#include <core/task.h>

/**
 * Task which indicates the system status using LEDs
 */
class StatusIndicatorTask final
        : core::Task<> {
public:
    StatusIndicatorTask();

    ~StatusIndicatorTask() final = default;

    [[noreturn]] void Run() final;
};

#endif //AMICE_STATUS_INDICATOR_TASK_H
