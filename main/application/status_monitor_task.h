#ifndef AMICE_STATUS_MONITOR_TASK_H
#define AMICE_STATUS_MONITOR_TASK_H

#include <core/task.h>
#include <core/timer.h>
#include <can/can_bus_status.h>
#include <can/can_store.h>

constexpr uint32_t LogicInterval = 100;
constexpr uint32_t NoBusBlinkInterval = 500;
constexpr uint32_t BusBlinkInterval = 100;
constexpr uint32_t BusMonitorInterval = 1000;

/**
 * Task which keeps track of the system status, and controls the status LED
 */
class StatusMonitorTask final
        : core::Task<> {
public:
    /**
     * Constructor
     */
    StatusMonitorTask(const can::Store& store, const core::Timer& timer, can::BusStatus& bus_status);

    /**
     * Destructor
     */
    ~StatusMonitorTask() final = default;

    /**
     * Status monitor run task
     */
    [[noreturn]] void Run() final;

private:
    const can::Store& m_store;
    const core::Timer& m_timer;
    can::BusStatus& m_bus_status;
};

#endif //AMICE_STATUS_MONITOR_TASK_H
