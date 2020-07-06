#include "status_monitor_task.h"

#include <core/utils.h>
#include <io/gpo.h>

StatusMonitorTask::StatusMonitorTask(const can::Store& store, const core::Timer& timer, can::BusStatus& bus_status)
    : Task<>{"StatusIndicator"}, m_store{store}, m_timer{timer}, m_bus_status{bus_status} {}

[[noreturn]] void StatusMonitorTask::Run() {
    io::GPO status_led{GPIO_NUM_2};

    bool led_status{false};
    bool bus_connected{false};

    uint32_t t_led{static_cast<uint32_t>(m_timer.millis())};
    uint32_t t_status{static_cast<uint32_t>(m_timer.millis())};

    while(true) {
        const auto t = static_cast<uint32_t>(m_timer.millis());

        if ((bus_connected && t - t_led >= BusBlinkInterval)
            || (!bus_connected && t - t_led >= NoBusBlinkInterval)) {
            status_led = led_status;
            led_status = !led_status;
            t_led = t;
        }

        if (t - t_status >= BusMonitorInterval) {
            m_bus_status.CalculateBusStatus(m_store, t);
        }

        core::WaitMs(LogicInterval);
    }
}