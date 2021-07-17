#include "status_monitor_task.h"

#include <config.h>
#include <core/utils.h>
#include <io/gpo.h>
#include <io/gpi.h>

StatusMonitorTask::StatusMonitorTask(const can::Store& store, const core::Timer& timer, can::BusStatus& bus_status)
    : Task<>{"StatusIndicator"}, m_store{store}, m_timer{timer}, m_bus_status{bus_status} {}

[[noreturn]] void StatusMonitorTask::Run() {
    io::GPO led1r{pinout::LED1R};
    io::GPO led1g{pinout::LED1G};
    io::GPO led1b{pinout::LED1B};
    io::GPO led2r{pinout::LED2R};
    io::GPO led2g{pinout::LED2G};
    io::GPO led2b{pinout::LED2B};

    io::GPI btn_l{pinout::BTN_L};
    io::GPI btn_r{pinout::BTN_R};

    led1r = true;
    led1g = true;
    led1b = true;
    led2r = true;
    led2g = true;
    led2b = true;

    bool led_status{false};
    uint8_t led_idx{0};
    bool bus_connected{false};

    uint32_t t_led{static_cast<uint32_t>(m_timer.millis())};
    uint32_t t_status{static_cast<uint32_t>(m_timer.millis())};

    while(true) {
        const auto t = static_cast<uint32_t>(m_timer.millis());

       //if ((bus_connected && t - t_led >= BusBlinkInterval)
       //     || (!bus_connected && t - t_led >= NoBusBlinkInterval)) {
//            status_led = led_status;
            led2r = led_idx != 0;
            led2b = led_idx != 1;
            led2g = led_idx != 2;
            led1r = led_idx != 0;
            led1b = led_idx != 1;
            led1g = led_idx != 2;
//            led1g = false;
//            led2g = false;
            led_status = !led_status;
            t_led = t;

            led_idx++;
            led_idx %= 3;
        //}

//        if (t - t_status >= BusMonitorInterval) {
//            m_bus_status.CalculateBusStatus(m_store, t);
//        }

        core::WaitMs(LogicInterval);
    }
}