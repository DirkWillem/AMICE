#ifndef AMICE_SLCAN_INTERFACE_H
#define AMICE_SLCAN_INTERFACE_H

extern "C" {
#include "driver/can.h"
}

#include <io/uart.h>
#include <core/timer.h>

#include "can_frame.h"

namespace can {

enum class SlCANCommandCode : char {
    Setup = 'S',
    SetupBtr = 's',
    Open = 'O',
    Listen = 'L',
    Close = 'C',
    Tx11 = 't',
    Tx29 = 'T',
    Rtr11 = 'r',
    Rtr29 = 'R',
    PollOne = 'P',
    PollMany = 'A',
    Flags = 'F',
    Autopoll = 'X',
    Filter = 'W',
    AccCode = 'M',
    AccMask = 'm',
    Uart = 'U',
    Version = 'V',
    Serial = 'N',
    Timestamp = 'Z',
    Autostart = 'Q'
};

/**
 * slcan Interface
 */
class SlCANInterface {
private:
    enum class Result {
        Error,
        Ok,
        OkLower,
        OkUpper
    };

public:
    /**
     * Constructor
     * @param uart UART interface to use
     * @param timer Timer
     */
    SlCANInterface(io::UART& uart, const core::Timer& timer);

    /**
     * Sends a frame over USB
     * @param frame Frame to send
     */
    void SendFrame(const can_message_t& frame);

    /**
     * Processes incoming commands
     */
    void ProcessCommand();

private:
    io::UART& m_uart;
    const core::Timer& m_timer;

    Result HandleCommand(SlCANCommandCode cmd);

    Result HandleSetupCommand(SlCANCommandCode cmd);

    Result HandleTransmitCommand(SlCANCommandCode cmd);

    Result HandleUartSetting(SlCANCommandCode cmd);

    [[nodiscard]] can_state_t can_state() const;

    enum class Mode {
        Closed,
        Open,
        ListenOnly
    };

    can_general_config_t m_cfg_g;
    can_timing_config_t m_cfg_t;
    can_filter_config_t m_cfg_f;
    Mode m_mode;
    bool m_auto_poll;
    bool m_timestamp;
};

}

#endif //AMICE_SLCAN_INTERFACE_H
