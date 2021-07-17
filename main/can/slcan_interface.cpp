#include "slcan_interface.h"

namespace can {

static constexpr char CR = 13;
static constexpr char Bell = 7;

using Cmd = SlCANCommandCode;

SlCANInterface::SlCANInterface(io::UART& uart, const core::Timer& timer)
    : m_uart{uart}, m_timer{timer},
      m_cfg_g{}, m_cfg_t{}, m_cfg_f{}, m_mode{Mode::Closed}, m_auto_poll{false}, m_timestamp{false} {
    m_cfg_g = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, CAN_MODE_NORMAL);
    m_cfg_t = CAN_TIMING_CONFIG_500KBITS();
    m_cfg_f = CAN_FILTER_CONFIG_ACCEPT_ALL();
}

void SlCANInterface::SendFrame(const can_message_t& msg) {
    if (m_mode == Mode::Closed || !m_auto_poll) {
        return;
    }

    // Print command + ID
    if (msg.extd) {
        m_uart.Write(msg.rtr ? Cmd::Rtr29 : Cmd::Tx29);

        char id_buf[9]{0};
        sprintf(id_buf, "%08X", msg.identifier);
        m_uart.Write(id_buf, 8);
    } else {
        m_uart.Write(msg.rtr ? Cmd::Rtr11 : Cmd::Tx11);

        char id_buf[4]{0};
        sprintf(id_buf, "%03X", msg.identifier);
        m_uart.Write(id_buf, 3);
    }

    // Print data length
    m_uart.Write(static_cast<char>(msg.data_length_code + '0'));

    // If we have data, print it
    if (!msg.rtr) {
        char data_buf[3]{0};
        for (uint8_t i = 0; i < msg.data_length_code; i++) {
            sprintf(data_buf, "%02X", msg.data[i]);
            m_uart.Write(data_buf, 2);
        }
    }

    // If required, write a timestamp
    if (m_timestamp) {
        auto t = m_timer.millis() % 60000;
        char timer_buf[5]{0};
        sprintf(timer_buf, "%04X", static_cast<uint32_t>(t));
        m_uart.Write(timer_buf, 4);
    }

    m_uart.Write(CR);
}

void SlCANInterface::ProcessCommand() {
    auto cmd_opt = m_uart.ReadOrWait<Cmd>(500);
    if (!cmd_opt) return;

    m_uart.Write(*cmd_opt);

    auto val = HandleCommand(*cmd_opt);

    if (val != Result::Error) {
        // Next value must be CR
        const auto ch = m_uart.Read<char>();
        if (!ch || *ch != CR) {
            val = Result::Error;
        }
    }

    // If we didn't encounter a CR, clear buffer until next cr
    while (val == Result::Error && m_uart.has_data()) {
        const auto ch = m_uart.Read<char>();
        if (ch && *ch == CR) {
            break;
        }
    }

    switch (val) {
        case Result::Ok:
            m_uart.Write(CR);
            break;
        case Result::OkUpper:
            m_uart.Write('Z');
            m_uart.Write(CR);
            break;
        case Result::OkLower:
            m_uart.Write('z');
            m_uart.Write(CR);
            break;
        case Result::Error:
            m_uart.Write(Bell);
            break;
    }
}

SlCANInterface::Result SlCANInterface::HandleCommand(SlCANCommandCode cmd) {
    switch (cmd) {
        case Cmd::Setup:
            return HandleSetupCommand(cmd);
        case Cmd::SetupBtr: return Result::Error;
        case Cmd::Open:
            m_mode = Mode::Open;
            return Result::Ok;
        case Cmd::Listen:
            m_mode = Mode::ListenOnly;
            return Result::Ok;
        case Cmd::Close:
            m_mode = Mode::Closed;
            return Result::Ok;
        case Cmd::Tx11: [[fallthrough]];
        case Cmd::Tx29: [[fallthrough]];
        case Cmd::Rtr11: [[fallthrough]];
        case Cmd::Rtr29:
            return HandleTransmitCommand(cmd);
        case Cmd::PollOne: {
            can_message_t msg{};
            msg.identifier = 0x123;
            msg.data_length_code = 3;
            msg.data[0] = 0x12;
            msg.data[1] = 0x34;
            msg.data[2] = 0xBA;
            SendFrame(msg);
            return Result::Ok;
        }
        case Cmd::PollMany: {
            can_message_t msg{};
            msg.identifier = 0x345;
            msg.data_length_code = 3;
            msg.data[0] = 0x12;
            msg.data[1] = 0x34;
            msg.data[2] = 0xBA;
            SendFrame(msg);
            return Result::Ok;
        }
        case Cmd::Flags:
            m_uart.Write("F00", 3);
            return Result::Ok;
        case Cmd::Autopoll: {
            auto n = m_uart.Read<char>();
            if (!n) return Result::Error;
            if (*n == '0') {
                m_auto_poll = false;
            } else if (*n == '1') {
                m_auto_poll = true;
            } else {
                return Result::Error;
            }
            return Result::Ok;
        }
        case Cmd::Filter: [[fallthrough]];
        case Cmd::AccCode: [[fallthrough]];
        case Cmd::AccMask:
            return Result::Ok;
        case Cmd::Uart:
            return HandleUartSetting(cmd);
        case Cmd::Version:
            m_uart.Write("0101", 4);
            return Result::Ok;
        case Cmd::Serial:
            m_uart.Write("APEX", 4);
            return Result::Ok;
        case Cmd::Timestamp:
        {
            auto n = m_uart.Read<char>();
            if (!n) return Result::Error;
            if (*n == '0') {
                m_timestamp = false;
            } else if (*n == '1') {
                m_timestamp = true;
            } else {
                return Result::Error;
            }
            return Result::Ok;

        }
        case Cmd::Autostart:
            return Result::Ok;
        default:
            return Result::Error;
    }
}

SlCANInterface::Result SlCANInterface::HandleSetupCommand(SlCANCommandCode cmd) {
    // Determine timing config depending on received command
    const auto n = m_uart.Read<char>();
    if (!n)
        return Result::Error;

    switch (*n) {
        case '0': [[fallthrough]];
        case '1': return Result::Error;
        case '2': m_cfg_t = CAN_TIMING_CONFIG_50KBITS(); break;
        case '3': m_cfg_t = CAN_TIMING_CONFIG_100KBITS(); break;
        case '4': m_cfg_t = CAN_TIMING_CONFIG_125KBITS(); break;
        case '5': m_cfg_t = CAN_TIMING_CONFIG_250KBITS(); break;
        case '6': m_cfg_t = CAN_TIMING_CONFIG_500KBITS(); break;
        case '7': m_cfg_t = CAN_TIMING_CONFIG_800KBITS(); break;
        case '8': m_cfg_t = CAN_TIMING_CONFIG_1MBITS(); break;
        default: return Result::Error;
    }

    // Stop and uninstall CAN driver if it is running
    if (can_state() == CAN_STATE_RUNNING) {
        if (can_stop() != ESP_OK) return Result::Error;
    }

    if (can_state() == CAN_STATE_STOPPED) {
        if (can_driver_uninstall() != ESP_OK) return Result::Error;
    }


    // Install driver again
    if (can_driver_install(&m_cfg_g, &m_cfg_t, &m_cfg_f) != ESP_OK)
        return Result::Error;
    if (can_start() != ESP_OK)
        return Result::Error;

    return Result::Ok;
}

SlCANInterface::Result SlCANInterface::HandleTransmitCommand(SlCANCommandCode cmd) {
    static_assert(sizeof(uint64_t) == sizeof(decltype(strtoull("", nullptr, 0))),
            "strtoull must return uint64_t");

    can_message_t msg{};

    // Read CAN ID
    char buf[9]{0};

    switch (cmd) {
        case Cmd::Tx11: [[fallthrough]];
        case Cmd::Rtr11:
            if (!m_uart.ReadArray<char>(&buf[0], 3)) {
                return Result::Error;
            }
            break;
        case Cmd::Tx29: [[fallthrough]];
        case Cmd::Rtr29:
            msg.extd = true;
            if (!m_uart.ReadArray<char>(&buf[0], 8)) {
                return Result::Error;
            }
            break;
        default: return Result::Error;
    }

    msg.identifier = static_cast<uint32_t>(strtoul(buf, nullptr, 16));

    // Read data length
    const auto len_opt = m_uart.Read<char>();
    if (!len_opt) {
        return Result::Error;
    }
    msg.data_length_code = *len_opt - '0';

    // If we're transmitting data, read data bytes. Otherwise, set RTR bit
    if (cmd == Cmd::Tx11 || cmd == Cmd::Tx29) {
        char data_buf[3]{0};

        for (auto i = 0; i < msg.data_length_code; i++) {
            if (!m_uart.ReadArray<char>(&data_buf[0], 2)) {
                return Result::Error;
            }

            msg.data[i] = strtoul(data_buf, nullptr, 16);
        }
    } else {
        msg.rtr = true;
    }

    // Send the message
    if (can_transmit(&msg, 1000) != ESP_OK) {
        return Result::Error;
    }

    // Send proper result
    if (!m_auto_poll) {
        return Result::Ok;
    } else {
        if (cmd == Cmd::Tx11 || cmd == Cmd::Rtr11) {
            return Result::OkLower;
        } else {
            return Result::OkUpper;
        }
    }

}

SlCANInterface::Result SlCANInterface::HandleUartSetting(SlCANCommandCode cmd) {
    auto n = m_uart.Read<char>();
    if (!n) return Result::Error;

    switch (*n) {
        case '0': m_uart.SetBaudRate(230400); break;
        case '1': m_uart.SetBaudRate(115200); break;
        case '2': m_uart.SetBaudRate(57600); break;
        case '3': m_uart.SetBaudRate(38400); break;
        case '4': m_uart.SetBaudRate(19200); break;
        case '5': m_uart.SetBaudRate(9600); break;
        case '6': m_uart.SetBaudRate(2400); break;
        default: return Result::Error;
    }

    return Result::Ok;
}

can_state_t SlCANInterface::can_state() const {
    can_status_info_t status{};
    can_get_status_info(&status);
    return status.state;
}


}