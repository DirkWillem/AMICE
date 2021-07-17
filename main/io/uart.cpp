#include "uart.h"

namespace io {

UART::UART(
        gpio_num_t tx,
        gpio_num_t rx,
        uart_port_t port,
        int baud,
        uart_word_length_t word_length,
        uart_parity_t parity,
        uart_stop_bits_t stop_bits,
        uart_hw_flowcontrol_t flow_control,
        int buf_size)
        : m_port{port} {
    // Create config
    uart_config_t cfg{};
    cfg.baud_rate = baud;
    cfg.data_bits = word_length;
    cfg.parity = parity;
    cfg.stop_bits = stop_bits;
    cfg.flow_ctrl = flow_control;

    // Configure interface
    uart_param_config(port, &cfg);

    // Set pin
    ESP_ERROR_CHECK(uart_set_pin(port, tx, rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Install driver
    ESP_ERROR_CHECK(uart_driver_install(port, buf_size, buf_size, 0, nullptr, 0));
}

UART::~UART() {
    uart_driver_delete(m_port);
}

void UART::SetBaudRate(uint32_t baud_rate) {
    uart_set_baudrate(m_port, baud_rate);
}

void UART::Write(const char* data, size_t len) {
    uart_write_bytes(m_port, data, len);
}

bool UART::has_data() const {
    size_t size;
    uart_get_buffered_data_len(m_port, &size);
    return size > 0;
}


}