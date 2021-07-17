#ifndef AMICE_UART_H
#define AMICE_UART_H

#include <optional>

extern "C" {
#include "driver/uart.h"
#include "driver/gpio.h"
}


namespace io {

class UART {
public:
    /**
     * Constructor
     * @param tx TX pin
     * @param rx RX pin
     * @param port UART port
     * @param baud Baud rate
     * @param word_length Word length
     * @param parity Parity
     * @param stop_bits Number of stop bits
     * @param flow_ctrl Flow control setting
     */
    UART(
            gpio_num_t tx,
            gpio_num_t rx,
            uart_port_t port,
            int baud = 9600,
            uart_word_length_t word_length = UART_DATA_8_BITS,
            uart_parity_t parity = UART_PARITY_DISABLE,
            uart_stop_bits_t stop_bits = UART_STOP_BITS_1,
            uart_hw_flowcontrol_t flow_control = UART_HW_FLOWCTRL_DISABLE,
            int buf_size = 2048);

    /**
     * Destructor
     */
    ~UART();

    /**
     * Sets the UART baud rate
     * @param baud_rate Baud rate to set
     */
    void SetBaudRate(uint32_t baud_rate);

    template<typename T>
    /**
     * Writes data over UART
     * @tparam T data type to write
     * @param data Data to write
     */
    void Write(const T& data) {
        uart_write_bytes(m_port, reinterpret_cast<const char*>(&data), sizeof(T));
    }

    /**
     * Writes data over UART
     * @param data Data to write
     * @param len Length of the data to write
     */
    void Write(const char* data, size_t len);

    template<typename T>
    /**
     * Reads data from UART
     * @return Read data, or std::nullopt if no data available
     */
    std::optional<T> ReadOrWait(uint32_t timeout) {
        T value{};
        if (uart_read_bytes(m_port, reinterpret_cast<uint8_t*>(&value), sizeof(T), pdMS_TO_TICKS(timeout)) == sizeof(T)) {
            return value;
        }
        return std::nullopt;
    }

    template<typename T>
    /**
     * Reads data from UART
     * @return Read data, or std::nullopt if no data available
     */
    std::optional<T> Read() {
        T value{};
        if (Read<T>(value))
            return value;
        return std::nullopt;
    }

    template<typename T>
    /**
     * Reads data form UART into the supplied buffer
     * @tparam T data type to read
     * @param into Data to read into
     * @return Whether data could be read
     */
    bool Read(T& into) {
        // Get data buffer length
        size_t size{};
        uart_get_buffered_data_len(m_port, &size);

        // If we have enough data, read
        if (size >= sizeof(T) && uart_read_bytes(m_port, reinterpret_cast<uint8_t*>(&into), sizeof(T), 100) == sizeof(T)) {
            ;
            return true;
        }

        // If we didn't have enough data, return false
        return false;
    }

    template<typename T>
    bool ReadArray(T* into, size_t n) {
        size_t size{};
        uart_get_buffered_data_len(m_port, &size);

        return size >= n * sizeof(T) &&
               uart_read_bytes(m_port, reinterpret_cast<uint8_t*>(into), n * sizeof(T), 100) == sizeof(T);

    }

    /**
     * returns whether the UART buffer currently contains data
     * @return Whether the UART buffer contains data
     */
    [[nodiscard]] bool has_data() const;

private:
    uart_port_t m_port;
};


}


#endif //AMICE_UART_H
