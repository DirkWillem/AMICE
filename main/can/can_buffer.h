#ifndef AMICE_CAN_BUFFER_H
#define AMICE_CAN_BUFFER_H

#include <array>

#include <driver/can.h>

#include "can_frame.h"

namespace can {

static constexpr size_t CANBufferSize = 80;
static constexpr size_t CANBufferSwapSize = 64;
/**
 * Class for temporarily storing CAN messages before they're written to the store
 */
class Buffer {
    static_assert(CANBufferSize >= CANBufferSwapSize, "CAN buffer size must be larger than the swap trigger size");
public:
    Buffer();

    /**
     * Pushes a CAN message to the buffer
     * @param msg Message to push
     * @param t Current timestamp
     */
    void Push(const can_message_t& msg, uint32_t t);

    /**
     * Resets the buffer
     */
    void Reset();

    /**
     * Indexing operator
     * @param index Index of the CAN message to read
     * @return message at index
     */
    const CANFrame& operator[](size_t index) const;

    /**
     * Returns the number of elements in the buffer
     * @return buffer count
     */
    [[nodiscard]] size_t count() const;

    /**
     * Returns whether the buffer is ready for a swap
     * @return Whether the buffer is ready for a swap
     */
    [[nodiscard]] bool ready_for_swap() const;

    /**
     * Returns whether the buffer is full
     * @return Whether the buffer is full
     */
    [[nodiscard]] bool full() const;
private:
    size_t m_count;
    std::array<CANFrame, CANBufferSize> m_buffer;
};

}


#endif //AMICE_CAN_BUFFER_H
