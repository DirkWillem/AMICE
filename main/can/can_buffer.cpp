#include "can_buffer.h"

#include "driver/timer.h"

namespace can {

Buffer::Buffer()
        : m_count{0} {}

void Buffer::Push(const can_message_t& msg) {
    // Ignore CAN2.0 B
    if (msg.extd)
        return;

    // Get current time
    uint64_t time;
    timer_get_counter_value(TIMER_GROUP_0, TIMER_0, &time);

    // Write data to buffer
    m_buffer[m_count].data = *reinterpret_cast<const uint64_t*>(msg.data);
    m_buffer[m_count].len = msg.data_length_code;
    m_buffer[m_count].timestamp = time;
    m_buffer[m_count].id = static_cast<uint16_t>(msg.identifier);
    ++m_count;
}

void Buffer::Reset() {
    m_count = 0;
}

const CANFrame& Buffer::operator[](size_t index) const {
    if (index < m_count)
        return m_buffer[index];
    return m_buffer[0];
}

size_t Buffer::count() const {
    return m_count;
}

bool Buffer::ready_for_swap() const {
    return m_count >= CANBufferSwapSize;
}

bool Buffer::full() const {
    return m_count >= CANBufferSize;
}

}
