#ifndef AMICE_CAN_READER_TASK_H
#define AMICE_CAN_READER_TASK_H

#include <can/can_buffer.h>
#include <core/timer.h>
#include <core/task.h>

#include "can_events.h"

static uint64_t MaxSwapInterval = 200;
static uint64_t ReadTimeout = 100;
static uint64_t SwapTimeout = 10;

class CANReaderTask final
    : public core::Task<> {
public:
    /**
     * Constructor
     * @param a Buffer A
     * @param b Buffer B
     */
    CANReaderTask(can::Buffer& a, can::Buffer& b, core::Timer& timer, CANEventGroup& event_group);

    ~CANReaderTask() final = default;

    /**
     * Task run function
     */
    [[noreturn]] void Run() final;
private:
    can::Buffer& m_buf_a;
    can::Buffer& m_buf_b;
    can::Buffer* m_buf_cur;

    core::Timer& m_timer;
    CANEventGroup& m_event_group;
};

#endif //AMICE_CAN_READER_TASK_H
