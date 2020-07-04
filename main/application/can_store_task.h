#ifndef AMICE_CAN_STORE_TASK_H
#define AMICE_CAN_STORE_TASK_H

#include <can/can_buffer.h>
#include <can/can_store.h>
#include <core/mutex.h>
#include <core/task.h>

#include "can_events.h"

class CANStoreTask final
: public core::Task<> {
public:
    /**
     * Constructor
     * @param a Buffer A
     * @param b Buffer B
     * @param store CAN store
     * @param event_group Store event group
     */
    CANStoreTask(can::Buffer& a, can::Buffer& b, can::Store& store, CANEventGroup& event_group);

    ~CANStoreTask() final = default;

    /**
     * Task run function
     */
    [[noreturn]] void Run() final;
private:
    can::Buffer& WaitForReleasedBuffer();

    can::Buffer& m_buf_a;
    can::Buffer& m_buf_b;

    can::Store& m_store;
    CANEventGroup& m_event_group;
};


#endif //AMICE_CAN_STORE_TASK_H
