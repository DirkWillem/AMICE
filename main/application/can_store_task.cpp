#include "can_store_task.h"

using ReaderReleasedX = core::GroupEvents<CANEvent::ReaderReleasedA, CANEvent::ReaderReleasedB>;

CANStoreTask::CANStoreTask(can::Buffer &a, can::Buffer &b, can::Store &store, CANEventGroup &event_group)
    : Task<>{"CANStore", core::Core::Core1},
    m_buf_a{a}, m_buf_b{b}, m_store{store}, m_event_group{event_group} {}

[[noreturn]] void CANStoreTask::Run() {
    // Initially, we are always ready
    m_event_group.Raise(CANEvent::WriterReady);

    while(true) {
        // Wait for a buffer to be released
        auto& buf = CANStoreTask::WaitForReleasedBuffer();


        // Write all data to the store
        m_store.Put(buf);

        // Clear buffer, indicate we're ready
        buf.Reset();
        m_event_group.Raise(CANEvent::WriterReady);
    }
}

can::Buffer& CANStoreTask::WaitForReleasedBuffer() {
    while(true) {
        auto snapshot = m_event_group.AwaitAny(ReaderReleasedX{}, true);
        if (snapshot) {
            if (*snapshot & CANEvent::ReaderReleasedA)
                return m_buf_a;
            if (*snapshot & CANEvent::ReaderReleasedB)
                return m_buf_b;
        }
    }
}