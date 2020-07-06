#include "can_reader_task.h"

#include <driver/can.h>
#include <esp_log.h>

CANReaderTask::CANReaderTask(can::Buffer& a, can::Buffer& b, core::Timer& timer, CANEventGroup& event_group)
    : Task<>{"canreader", core::Core::Core1},
        m_buf_a{a}, m_buf_b{b}, m_buf_cur{&a}, m_timer{timer}, m_event_group{event_group} {}

[[noreturn]] void CANReaderTask::Run() {
    can_message_t can_msg{};
    esp_err_t err;

    while(true) {
        auto t0 = m_timer.millis();

        bool writer_ready = false;

        // receive message until the buffer is full, or we hit the timeout
        while (m_timer.millis() - t0 < MaxSwapInterval && !m_buf_cur->full()) {
            // Receive message
            err = can_receive(&can_msg, pdMS_TO_TICKS(ReadTimeout));
            if (err == ESP_OK) {
                m_buf_cur->Push(can_msg, m_timer.millis());
            } else if (err != ESP_ERR_TIMEOUT) {
                ESP_LOGE("CanReader", "Failure in can_receive: %d", err);
            }

            // If we are ready for a swap, indicate if we didn't yet
            if (m_buf_cur->ready_for_swap()
                && m_event_group.Await(CANEvent::WriterReady, SwapTimeout, true)) {
                writer_ready = true;
                break;
            }
        }

        //ESP_LOGI("CanReader", "Done reading");

        // Wait for the writer to be ready
        while (!writer_ready) {
            writer_ready = m_event_group.Await(CANEvent::WriterReady, ReadTimeout, true);
        }

        // Swap buffers
        if (m_buf_cur == &m_buf_a) {
            m_buf_cur = &m_buf_b;
            //ESP_LOGI("CanReader", "Releasing buffer A");
            m_event_group.Raise(CANEvent::ReaderReleasedA);
        } else {
            m_buf_cur = &m_buf_a;
            //ESP_LOGI("CanReader", "Releasing buffer B");
            m_event_group.Raise(CANEvent::ReaderReleasedB);
        }


    }
}