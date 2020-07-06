#ifndef AMICE_CAN_STORE_H
#define AMICE_CAN_STORE_H

#include <array>
#include <optional>

#include <core/mutex.h>

#include "can_frame.h"
#include "can_buffer.h"

#include <esp_log.h>

namespace can {

/**
 * Store for all CAN messages
 */
class Store {
    // Assertions
    static_assert(sizeof(CANFrame) == 16, "CANFrame must be 16 bytes large");
public:
    Store();

    /**
     * Puts a frame to the store
     * @param frame Frame to put
     */
    void Put(const CANFrame& frame);

    /**
     * Puts all items in the buffer to the store
     * @param buffer Buffer to put
     */
    void Put(const Buffer& buffer);

    template<uint16_t... Ids>
    void ToBuffer(std::array<CANFrame, sizeof...(Ids)>& buffer) const {
        while(true) {
            if (m_mutex.Lock()) break;
        }

        size_t index{0};
        (MessageToBuffer<Ids, sizeof...(Ids)>(buffer, index), ...);

        m_mutex.Unlock();
    }

    /**
     * Returns a CAN frame
     * @param id ID of the frame to get
     * @return Frame if it was received, std::nullopt otherwise
     */
    [[nodiscard]] std::optional<CANFrame> frame(uint16_t id) const;

    template<typename Msg>
    /**
     * Returns a deserialized CAN message
     * @tparam Msg CAN message type
     * @return Deserialized message, or std::nullopt if the message wasn't received
     */
    [[nodiscard]] std::optional<Msg> message() const {
        auto f = frame(Msg::ID);
        if (f) {
            return *reinterpret_cast<Msg*>(&(*f));
        }

        return std::nullopt;
    }

private:
    template<uint16_t Id, size_t NMsgs>
    void MessageToBuffer(std::array<CANFrame, NMsgs>& buffer, size_t& index) const {
        buffer[index].timestamp = m_a_frames[Id].timestamp;
        buffer[index].id = m_a_frames[Id].id;
        buffer[index].empty = m_a_frames[Id].empty;
        buffer[index].len = m_a_frames[Id].len;
        buffer[index].data = m_a_frames[Id].data;

        index++;
    }

    void PutNoMtx(const CANFrame& frame);

    /** Array containing all possible CAN 2.0A frames */
    std::array<CANFrame, 2048> m_a_frames;

    mutable core::Mutex m_mutex;
};

}



#endif //AMICE_CAN_STORE_H
