#include "can_store.h"

namespace can {

Store::Store()
        : m_a_frames{}, m_mutex{} {}

void Store::Put(const CANFrame& frame) {
    while(true) {
        if (m_mutex.Lock()) break;
    }

    PutNoMtx(frame);

    m_mutex.Unlock();
}

void Store::Put(const Buffer& buffer) {
    while(true) {
        if (m_mutex.Lock()) break;
    }

    for (size_t i = 0; i < buffer.count(); i++)
        PutNoMtx(buffer[i]);

    m_mutex.Unlock();
}

std::optional<CANFrame> Store::frame(uint16_t id) const {
    if (m_a_frames[id].empty)
        return std::nullopt;
    return m_a_frames[id];
}

void Store::PutNoMtx(const CANFrame& frame) {
    auto& f = m_a_frames[frame.id];
    f.data = frame.data;
    f.empty = false;
    f.id = frame.id;
    f.len = frame.len;
    f.timestamp = f.timestamp;
}


}
