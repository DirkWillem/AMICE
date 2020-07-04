#ifndef AMICE_CAN_EVENTS_H
#define AMICE_CAN_EVENTS_H

#include <cstdint>

#include "../core/event_group.h"

enum class CANEvent : uint8_t {
    ReaderReleasedA,
    ReaderReleasedB,
    WriterReady
};

using CANEventGroup = core::EventGroup<CANEvent>;

#endif //AMICE_CAN_EVENTS_H
