#ifndef AMICE_CAN_FRAME_H
#define AMICE_CAN_FRAME_H

#include <cstdint>

namespace can {

#pragma pack(push, 2)
struct CANFrame {
    uint32_t timestamp;
    uint16_t id;
    bool empty: 1;
    uint8_t pad: 3;
    uint8_t len: 4;
    uint64_t data;

    CANFrame();
};
#pragma pack(pop)

}



#endif //AMICE_CAN_FRAME_H
