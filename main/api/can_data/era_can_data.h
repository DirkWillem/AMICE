#ifndef AMICE_ERA_CAN_DATA_H
#define AMICE_ERA_CAN_DATA_H

#include "../can_data_handler.h"

#include <canspec/era/era_canspec.h>
#include <canspec/era/era_matcher.h>


namespace api {

using EraBatteryData = CANMessageList<
        era::BMS_State,
        era::BMS_MinMaxCellVoltage,
        era::BMS_MinMaxCellTemp,
        era::BMS_IMDStatus,
        era::LVC_VehicleState>;

struct EraBatteryDataHandler
        : public CANDataHandler<era::CANBusMatcher, EraBatteryData> {
    static constexpr const char* URI = "/data/era/battery";
};

}

#endif //AMICE_ERA_H
