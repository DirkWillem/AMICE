#ifndef AMICE_ERA_CAN_DATA_H
#define AMICE_ERA_CAN_DATA_H

#include "../can_data_handler.h"
#include "../can_consecutive_data_handler.h"

#include <canspec/era/era_canspec.h>
#include <canspec/era/era_matcher.h>


namespace api {

struct EraInfoMatcher {
    static bool Match(const can::BusInfo& bus_info) {
        return bus_info.is_stella_era;
    }
};

struct EraGlobalInfoHandler
        : public CANDataHandler<EraInfoMatcher, CANMessageList<
                era::BMS_MinMaxCellVoltage,
                era::BMS_MinMaxCellTemp,
                era::LVC_VehicleState,
                era::BMS_SoCSoH,
                era::BMS_BatteryPower,
                era::ChC_MainsState,
                era::SSB_Lights,
                era::BMS_State>> {
    static constexpr const char* URI = "/data/era/global/info";
};


struct EraBatteryInfoHandler
        : public CANDataHandler<EraInfoMatcher, CANMessageList<
                era::BMS_Version,
                era::BMS_State,
                era::BMS_MinMaxCellVoltage,
                era::BMS_MinMaxCellTemp,
                era::BMS_IMDStatus,
                era::LVC_VehicleState,
                era::BMS_Current,
                era::BMS_BatteryPower,
                era::BMS_SoCSoH,
                era::BMS_HVMeasurements
        >> {
    static constexpr const char* URI = "/data/era/batt/info";
};

struct EraBatteryStateHandler
        : public CANDataHandler<EraInfoMatcher, CANMessageList<
                era::BMS_Version,
                era::BMS_State,
                era::BMS_SubStates,
                era::BMS_Inputs,
                era::BMS_OverridesAndContactors,
                era::BMS_SatInterfaceState,
                era::BMS_SatBalancingControl,
                era::BMS_SatPCBTemperatures,
                era::BMS_PCBTemperatures,
                era::BMS_IVTOCSettings
        >> {
    static constexpr const char* URI = "/data/era/batt/state";
};

struct EraBatteryCellDataHandler
  : public CANConsecutiveDataHandler<EraInfoMatcher, 1080, 1091> {
  static constexpr const char* URI = "/data/era/batt/cells";
};

}

#endif //AMICE_ERA_H
