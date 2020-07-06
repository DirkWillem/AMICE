#ifndef AMICE_ERA_CANSPEC_H
#define AMICE_ERA_CANSPEC_H

#include <cstdint>

namespace era {

enum class Controller : uint8_t { Unknown, LVC, BMS, ACU, ChC, PvC, SSB, FUC, RAUX, BB, AVC, MK5, BoC, PsCB, ESB };
// ACUMode: ACU mode
enum class ACUMode : uint8_t { RDW, WSC, DARE };
// AVCState: State of the AVC
enum class AVCState : uint8_t {Unknown, Startup, ConvertersEnabled, ManualOverride, Failure};
// BrakingPrecheckState: Braking precheck state
enum class BrakingPrecheckState : uint8_t { Unknown, Inactive, CheckingPressure, WaitingForHandbrake, WaitingForActuators };
// BMSBalancingState: State of the balancing system of the BMS
enum class BMSBalancingState : uint8_t { None, Unbalanced, Balanced, Balancing, Recovering, Shutdown };
// BMSCoolingLimit: State on which the cooling limits of the BMS are based
enum class BMSCoolingLimit : uint8_t { Charging, Discharging };
// BMSError: Errors that can occur in the BMS main state machine
enum class BMSError : uint8_t { None, CellUndervoltage, CellOvervoltage, BatteryUndertemperature, BatteryOvertemperature, PCBOvertemperature, EmergencyStop, HVILError, Overcurrent, IVTOCS, SatelliteFault, SatelliteInternalFault, SatellitePCBOvertemperature, NegContactorAUXMismatch, PosContactorAUXMismatch, NegContEnableChainMismatch, PosContEnableChainMismatch, IMDStartupKl31Fault, IMDStartupDeviceError, IMDStartupTimeout, IMDKl31Fault, IMDDeviceError, IMDNoData, IMDInvalidData, IMDNotOK, SatelliteStartupTimeout, SatelliteCommunicationLoss, IVTStartupTimeout, IVTCommunicationLoss, ContactorError, PrechargeTimeout, LVCWakeLow, BatteryHVUndervoltage, UVProtectionDischargeLimitExceeded, UVProtectionChargingTimeout, PermanentUVProtection };
// BMSESSSubState: Sub-state of the ESS state of the main BMS state
enum class BMSESSSubState : uint8_t { None, WaitingForLVC, Ready };
// BMSState: State of the main BMS state machine
enum class BMSState : uint8_t { ESS, Startup, Precharge, Active };
// BMSSatState: State of the BMS sat board interface
enum class BMSSatState : uint8_t { Unknown, Configuring, Operational, Balancing, Sleep, Error, Waking, InitialMeasurement };
// BMSSatError: Errors contained in the BMS Sat interface
enum class BMSSatError : uint8_t { Unknown, None, NoneInStack, WrongNumberInStack, WakeupTimeout, NoDataReceived };
// CANOpenHeartbeatData: Data contained in a CANopen Heartbeat message
enum class CANOpenHeartbeatData : uint8_t { Null = 0x0, StoppedNoToggle = 0x04, OperationalNoToggle = 0x05, PreOperationalNoToggle = 0x7F, StoppedToggle = 0x84, OperationalToggle = 0x85, PreOperationalToggle = 0xFF };
// CANOpenInterfaceDeviceState: State of CANopen device states
enum class CANOpenInterfaceDeviceState : uint8_t { Unknown, OK, Timeout, Emergency };
// CANOpenInterfaceState: State of a CANOpenInterface
enum class CANOpenInterfaceState : uint8_t { Off, DevicesOffline, Configuring, DoneConfiguring, Operational, Error };
// CANOpenNMTCommand: Command code for a CANopen NMT message
enum class CANOpenNMTCommand : uint8_t { Null = 0x0, Operational = 0x01, Stopped = 0x02, PreOperational = 0x80, ResetNode = 0x81, ResetCommunication = 0x82 };
// CANOpenNodeState: State of a CANopen node according to the CANopen node state machine
enum class CANOpenNodeState : uint8_t { Unknown, Stopped, PreOperational, Operational };
// CANOpenSDOCommand: Different possible CANOpen SDO commands
enum class CANOpenSDOCommand : uint8_t { Null = 0x0, ExpeditedTransfer1Byte = 0x2F, ExpeditedTransfer2Bytes = 0x2B, ExpeditedTransfer3Bytes = 0x27, ExpeditedTransfer4Bytes = 0x23, Read = 0x40, ReadOK = 0x4B, WriteOK = 0x60 };
// CellOvervoltageLimit: Factor on which the cell overvoltage of the main BMS state machine is based
enum class CellOvervoltageLimit : uint8_t { None, NormalOV, BalancingOV };
// ChargingState: Current state of the ChargerFaultFlag
enum class ChargingState : uint8_t { Off, DualChargerStrategy, TripleChargerStrategy };
// ChargingError: Cause of error for an Error state of the mains charging subsystem
enum class ChargingError : uint8_t { None, ChargerFaultFlag, ChargerTimeout, RelayNotClosing, RelayWelded, BusVoltageTimeout, RestartErrorTimeout };
// ChargerVoltageSetpointState: State of the setpoint voltage
enum class ChargerVoltageSetpointState : uint8_t { None, VoltageMeasurementTimeout, SetpointOvervoltageDelay, BusVoltageStable, SetpointUndervoltageDelay };
// ChCLimitingFactor: Limiting factor of the charging current of the ChC
enum class ChCLimitingFactor : uint8_t { Protocol, BMS };
// ControlState: State of any controlling entity with control access to the ACU
enum class ControlState : uint8_t { Unknown, Off, On, TurnOff };
// CMSError: Error by the CMS
enum class CMSError : uint8_t { None, CameraDisconnected, DisplayDisconnected, UnknownError };
// CMSState: State of the CMS
enum class CMSState : uint8_t { On, Standby, Off };
// DNR: Driving direction status
enum class DNR : uint8_t { Unknown, D, N, R };
// DSSActuationState: Status of the DSS control component
enum class DSSActuationState : uint8_t { Unknown, Off, Starting, ActuatingDSS, ActuatingPrecheck, ActuationPaused, Error };
// DSSActuationError: Errors that can occur during DSS actuation
enum class DSSActuationError : uint8_t { None, DSSConnectionLoss, BrakePressed, ThrottlePressed, HandbrakeActive, BrakePedalError, ThrottlePedalError, BrakingSystemError, LeftInverterError, RightInverterError, MaxForwardVelocityExceeded, MaxBackwardVelocityExceeded, EPOS4Timeout, SACUnknownEPOS4Error, BACUnknownEPOS4Error, SACOvercurrent, BACOvercurrent, SACPowerStageProtection, BACPowerStageProtection, SACOvervoltage, BACOvervoltage, SACUndervoltage, BACUndervoltage, SACThermalOverload, BACThermalOverload, SACThermalMotorOverload, BACThermalMotorOverload, SACLogicSupplyVoltageLow, BACLogicSupplyVoltageLow, SACHardwareError, BACHardwareError, SACHallSensorNotFound, BACHallSensorNotFound, SACMissingMainSensor, BACMissingMainSensor, SACMissingCommutationSensor, BACMissingCommutationSensor, UnknownInterfaceError, _Last };
// DSSControllerError: Contains the possible errors produced by the DSS control system
enum class DSSControllerError : uint8_t { None, LVCTimeout, InterfaceStartTimeout, InterfaceConfigTimeout, ShutdownEPOS4Timeout, SwitchOnEPOS4Timeout, EPOS4Timeout, CANOpenInterfaceError, UnknownError };
// DSSControllerStartingStage: Stage of the DSS controller startup phase
enum class DSSControllerStartingStage : uint8_t { None, LVC, InterfaceStart, InterfaceConfig, ShutdownEPOS4, SwitchOnEPOS4 };
// DSSPrecheckError: Errors that can occur during DSS precheck
enum class DSSPrecheckError : uint8_t { None, PrecheckFailed, DSSConnectionLoss, BrakePressed, ThrottlePressed, HandbrakeNotActive, BrakePedalError, ThrottlePedalError, BrakingSystemError, LeftInverterError, RightInverterError, VelocityError, DNRNotN, LVCTimeout, CANOpenInterfaceStartTimeout, CANOpenInterfaceConfigTimeout, ShutdownEPOS4Timeout, SwitchOnEPOS4Timeout, EPOS4Timeout, SACUnknownEPOS4Error, BACUnknownEPOS4Error, SACOvercurrent, BACOvercurrent, SACPowerStageProtection, BACPowerStageProtection, SACOvervoltage, BACOvervoltage, SACUndervoltage, BACUndervoltage, SACThermalOverload, BACThermalOverload, SACThermalMotorOverload, BACThermalMotorOverload, SACLogicSupplyVoltageLow, BACLogicSupplyVoltageLow, SACHardwareError, BACHardwareError, SACHallSensorNotFound, BACHallSensorNotFound, SACMissingMainSensor, BACMissingMainSensor, SACMissingCommutationSensor, BACMissingCommutationSensor, UnknownInterfaceError, _Last };
// DSSState: State of the autonomous system
enum class DSSState : uint8_t {Off, Monitoring, Active};
// DualChargerStrategySubState: Substate of the dual-strategy state of the ChC
enum class DualChargerStrategySubState : uint8_t { None, Active, SetpointUndervoltageDelay };
// EPOS4OpMode: Possible operation modes of the EPOS4 motor controller
enum class EPOS4OpMode : uint8_t { PPM = 1, PVM = 3, HMM = 6, CSP = 8, CSV = 9, CST = 10 };
// IMDMeasurement:
enum class IMDMeasurement : uint8_t { Normal, Undervoltage, SST, DeviceError, Kl31Fault, Override, InvalidData, NoData };
// IMDState: State of the IMD
enum class IMDState : uint8_t { Off, Starting, StartupDelay, Operational, Error };
// InverterType: Type of inverter used on the ACU
enum class InverterType : uint8_t { Unknown, Tritium, NLE };
// IVTChargingLimitSetting: Setting of the charging limit of the IVT according to the BMS state machine
enum class IVTChargingLimitSetting : uint8_t { IVTOff, LowTempLimit, HighTempLimit, DisableCharging };
// IVTDischargingLimitSetting: Setting of the discharging limit of the IVT according to the BMS state machine
enum class IVTDischargingLimitSetting : uint8_t { IVTOff, LowTempLimit, MidTempLimit, HighTempLimit };
// IVTLimitState: State of the IVT limiting state
enum class IVTLimitState : uint8_t { IVTOff, ConfiguringDischargingLimit, ConfiguringChargingLimit, IVTOn };
// IVTStartupState: State of the IVT startup sequence
enum class IVTStartupState : uint8_t { None, Starting, WaitingForConfig, Operational, Error };
// KeyStatus: Key status
enum class KeyStatus:uint8_t {Off = 0, On, Start};
// LightingState: State of the main lighting
enum class LightingState : uint8_t { Off, DRT, LB, HB };
// LVCState: State of the LVC
enum class LVCState : uint8_t {Initializing = 0, Sleep, Error, PowerUpBMS, PowerUpHVCheck, PowerUpPrimaryLV, PowerUpPrimaryLVDelay, LVVoltageCheck, EnableLVState, SexTupleConvDelay, SexTupleConv, QuintupleConvDelay, QuintupleConv, QuadrupleConvDelay, QuadrupleConv,TripleConvDelay, TripleConv, DoubleConvDelay, DoubleConv, SingleConvDelay, SingleConv};
// MainsChargingState: State of the mains charging subsystem of the ChC
enum class MainsChargingState : uint8_t { Unknown, Off, Starting, Restarting, Ready, Charging, Releasing, Error, Recovered, Disabled, ACRecovery, Cooldown };
// MotorControlMode: Motor control mode
enum class MotorControlMode : uint8_t { Manual, Brake, Velocity, Dare, Block };
// MPPTState: States of an MPPT connected to the ChC
enum class MPPTState : uint8_t {Unknown, Off, Charging, Timeout, ReverseCurrent };
// MW3200FaultFlags: Fault flags of the MW3200
struct MW3200FaultFlags { bool fanFail: 1; bool otp: 1; bool ovp: 1; bool olp: 1; bool shorted: 1; bool acFail: 1; bool opOff: 1; bool hiTemp: 1; uint8_t _hb_pad; };
// OnOff: Generic on/off state, might be a bit more descriptive than a boolean
enum class OnOff : uint8_t { None, Off, On };
// OperationMode: Current operation mode of the ACU
enum class OperationMode : uint8_t { Unknown, Manual, CruiseControl, DSS, Block, TurnOff };
// PrimaryDisplayMode: The mode of the primary display
enum class PrimaryDisplayMode : uint8_t {WSC, RDW};
// SAEJ1772Current: Contains the possible currents signaled by the SAE J1772 protocol
enum class SAEJ1772Current : uint8_t { Unknown, None, _30A, _24A, _18A, _15A, _9_6A, _6A, EquationBased, Invalid };
// SAEJ1772State: State of the SAE J1772 protocol on the ChC
enum class SAEJ1772State : uint8_t { Unknown, Standby, Connected, Charging, Releasing };
// SatelliteStartupState: State of the BMS satellite startup sequence
enum class SatelliteStartupState : uint8_t { None, Starting, Operational, Error };
// SolarChargingError: Errors that can occur in the solar charging system
enum class SolarChargingError : uint8_t { None, EStop, UV, InterlockNotOK, ReverseCurrent };
// SolarChargingState: The state of the solar charging system
enum class SolarChargingState : uint8_t {Unknown, Enabled, ESDisabled, Error };
// TripleChargerStrategySubState: Sub-state of the triple charger strategy state
enum class TripleChargerStrategySubState : uint8_t { None, VoltageMeasurementTimeout, InitialVoltageIncrease, Active, DisbalanceResetDelay, SetpointUndervoltageDelay };
// PVCurrentMethod: Method by which the PV current on the ChC was determined
enum class PVCurrentMethod : uint8_t { Unknown, PvC, PvCClamped, NetBatt, NetBattClamped };
// UVRecoveryState: State of the UV recovery system
enum class UVRecoveryState : uint8_t { None, RecoveryDisabled, RecoveryEnabled, ChargingTimeout, Charging, PermanentProtection };
// VehicleState: The state of the vehicle
enum class VehicleState : uint8_t { Standby = 0, Demo, Auto, Drive, WSC, ESS };
// WiperSpeed: Speed of the window wiper
enum class WiperSpeed : uint8_t {Off, Slow, Fast};
// SACSetpointType: Steering actuator control types
enum class SACSetpointType : uint8_t {Torque = 0, Angle = 1};

#pragma pack(push, 1)


/*
 * LVC_VehicleState: The state of the vehicle
 */
struct LVC_VehicleState {
    static constexpr unsigned int ID = 1;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::LVC;

    LVCState lvcState; // [lvcState]
    VehicleState state; // [state]
};

/*
 * BMS_State: Main states of the BMS state machine
 */
struct BMS_State {
    static constexpr unsigned int ID = 10;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    BMSState mainState; // [BMS s t a t e]
    IMDState imdState; // [IMD s t a t e]
    BMSCoolingLimit coolingLimitState; // [Cooling l i m i t  s  t  a  t  e]
    OnOff coolingControlState; // [on/off]
    OnOff battfrontState; // [on/off]
    OnOff canControllerState; // [on/off]
    BMSError error; // [error]
    bool _dummy; // [dontcare]
};

/*
 * BMS_IMDStatus: Current IMD state,  IMD operating frequency,  IMD measured insulation according to bender datasheet
 */
struct BMS_IMDStatus {
    static constexpr unsigned int ID = 11;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    uint32_t insulationValue; // [kOhm]
    uint16_t frequencyHz; // [Hz]
    IMDMeasurement state; // [state]
};

/*
 * BMS_MinMaxCellVoltage: Minimal & Maximal  Battery MetaCell Voltages  including cell identification Identification starts from 0 on top battery layer, from 100 on middle battery layer and 200 on bottom battery layer
 */
struct BMS_MinMaxCellVoltage {
    static constexpr unsigned int ID = 12;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    uint16_t minCellVoltage; // [mV]
    uint16_t maxCellVoltage; // [mV]
    uint8_t minCellNumber; // [-]
    uint8_t maxCellNumber; // [-]
    uint16_t maxNonAdjacentBalancedCellVoltage; // [mV]
};

/*
 * BMS_MinMaxCellTemp: Minimal & Maximal  Battery MetaCell Temperatures  including cell identification Identification starts from 0 on top battery layer, from 100 on middle battery layer and 200 on bottom battery layer
 */
struct BMS_MinMaxCellTemp {
    static constexpr unsigned int ID = 13;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    int16_t minCellTemperature; // [c°C]
    int16_t maxCellTemperature; // [c°C]
    uint8_t minNTCNumber; // [-]
    uint8_t maxNTCNumber; // [-]
};

/*
 * ACU_MotorControlState: Contains the motor control state of the ACU
 */
struct ACU_MotorControlState {
    static constexpr unsigned int ID = 22;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ACU;

    MotorControlMode mode; // [mode]
    bool hillAssistActive : 1; // [T/F]
    bool leftInverterAlive : 1; // [T/F]
    bool rightInverterAlive : 1; // [T/F]
    bool leftInverterError : 1; // [T/F]
    bool rightInverterError : 1; // [T/F]
    bool throttlePedalOK : 1; // [T/F]
    bool brakePedalOK : 1; // [T/F]
    bool leftInvInternalOverTemperature : 1; // [T/F]
    bool leftInvMotorOverTemperature : 1; // [T/F]
    bool leftInvOvercurrent : 1; // [T/F]
    bool leftInvUndercurrent : 1; // [T/F]
    bool leftInvOvervoltage : 1; // [T/F]
    bool leftInvUndervoltage : 1; // [T/F]
    bool rightInvInternalOverTemperature : 1; // [T/F]
    bool rightInvMotorOverTemperature : 1; // [T/F]
    bool rightInvOvercurrent : 1; // [T/F]
    bool rightInvUndercurrent : 1; // [T/F]
    bool rightInvOvervoltage : 1; // [T/F]
    bool rightInvUndervoltage : 1; // [T/F]
};

/*
 * PvC_Output: Voltage and current output of the PvC
 */
struct PvC_Output {
    static constexpr unsigned int ID = 30;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::PvC;

    float voltage; // [V]
    float current; // [A]
};

/*
 * SSB_Lights: Current lighting state according to the SSB
 */
struct SSB_Lights {
    static constexpr unsigned int ID = 60;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::SSB;

    LightingState lights; // [State]
    bool fogLight : 1; // [On/Off]
    bool leftIndicator : 1; // [On/Off]
    bool rightIndicator : 1; // [On/Off]
};

/*
 * LVC_Version: For logging/debugging
 */
struct LVC_Version {
    static constexpr unsigned int ID = 115;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::LVC;

    uint8_t controller_major_version; // [-]
    uint8_t controller_minor_version; // [-]
    uint8_t eslib_major_version; // [-]
    uint8_t eslib_minor_version; // [-]
    uint8_t csv2cpp_major_version; // [-]
    uint8_t csv2cpp_minor_version; // [-]
};

/*
 * BMS_SubStates: Sub-states of the BMS state machine
 */
struct BMS_SubStates {
    static constexpr unsigned int ID = 200;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    BMSESSSubState essSubState; // [ESS s u b s t a t e]
    SatelliteStartupState satStartupState; // [satellite s t a r t u p  s  t  a  t  e]
    IVTStartupState ivtStartupState; // [IVT s t a r t u p  s  t  a  t  e]
    BMSBalancingState balancingState; // [balancing s t a t e]
    CellOvervoltageLimit cellOvervoltageState; // [cell o v e r v o l t a g e  s  t  a  t  e]
    UVRecoveryState uvRecoveryState; // [undervoltage r e c o v e r y  s  t  a  t  e]
};

/*
 * BMS_Inputs: Input measurements by the BMS state machine
 */
struct BMS_Inputs {
    static constexpr unsigned int ID = 201;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    bool estop : 1; // [high/low]
    bool hvilOk : 1; // [high/low]
    bool hvilMid : 1; // [high/low]
    bool imdOk : 1; // [high/low]
    bool ivtOCS : 1; // [high/low]
    bool satFaultInt : 1; // [high/low]
    bool hvAuxNeg : 1; // [high/low]
    bool hvAuxPos : 1; // [high/low]
    bool hvContNegMeas : 1; // [high/low]
    bool hvContPosMeas : 1; // [high/low]
    bool lvcWake : 1; // [high/low]
};

/*
 * BMS_OverridesAndContactors: Whether the overrides of the BMS are active and whether contactors are enabled
 */
struct BMS_OverridesAndContactors {
    static constexpr unsigned int ID = 202;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    bool hvilOverridden; // [T/F]
    bool imdOverridden; // [T/F]
    bool ivtOCSOverridden; // [T/F]
    bool enableNeg; // [T/F]
    bool enablePos; // [T/F]
    bool dischargeOff; // [T/F]
};

/*
 * BMS_Current: Battery current as given by the IVT
 */
struct BMS_Current {
    static constexpr unsigned int ID = 205;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    float current; // [A]
};

/*
 * BMS_BatteryPower: Battery power usage
 */
struct BMS_BatteryPower {
    static constexpr unsigned int ID = 207;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    float power; // [W]
};

/*
 * BMS_SatInterfaceState: State of the BMS sat board interface
 */
struct BMS_SatInterfaceState {
    static constexpr unsigned int ID = 208;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    BMSSatState state; // [state]
    BMSSatError error; // [error]
    uint8_t nSats; // [-]
    uint8_t satTimeoutCounter; // [-]
    bool satOK : 1; // [T/F]
    bool undervoltage : 1; // [T/F]
    bool overvoltage : 1; // [T/F]
    bool auxUndervoltage : 1; // [T/F]
    bool auxOvervoltage : 1; // [T/F]
    bool comparatorUndervoltage : 1; // [T/F]
    bool comparatorOvervoltage : 1; // [T/F]
    bool communicationFault : 1; // [T/F]
    bool systemFault : 1; // [T/F]
    bool chipFault : 1; // [T/F]
    bool gpiFault : 1; // [T/F]
};

/*
 * BMS_SatBalancingControl: Control parameters for the BMS sat interface related to balancing
 */
struct BMS_SatBalancingControl {
    static constexpr unsigned int ID = 211;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    uint16_t sat1BalancingChannels; // [bit a r r a y]
    uint16_t sat2BalancingChannels; // [bit a r r a y]
    uint16_t sat3BalancingChannels; // [bit a r r a y]
    uint16_t balancingThresholdVoltage; // [mV]
};

/*
 * BMS_SoCSoH: Battery State of Charge per %,  Battery State of Health per %
 */
struct BMS_SoCSoH {
    static constexpr unsigned int ID = 212;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    float soc; // [%]
    float soh; // [%]
};

/*
 * BMS_SatPCBTemperatures: Sat Board PCB Temperatures, used to discover hot pcb for instance when balancing
 */
struct BMS_SatPCBTemperatures {
    static constexpr unsigned int ID = 213;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    int16_t sat1pcbTemperature; // [c°C]
    int16_t sat2pcbTemperature; // [c°C]
    int16_t sat3pcbTemperature; // [c°C]
};

/*
 * BMS_Version: BMS version message
 */
struct BMS_Version {
    static constexpr unsigned int ID = 215;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    uint8_t swMajor; // [#]
    uint8_t swMinor; // [#]
    uint8_t eslibMajor; // [#]
    uint8_t eslibMinor; // [#]
    uint8_t cliMajor; // [#]
    uint8_t cliMinor; // [#]
};

/*
 * BMS_PCBTemperatures: PCB temperatures measured by the NTC's on the BMS
 */
struct BMS_PCBTemperatures {
    static constexpr unsigned int ID = 216;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    int16_t ntc1; // [c°C]
    int16_t ntc2; // [c°C]
    int16_t ntc3; // [c°C]
};

/*
 * BMS_HVMeasurements: Voltage measurements of the battery and the bus as received from the IVT
 */
struct BMS_HVMeasurements {
    static constexpr unsigned int ID = 217;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    float batteryVoltage; // [V]
    float busVoltage; // [V]
};

/*
 * BMS_IVTOCSettings: Settings of the IVT current thresholds
 */
struct BMS_IVTOCSettings {
    static constexpr unsigned int ID = 218;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    IVTLimitState state; // [state]
    IVTChargingLimitSetting chargingLimit; // [state]
    IVTDischargingLimitSetting dischargingLimit; // [state]
};

/*
 * ChC_Version: ChC version message
 */
struct ChC_Version {
    static constexpr unsigned int ID = 500;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    uint8_t swMajor; // [#]
    uint8_t swMinor; // [#]
    uint8_t eslibMajor; // [#]
    uint8_t eslibMinor; // [#]
    uint8_t cliMajor; // [#]
    uint8_t cliMinor; // [#]
};

/*
 * ChC_MainsState: State of the charge controller and possible error reasons
 */
struct ChC_MainsState {
    static constexpr unsigned int ID = 502;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    MainsChargingState state; // [state]
    ChargingError error; // [error r e a s o n]
    bool relayClosed : 1; // [T/F]
    bool relayStatus : 1; // [T/F]
    bool mosfetClosed : 1; // [T/F]
    bool lockActive : 1; // [T/F]
};

/*
 * ChC_BusMeasurement: Voltage and current measured at the bus side of the chargers
 */
struct ChC_BusMeasurement {
    static constexpr unsigned int ID = 509;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    float voltage; // [V]
    float current; // [A]
};

/*
 * ChC_InputCurrent: RMS current measured at the input of the charger
 */
struct ChC_InputCurrent {
    static constexpr unsigned int ID = 511;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    float current; // [A]
};

/*
 * ChC_InputVoltage: Voltage measured at the input of the charger
 */
struct ChC_InputVoltage {
    static constexpr unsigned int ID = 512;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    float voltage; // [V]
};

/*
 * ChC_ControllerSetting: Target current and limiting factor of the BMS
 */
struct ChC_ControllerSetting_STRUCT {
    static constexpr unsigned int ID = 517;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    float targetCurrent; // [A]
    ChCLimitingFactor limitingFactor; // [Factor]
    uint8_t eta; // [%]
    bool enabled : 1; // [T/F]
};

/*
 * ChC_ChargerOutputVoltages: Output voltages of the separate chargers connected to the ChC
 */
struct ChC_ChargerOutputVoltages {
    static constexpr unsigned int ID = 518;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    uint16_t ch1Voltage; // [dV]
    uint16_t ch2Voltage; // [dV]
    uint16_t ch3Voltage; // [dV]
};

/*
 * ChC_ChargerOutputCurrents: Output currents of the separate chargers connected to the ChC
 */
struct ChC_ChargerOutputCurrents {
    static constexpr unsigned int ID = 519;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    uint16_t ch1Current; // [dA]
    uint16_t ch2Current; // [dA]
    uint16_t ch3Current; // [dA]
};

/*
 * ChC_ProtocolState: State of the SAE J1772 protocol connection on the ChC
 */
struct ChC_ProtocolState {
    static constexpr unsigned int ID = 520;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    SAEJ1772State state; // [state]
    SAEJ1772Current current; // [current]
    bool hasProximity; // [T/F]
};

/*
 * ChC_ChargingControlState: State of the ChC charging control
 */
struct ChC_ChargingControlState {
    static constexpr unsigned int ID = 522;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    ChargingState state; // [state]
    DualChargerStrategySubState dualChargerSubState; // [state]
    TripleChargerStrategySubState tripleChargerSubState; // [state]
};

/*
 * ChC_ProtocolAmpacity: Ampacity according to the SAE J1772 protocol in case where the ampacity is equation-based
 */
struct ChC_ProtocolAmpacity {
    static constexpr unsigned int ID = 523;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    float ampacity; // [A]
};

/*
 * ChC_ChargerTemperatures: Charger temperatures
 */
struct ChC_ChargerTemperatures {
    static constexpr unsigned int ID = 524;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::ChC;

    int16_t ch1Temperature; // [ddegC]
    int16_t ch2Temperature; // [ddegC]
    int16_t ch3Temperature; // [ddegC]
};

struct Era_MagicString {
    static constexpr unsigned int ID = 2019;
    static constexpr bool IsCAN20B = false;
    static constexpr bool Empty = false;
    static constexpr Controller Sender = Controller::BMS;

    uint64_t magicString;
};

#pragma pack(pop)

}

#endif //AMICE_ERA_CANSPEC_H
