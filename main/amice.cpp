#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include <cmath>
#include <api/api_server.h>
#include <application/can_reader_task.h>
#include <application/can_store_task.h>
#include <application/status_monitor_task.h>
#include <can/can_buffer.h>
#include <can/can_store.h>
#include <driver/gpio.h>
#include <wifi/station.h>
#include <core/utils.h>
#include <can/slcan_interface.h>
#include <wifi/access_point.h>
#include <array>

#include <driver/i2c.h>

#include "oled_main.h"

class SOCTask : public core::Task<> {
public:
    SOCTask(core::Timer& timer, can::Store& store
      //, can::SlCANInterface& ifc
    ): Task<>{"SOC"}, timer{timer}, store{store}//, ifc{ifc}
    {}

    void Run() final {
        era::BMS_SoCSoH socsoh{};
        can::Frame soc_frame{};
        soc_frame.id = era::BMS_SoCSoH::ID;
        soc_frame.len = 8;

        era::SSB_Lights lights{};
        can::Frame lights_frame{};
        lights_frame.id = era::SSB_Lights::ID;
        lights_frame.len = 2;

        socsoh.soc = 80.f;
        socsoh.soh = 97.8f;

        std::array<uint16_t, 4> cellvs{};
        cellvs[0] = htons(3421);
        cellvs[1] = htons(3429);
        cellvs[2] = htons(3418);
        cellvs[3] = htons(3425);

        can::Frame cellvs_frame;
        cellvs_frame.id = 1080;
        cellvs_frame.data = *reinterpret_cast<const uint64_t*>(cellvs.data());
        cellvs_frame.len = 8;
        store.Put(cellvs_frame);

      era::BMS_State state{};
      state.mainState = era::BMSState::Active;
      state.imdState = era::IMDState::Operational;
      state.coolingLimitState = era::BMSCoolingLimit::Charging;
      state.battfrontState = era::OnOff::On;
      state.coolingControlState = era::OnOff::Off;
      state.canControllerState = era::OnOff::On;
      state.error = era::BMSError::IVTOCS;
      can::Frame state_frame{};

      *reinterpret_cast<era::BMS_State*>(&state_frame.data) = state;
      state_frame.id = era::BMS_State::ID;
      state_frame.len = 8;
      store.Put(state_frame);

        std::array<uint8_t, 8> cellts{};
        cellts[0] = 32;
        cellts[1] = 33;
        cellts[2] = 33;
        cellts[3] = 32;
        cellts[4] = 33;
        cellts[5] = 34;

        can::Frame cellts_frame;
        cellts_frame.id = 1089;
        cellts_frame.data = *reinterpret_cast<const uint64_t*>(cellts.data());
        cellts_frame.len = 6;
        store.Put(cellts_frame);

        uint32_t t0 = 0;
        while(true) {
            socsoh.soc = 80.f + 10*sinf(timer.millis() / 1000.f);
            *reinterpret_cast<era::BMS_SoCSoH*>(&soc_frame.data) = socsoh;

            store.Put(soc_frame);

            auto t = timer.millis();

            if (t - t0 >= 6000) {
                t0 = t;
            }

            if (t - t0 >= 4000) {
                lights.lights = era::LightingState::HB;
                state.mainState = era::BMSState::ESS;
            } else if (t - t0 >= 2000) {
                state.mainState = era::BMSState::Active;
                lights.lights = era::LightingState::LB;
            } else {
              state.mainState = era::BMSState::Active;
                lights.lights = era::LightingState::Off;
            }

            *reinterpret_cast<era::SSB_Lights*>(&lights_frame.data) = lights;
            store.Put(lights_frame);

            *reinterpret_cast<era::BMS_State*>(&state_frame.data) = state;
            store.Put(state_frame);

            can_message_t msg{};
            msg.identifier = 0x123;
            msg.data_length_code = 3;
            msg.data[0] = 0x12;
            msg.data[1] = 0x34;
            msg.data[2] = 0xBA;
            //ifc.SendFrame(msg);

            core::WaitMs(200);
        }
    }

private:
    core::Timer& timer;
    can::Store& store;
    //can::SlCANInterface& ifc;
};

class CANReadTask : public core::Task<> {
public:
    explicit CANReadTask(can::SlCANInterface& ifc) : core::Task<>{"Read"}, m_ifc{ifc} {}

    void Run() final {
        while (true) {
            m_ifc.ProcessCommand();
        }
    }

private:
    can::SlCANInterface& m_ifc;
};

void InstallDrivers() {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI("System", "NVS flash initialized");

    // Initialize TCP/IP adapter
    auto err = esp_netif_init();
    if (err == ESP_OK) {
        ESP_LOGI("System", "Network interface initialized");
    } else {
        ESP_LOGE("System", "Failed to initialize network interface (%d)", err);
        return;
    }

    //Initialize configuration structures using macro initializers
    can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(GPIO_NUM_21, GPIO_NUM_22, CAN_MODE_NORMAL);
    can_timing_config_t t_config = CAN_TIMING_CONFIG_500KBITS();
    can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

    //Install CAN driver
    err = can_driver_install(&g_config, &t_config, &f_config);
    if (err == ESP_OK) {
        ESP_LOGI("System", "CAN driver installed");
    } else {
        ESP_LOGE("System", "Failed to install CAN driver (%d)", err);
        return;
    }

    //Start CAN driver
    err = can_start();
    if (err == ESP_OK) {
        ESP_LOGI("System", "CAN driver started");
    } else {
        ESP_LOGE("System", "Failed to start CAN driver (%d)", err);
        return;
    }

   /* // Initialize I2C
    int i2c_master_port = I2C_NUM_0;
    i2c_config_t i2c_config{
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 33,
        .scl_io_num = 32,
        .sda_pullup_en = true,
        .scl_pullup_en = true,
    };
    i2c_config.master.clk_speed = 100'000;

    err = i2c_param_config(i2c_master_port, &i2c_config);
    if (err == ESP_OK) {
      ESP_LOGI("System", "I2C driver configured");
    } else {
      ESP_LOGE("System", "Failed to configure I2C driver (%d)", err);
      return;
    }

    err = i2c_driver_install(i2c_master_port, i2c_config.mode, 0, 0, 0);
    if (err == ESP_OK) {
      ESP_LOGI("System", "I2C driver configured");
    } else {
      ESP_LOGE("System", "Failed to configure I2C driver (%d)", err);
      return;
    }*/
}

void PutMockData(can::Store& store) {
    // Put mock VCellMin, VCellmax
    era::BMS_MinMaxCellVoltage cv{};
    cv.minCellVoltage = htons(3301);
    cv.maxCellVoltage = htons(3347);
    cv.minCellNumber = 7;
    cv.maxCellNumber = 201;

    can::Frame frame{};
    *reinterpret_cast<era::BMS_MinMaxCellVoltage*>(&frame.data) = cv;
    frame.id = era::BMS_MinMaxCellVoltage::ID;
    frame.len = 8;
    frame.empty = false;
    frame.timestamp = 123;

    store.Put(frame);

    era::BMS_SoCSoH socsoh{};
    socsoh.soc = 66.1f;
    socsoh.soh = 97.8f;

    *reinterpret_cast<era::BMS_SoCSoH*>(&frame.data) = socsoh;
    frame.id = era::BMS_SoCSoH::ID;
    frame.len = 8;
    store.Put(frame);

    era::BMS_MinMaxCellTemp ts{};
    ts.minCellTemperature = htons(3210);
    ts.maxCellTemperature = htons(3480);
    ts.minNTCNumber = 1;
    ts.maxNTCNumber = 208;

  *reinterpret_cast<era::BMS_MinMaxCellTemp*>(&frame.data) = ts;
  frame.id = era::BMS_MinMaxCellTemp::ID;
  frame.len = 6;
  store.Put(frame);

    era::BMS_BatteryPower batt_power{};
    batt_power.power = -3201;
    *reinterpret_cast<era::BMS_BatteryPower*>(&frame.data) = batt_power;
    frame.id = era::BMS_BatteryPower::ID;
    frame.len = 4;
    store.Put(frame);

    era::BMS_State state{};
    state.mainState = era::BMSState::Active;
    state.imdState = era::IMDState::Operational;
    state.coolingLimitState = era::BMSCoolingLimit::Charging;
    state.battfrontState = era::OnOff::On;
    state.coolingControlState = era::OnOff::Off;
    state.canControllerState = era::OnOff::On;
    state.error = era::BMSError::IVTOCS;

    *reinterpret_cast<era::BMS_State*>(&frame.data) = state;
    frame.id = era::BMS_State::ID;
    frame.len = 8;
    store.Put(frame);

    era::LVC_VehicleState vs{};
    vs.state = era::VehicleState::Drive;
    vs.lvcState = era::LVCState::DoubleConv;

    *reinterpret_cast<era::LVC_VehicleState*>(&frame.data) = vs;
    frame.id = era::LVC_VehicleState::ID;
    frame.len = 2;
    store.Put(frame);


    era::SSB_Lights lights{};
    lights.lights = era::LightingState::HB;

    *reinterpret_cast<era::SSB_Lights*>(&frame.data) = lights;
    frame.id = era::SSB_Lights::ID;
    frame.len = 2;
    store.Put(frame);

    era::ChC_MainsState chc;
    chc.state = era::MainsChargingState::Charging;
    chc.error = era::ChargingError::None;

    *reinterpret_cast<era::ChC_MainsState*>(&frame.data) = chc;
    frame.id = era::ChC_MainsState::ID;
    frame.len = 3;
    store.Put(frame);
}

extern "C" void app_main() {
    // Install drivers
    InstallDrivers();

    // Create default event loop
    //ESP_ERROR_CHECK(esp_event_loop_create_default());

    /*// General stuff
    static core::Timer timer0{TIMER_GROUP_0, TIMER_0};

    // Event groups
    static CANEventGroup can_event_group{};

    // CAN store and buffers
    static can::Store can_store{};
    static can::Buffer can_buf_a{};
    static can::Buffer can_buf_b{};
    static can::BusStatus can_bus_status{};

    //static io::UART uart0{GPIO_NUM_1, GPIO_NUM_3, UART_NUM_0, 115200};
    //static can::SlCANInterface slcan_ifc{uart0, timer0};
    //static CANReadTask can_read_task{slcan_ifc};

    //PutMockData(can_store);

    // Global tasks
    static StatusMonitorTask status_task{can_store, timer0, can_bus_status};

    // CAN tasks
    //static CANReaderTask can_reader_task{can_buf_a, can_buf_b, timer0, can_event_group};
    //static CANStoreTask can_store_task{can_buf_a, can_buf_b, can_store, can_event_group};

    //static SOCTask soc_task{timer0, can_store};

    ESP_LOGI("System", "AMICE 1.0 - Starting...");

    // Set up WiFi in Station mode
    static wifi::AccessPoint ap{"AMICE", "PruttelMobiel", 5};
    //static wifi::Station station{"Pretzel2", "K0rt3Krk6403DD"};
   //static wifi::Station station{"Ziggo58760", "pAK4^TbA7aWW"};

    //static wifi::Station station{"Ziggo5458078", "fnr4kkvxXjh4"};
    // Set up API server
    static api::Context api_context{can_bus_status, can_store};
    static api::ApiServer server{api_context};*/

    OLEDMain();
}
