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
#include <wifi/access_point.h>
#include <wifi/station.h>
#include <core/utils.h>

class SOCTask : public core::Task<> {
public:
    SOCTask(core::Timer& timer, can::Store& store): Task<>{"SOC"}, timer{timer}, store{store} {}

    void Run() final {
        era::BMS_SoCSoH socsoh{};
        can::CANFrame soc_frame{};
        soc_frame.id = era::BMS_SoCSoH::ID;
        soc_frame.len = 8;

        era::SSB_Lights lights{};
        can::CANFrame lights_frame{};
        lights_frame.id = era::SSB_Lights::ID;
        lights_frame.len = 2;

        socsoh.soc = 80.f;
        socsoh.soh = 97.8f;

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
            } else if (t - t0 >= 2000) {
                lights.lights = era::LightingState::LB;
            } else {
                lights.lights = era::LightingState::Off;
            }

            *reinterpret_cast<era::SSB_Lights*>(&lights_frame.data) = lights;
            store.Put(lights_frame);

            core::WaitMs(200);
        }
    }

private:
    core::Timer& timer;
    can::Store& store;
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
}

void PutMockData(can::Store& store) {
    // Put mock VCellMin, VCellmax
    era::BMS_MinMaxCellVoltage cv{};
    cv.minCellVoltage = 3301;
    cv.maxCellVoltage = 3347;
    cv.minCellNumber = 7;
    cv.maxCellNumber = 201;

    can::CANFrame frame{};
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

    era::SSB_Lights lights{};
    lights.lights = era::LightingState::HB;

    *reinterpret_cast<era::SSB_Lights*>(&frame.data) = lights;
    frame.id = era::SSB_Lights::ID;
    frame.len = 2;
    store.Put(frame);
}

extern "C" void app_main() {
    // Install drivers
    InstallDrivers();

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // General stuff
    static core::Timer timer0{TIMER_GROUP_0, TIMER_0};

    // Event groups
    static CANEventGroup can_event_group{};

    // CAN store and buffers
    static can::Store can_store{};
    static can::Buffer can_buf_a{};
    static can::Buffer can_buf_b{};
    static can::BusStatus can_bus_status{};

    PutMockData(can_store);

    // Global tasks
    static StatusMonitorTask status_task{can_store, timer0, can_bus_status};

    // CAN tasks
    static CANReaderTask can_reader_task{can_buf_a, can_buf_b, timer0, can_event_group};
    static CANStoreTask can_store_task{can_buf_a, can_buf_b, can_store, can_event_group};

    static SOCTask soc_task{timer0, can_store};

    ESP_LOGI("System", "AMICE 1.0 - Starting...");

    // Set up WiFi in Station mode
    //static wifi::AccessPoint ap{"AMICE", "PruttelMobiel", 5};
    // static wifi::Station station{"Pretzel2", "K0rt3Krk6403DD"};
    static wifi::Station station{"Ziggo58760", "pAK4^TbA7aWW"};
    // Set up API server
    static api::Context api_context{can_bus_status, can_store};
    static api::ApiServer server{api_context};
}
