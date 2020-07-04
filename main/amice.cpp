#include <esp_log.h>
#include <esp_event.h>
#include <nvs_flash.h>

#include <api/api_server.h>
#include <application/can_reader_task.h>
#include <application/can_store_task.h>
#include <application/status_indicator_task.h>
#include <can/can_buffer.h>
#include <can/can_store.h>
#include <driver/gpio.h>
#include <wifi/access_point.h>

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

    static StatusIndicatorTask status_task{};

    // CAN tasks
    static CANReaderTask can_reader_task{can_buf_a, can_buf_b, timer0, can_event_group};
    static CANStoreTask can_store_task{can_buf_a, can_buf_b, can_store, can_event_group};

    ESP_LOGI("System", "AMICE 1.0 - Starting...");

    // Set up WiFi in Station mode
    static wifi::AccessPoint ap{"AMICE", "PruttelMobiel", 5};

    // Set up API server
    static api::Context api_context{can_store};
    static api::ApiServer server{api_context};
}
