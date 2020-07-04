#include "utils.h"

extern "C" {
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
}

namespace core {

void WaitMs(uint32_t ms) {
    vTaskDelay(pdMS_TO_TICKS(ms));
}

}