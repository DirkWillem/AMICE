#include "mutex.h"

namespace core {

Mutex::Mutex()
        : m_semaphore{} {
    m_handle = xSemaphoreCreateMutexStatic(&m_semaphore);
}

bool Mutex::Lock(uint32_t timeout_ms) {
    return xSemaphoreTake(m_handle, timeout_ms / portTICK_PERIOD_MS);
}

void Mutex::Unlock() {
    xSemaphoreGive(m_handle);
}

}
