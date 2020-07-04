#ifndef AMICE_MUTEX_H
#define AMICE_MUTEX_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

namespace core {

class Mutex {
public:
    /**
     * Constructor
     */
    Mutex();

    /**
     * Locks the mutex
     * @param timeout_ms Lock timeout
     * @return Whether the lock was successful
     */
    bool Lock(uint32_t timeout_ms = 5000);

    /**
     * Unlocks the mutex
     */
    void Unlock();

private:
    SemaphoreHandle_t m_handle;
    StaticSemaphore_t m_semaphore;

};

}

#endif //AMICE_MUTEX_H
