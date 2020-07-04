#ifndef AMICE_TASK_H
#define AMICE_TASK_H

#include <array>
#include <cstdint>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

namespace core {

/**
 * Virtual base class for all tasks.
 */
class TaskBase {
public:
    /**
     * Destructor
     */
    virtual ~TaskBase() = default;

    /**
     * Run method, should be implemented by task implementations
     */
    [[noreturn]] virtual void Run() = 0;

    /**
     * Static run function, passed to FreeRTOS
     * @param ptr Task user data
     */
    static void _StaticRun(void* ptr);
};

enum class Core {
    Core0,
    Core1
};

template<size_t StackSize = 4096>
/**
 * Base classs for all tasks
 * @tparam StackSize Task stack size
 */
class Task
        : public TaskBase {
public:

    explicit Task(const char* name)
            : m_stack{}, m_task{} {
        m_handle = xTaskCreateStatic(&TaskBase::_StaticRun,
                                     name, StackSize, this, tskIDLE_PRIORITY, m_stack.data(), &m_task);
    }

    explicit Task(const char* name, Core core)
        : m_stack{}, m_task{} {
        m_handle = xTaskCreateStaticPinnedToCore(&TaskBase::_StaticRun,
                                     name, StackSize, this, tskIDLE_PRIORITY, m_stack.data(), &m_task,
                                     core == Core::Core0 ? 0 : 1);
    }

    ~Task() override = default;

private:
    std::array<StackType_t, StackSize> m_stack{};
    StaticTask_t m_task;
    TaskHandle_t m_handle;
};

}

#endif //AMICE_TASK_H
