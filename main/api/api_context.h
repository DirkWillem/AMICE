#ifndef AMICE_API_CONTEXT_H
#define AMICE_API_CONTEXT_H

#include <array>

#include <core/mutex.h>
#include <can/can_store.h>
#include <can/can_bus_status.h>

namespace api {

/**
 * API Context - contains link from API handlers to the rest of the application
 */
class Context {
public:
    /**
     * Constructor
     * @param bus_status CAN bus status monitor
     * @param can_store CAN message store
     */
    Context(const can::BusStatus& bus_status, const can::Store& can_store);

    /**
     * Returns the CAN store
     * @return CAN store
     */
    [[nodiscard]] const can::Store& store() const;

    /**
     * Returns the CAN bus status
     * @return CAN bus status
     */
    [[nodiscard]] const can::BusStatus& bus_status() const;
private:
    const can::BusStatus& m_bus_status;
    const can::Store& m_can_store;
};

}


#endif //AMICE_API_CONTEXT_H
