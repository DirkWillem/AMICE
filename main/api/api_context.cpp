#include "api_context.h"

namespace api {

Context::Context(const can::BusStatus& bus_status, const can::Store& can_store)
    : m_bus_status{bus_status}, m_can_store{can_store} {}

const can::Store& Context::store() const {
    return m_can_store;
}

const can::BusStatus& Context::bus_status() const {
    return m_bus_status;
}

}