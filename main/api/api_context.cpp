#include "api_context.h"

namespace api {

Context::Context(const can::Store& can_store)
    : m_can_store{can_store} {}

const can::Store& Context::store() const {
    return m_can_store;
}

}