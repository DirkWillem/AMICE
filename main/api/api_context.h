#ifndef AMICE_API_CONTEXT_H
#define AMICE_API_CONTEXT_H

#include <array>

#include <core/mutex.h>
#include <can/can_store.h>

namespace api {

class Context {
public:
    explicit Context(const can::Store& can_store);

    const can::Store& store() const;
private:
    const can::Store& m_can_store;
};

}


#endif //AMICE_API_CONTEXT_H
