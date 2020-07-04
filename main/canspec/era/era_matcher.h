#ifndef AMICE_ERA_MATCHER_H
#define AMICE_ERA_MATCHER_H

#include <can/can_store.h>

namespace era {

struct CANBusMatcher {
public:
    static bool Match(const can::Store& store);
};

}

#endif //AMICE_ERA_MATCHER_H
