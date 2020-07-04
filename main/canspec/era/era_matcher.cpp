#include "era_matcher.h"

#include <cstring>

#include "era_canspec.h"

namespace era {

bool CANBusMatcher::Match(const can::Store& store) {
    auto magic_str = store.frame(Era_MagicString::ID);
    if (magic_str) {
        const char* stella19 = "STELLA19";
        return (*magic_str).data == *reinterpret_cast<const uint64_t*>(stella19);
    }

    return false;
}

}