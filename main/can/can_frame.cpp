#include "can_frame.h"

namespace can {

Frame::Frame()
        : timestamp{0}, empty{true}, pad{0}, len{0}, data{0} {}

}
