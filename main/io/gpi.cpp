#include "gpi.h"

namespace io {

GPI::GPI(gpio_num_t pin_num)
    : m_pin{pin_num} {
    gpio_pad_select_gpio(pin_num);
    gpio_set_direction(pin_num, GPIO_MODE_INPUT);
}

GPI::operator bool() const {
    return static_cast<bool>(gpio_get_level(m_pin));
}

}