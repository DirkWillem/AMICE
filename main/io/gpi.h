#ifndef AMICE_GPI_H
#define AMICE_GPI_H

#include <driver/gpio.h>

namespace io {

class GPI {
public:
    explicit GPI(gpio_num_t pin_num);

    operator bool() const;

private:
    gpio_num_t m_pin;
};

}

#endif //AMICE_GPI_H
