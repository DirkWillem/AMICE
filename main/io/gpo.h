#ifndef AMICE_GPO_H
#define AMICE_GPO_H

#include <driver/gpio.h>

namespace io {

/**
 * Represents a general purpose output
 */
class GPO {
public:
    /**
     * Constructor
     * @param pin_num Pin number
     */
    explicit GPO(gpio_num_t pin_num);

    /**
     * Assigns a binary value to the pin
     * @param val Value to assign
     * @return Reference to gpo
     */
    GPO& operator=(bool val);

private:
    gpio_num_t m_pin;
};

}

#endif //AMICE_GPO_H
