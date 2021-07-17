#ifndef AMICE_CONFIG_H
#define AMICE_CONFIG_H

#include <io/gpo.h>

//#define CONFIG_ESP32
#define CONFIG_AMICE

#ifdef CONFIG_ESP32

namespace pinout {
    static constexpr auto LED1R = GPIO_NUM_NC;
    static constexpr auto LED1G = GPIO_NUM_NC;
    static constexpr auto LED1B = GPIO_NUM_2;

    static constexpr auto LED2R = GPIO_NUM_NC;
    static constexpr auto LED2G = GPIO_NUM_NC;
    static constexpr auto LED2B = GPIO_NUM_NC;
}

#endif

#ifdef CONFIG_AMICE

namespace pinout {
    static constexpr auto LED1R = GPIO_NUM_19;
    static constexpr auto LED1G = GPIO_NUM_18;
    static constexpr auto LED1B = GPIO_NUM_17;

    static constexpr auto LED2R = GPIO_NUM_25;
    static constexpr auto LED2G = GPIO_NUM_27;
    static constexpr auto LED2B = GPIO_NUM_26;

    static constexpr auto BTN_L = GPIO_NUM_16;
    static constexpr auto BTN_R = GPIO_NUM_5;
}

#endif

#endif //AMICE_CONFIG_H
