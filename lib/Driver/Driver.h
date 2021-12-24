#pragma once

#include "stdint-gcc.h"

template <typename PIN_STEP, typename PIN_DIR>
class Driver
{
private:
    Driver() = delete;

public:

    constexpr static auto SPR = 400 * 64;

    static inline __attribute__((always_inline)) void step()
    {
        PIN_STEP::pulse();
    }

    static inline __attribute__((always_inline)) void dir(bool value)
    {
        if (value)
        {
            PIN_DIR::high();
        }
        else
        {
            PIN_DIR::low();
        }
    }
};
