#pragma once

#include "stdint-gcc.h"
#include "FastPin.h"

template <uint8_t PIN_STEP, uint8_t PIN_DIR>
class Driver
{
private:
    Driver() = delete;

public:
    static inline __attribute__((always_inline)) void step()
    {
        FastPin<PIN_STEP>::pulse();
    }

    static inline __attribute__((always_inline)) void dir(bool value)
    {
        if (value)
        {
            FastPin<PIN_DIR>::high();
        }
        else
        {
            FastPin<PIN_DIR>::low();
        }
    }
};
