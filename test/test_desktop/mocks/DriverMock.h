#pragma once

#define DRIVER_CUSTOM_IMPL
#include "Driver.h"

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR, bool T_INVERT_DIR = false>
class DriverMock
{
private:
    DriverMock() = delete;

public:
    using REAL_TYPE = Driver<T_SPR, T_PIN_STEP, T_PIN_DIR>;

    static int32_t position;
    static bool cw;

    static void reset()
    {
        position = 0;
        cw = true;
    }

    static void step()
    {
        T_PIN_STEP::pulse();
        position += (cw) ? 1 : -1;
    }

    static void dir(bool value)
    {
        cw = value;
        if (value)
        {
            T_PIN_DIR::high();
        }
        else
        {
            T_PIN_DIR::low();
        }
    }
};

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR, bool T_INVERT_DIR>
int32_t DriverMock<T_SPR, T_PIN_STEP, T_PIN_DIR, T_INVERT_DIR>::position = 0;

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR, bool T_INVERT_DIR>
bool DriverMock<T_SPR, T_PIN_STEP, T_PIN_DIR, T_INVERT_DIR>::cw = true;

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR, bool T_INVERT_DIR>
void Driver<T_SPR, T_PIN_STEP, T_PIN_DIR, T_INVERT_DIR>::step()
{
    DriverMock<T_SPR, T_PIN_STEP, T_PIN_DIR>::step();
}

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR, bool T_INVERT_DIR>
void Driver<T_SPR, T_PIN_STEP, T_PIN_DIR, T_INVERT_DIR>::dir(bool cw)
{
    DriverMock<T_SPR, T_PIN_STEP, T_PIN_DIR>::dir(cw);
}