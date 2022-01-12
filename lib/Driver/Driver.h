#pragma once

#include <stdint.h>

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR>
class Driver
{
private:
    Driver() = delete;

public:
    constexpr static auto SPR = T_SPR;

    static inline __attribute__((always_inline)) void step();

    static inline __attribute__((always_inline)) void dir(bool cw);
};

#ifndef DRIVER_CUSTOM_IMPL

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR>
inline __attribute__((always_inline)) void Driver<T_SPR, T_PIN_STEP, T_PIN_DIR>::step()
{
    T_PIN_STEP::pulse();
}

template <uint32_t T_SPR, typename T_PIN_STEP, typename T_PIN_DIR>
inline __attribute__((always_inline)) void Driver<T_SPR, T_PIN_STEP, T_PIN_DIR>::dir(bool cw)
{
    if (cw)
    {
        T_PIN_DIR::high();
    }
    else
    {
        T_PIN_DIR::low();
    }
}

#endif