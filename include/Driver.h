#pragma once

#include <stdint.h>

template <typename T_PIN_STEP, typename T_PIN_DIR>
class Driver
{
private:
    static bool isInverted;

public:
    Driver() = delete;

    static void init()
    {
        T_PIN_STEP::init();
        T_PIN_DIR::init();
    }

    static inline void __attribute__((always_inline)) setInverted(bool value);

    static inline __attribute__((always_inline)) void step();

    static inline __attribute__((always_inline)) void dir(bool cw);
};

#ifndef DRIVER_CUSTOM_IMPL

template <typename T_PIN_STEP, typename T_PIN_DIR>
inline __attribute__((always_inline)) void Driver<T_PIN_STEP, T_PIN_DIR>::setInverted(bool value)
{
    isInverted = value;
}

template <typename T_PIN_STEP, typename T_PIN_DIR>
inline __attribute__((always_inline)) void Driver<T_PIN_STEP, T_PIN_DIR>::step()
{
    T_PIN_STEP::pulse();
}

template <typename T_PIN_STEP, typename T_PIN_DIR>
inline __attribute__((always_inline)) void Driver<T_PIN_STEP, T_PIN_DIR>::dir(bool cw)
{
    if (cw || isInverted)
    {
        T_PIN_DIR::low();
    }
    else
    {
        T_PIN_DIR::high();
    }
}

#endif