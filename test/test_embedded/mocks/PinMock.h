#pragma once

#define PIN_CUSTOM_IMPL

#include "Pin.h"

#include <stdint.h>

template <uint8_t PIN>
struct PinMock
{
    using REAL_TYPE = Pin<PIN>;

    static bool initialized;
    static uint32_t toggle_count;
    static bool state;

    static void reset()
    {
        initialized = false;
        toggle_count = 0;
        state = false;
    }
};

template <uint8_t PIN>
bool PinMock<PIN>::initialized = false;

template <uint8_t PIN>
uint32_t PinMock<PIN>::toggle_count = 0;

template <uint8_t PIN>
bool PinMock<PIN>::state = false;

template <uint8_t PIN>
void Pin<PIN>::init()
{
    PinMock<PIN>::initialized = true;
}

template <uint8_t PIN>
void Pin<PIN>::pulse()
{
    high();
    low();
    PinMock<PIN>::toggle_count++;
}

template <uint8_t PIN>
void Pin<PIN>::high()
{
    PinMock<PIN>::state = true;
}

template <uint8_t PIN>
void Pin<PIN>::low()
{
    PinMock<PIN>::state = false;
}