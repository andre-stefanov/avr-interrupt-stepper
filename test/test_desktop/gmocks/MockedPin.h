#pragma once

#include "gmock/gmock.h"
#include <iostream>

struct PinMock
{
    MOCK_METHOD(void, init, ());
    MOCK_METHOD(void, pulse, ());
    MOCK_METHOD(void, high, ());
    MOCK_METHOD(void, low, ());
};

template <uint8_t T_PIN>
struct MockedPin
{
    constexpr static auto PIN = T_PIN;

    static PinMock *mock;

    static void init()
    {
        mock->init();
    }

    static void pulse()
    {
        mock->pulse();
    }

    static void high()
    {
        mock->high();
    }

    static void low()
    {
        mock->low();
    }

};

template <uint8_t T_PIN>
PinMock *MockedPin<T_PIN>::mock = nullptr;