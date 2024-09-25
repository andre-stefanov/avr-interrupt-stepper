#pragma once

#include "gmock/gmock.h"
#include <iostream>

struct DriverMock
{
    MOCK_METHOD(void, step, ());
    MOCK_METHOD(void, dir, (bool));
};

template<uint32_t T_SPR>
struct MockedDriver
{
    constexpr static auto SPR = T_SPR;

    static DriverMock *mock;

    static bool direction;
    static int32_t position;

    static void step()
    {
        position += direction ? 1 : -1;
        mock->step();
    }

    static void dir(bool value)
    {
        direction = value;
        mock->dir(value);
    }
};

template<uint32_t T_SPR>
DriverMock* MockedDriver<T_SPR>::mock = nullptr;

template<uint32_t T_SPR>
bool MockedDriver<T_SPR>::direction = false;

template<uint32_t T_SPR>
int32_t MockedDriver<T_SPR>::position = 0;