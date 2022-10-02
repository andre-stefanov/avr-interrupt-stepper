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

    static void step()
    {
        mock->step();
    }

    static void dir(bool value)
    {
        mock->dir(value);
    }
};

template<uint32_t T_SPR>
DriverMock* MockedDriver<T_SPR>::mock = nullptr;