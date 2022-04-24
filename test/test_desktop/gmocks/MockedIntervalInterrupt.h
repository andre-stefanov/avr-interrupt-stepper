#pragma once

#include "IntervalInterrupt.h"
#include "gmock/gmock.h"

struct IntervalInterruptMock
{
    timer_callback callback = nullptr;

    IntervalInterruptMock()
    {
        ON_CALL(*this, setCallback).WillByDefault([this](timer_callback cb)
                                                  { this->callback = cb; });
        ON_CALL(*this, stop).WillByDefault([this]()
                                           { this->callback = nullptr; });
    }

    MOCK_METHOD(void, init, ());
    MOCK_METHOD(void, setCallback, (timer_callback));
    MOCK_METHOD(void, setInterval, (uint32_t));
    MOCK_METHOD(void, stop, ());

    void loop(uint32_t limit)
    {
        uint32_t loop = 0;
        while (loop++ < limit)
        {
            callback();
        }
    }

    void loopUntilStopped(uint32_t limit)
    {
        uint32_t loop = 0;
        while (loop++ < limit && callback != nullptr)
        {
            callback();
        }
    }
};

template <uint8_t ID>
struct MockedIntervalInterrupt
{
    static IntervalInterruptMock *mock;

    constexpr static unsigned long int FREQ = F_CPU;

    static void init()
    {
        mock->init();
    }

    static void setCallback(timer_callback fn)
    {
        mock->setCallback(fn);
    }

    static void setInterval(uint32_t value)
    {
        mock->setInterval(value);
    }

    static void stop()
    {
        mock->stop();
    }

    static void loop(uint32_t limit)
    {
        mock->loop(limit);
    } 

    static void loopUntilStopped(uint32_t limit = 100000)
    {
        EXPECT_CALL(*mock, stop()).Times(1);
        mock->loopUntilStopped(limit);
    } 
};

template <uint8_t ID>
IntervalInterruptMock *MockedIntervalInterrupt<ID>::mock = nullptr;