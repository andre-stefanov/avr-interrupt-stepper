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

    void loopUntilStopped(uint32_t limit, bool expect_stopped = true) const
    {
        uint32_t loop = 0;
        while (loop++ < limit && callback != nullptr)
        {
            callback();
        }
        if (expect_stopped)
        {
            ASSERT_EQ(callback, nullptr);
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

    static void loopUntilStopped(uint32_t limit, bool expect_stopped = true)
    {
        mock->loopUntilStopped(limit, expect_stopped);
    } 
};

template <uint8_t ID>
IntervalInterruptMock *MockedIntervalInterrupt<ID>::mock = nullptr;