#pragma once

#define CUSTOM_TIMER_INTERRUPT_IMPL
#include "IntervalInterrupt.h"

enum class Timer : int
{
    TIMER_TEST = 1
};

template <Timer T>
struct IntervalInterruptMock
{
    using REAL_TYPE = IntervalInterrupt<Timer::TIMER_TEST>;

    static bool initialized;
    static bool running;
    static timer_callback callback;

    static uint32_t max_interval;
    static uint32_t min_interval;

    static void run_until_stopped(uint32_t limit_steps = UINT32_MAX)
    {
        uint32_t steps_left = limit_steps;
        while (running && steps_left--)
        {
            callback();
        }
    }

    static void reset()
    {
        initialized = false;
        running = false;
        callback = nullptr;
        max_interval = 0;
        min_interval = UINT32_MAX;
    }
};

template <Timer T>
bool IntervalInterruptMock<T>::running = false;

template <Timer T>
bool IntervalInterruptMock<T>::initialized = false;

template <Timer T>
timer_callback IntervalInterruptMock<T>::callback = nullptr;

template <Timer T>
const unsigned long int IntervalInterrupt<T>::FREQ = F_CPU;

template <Timer T>
uint32_t IntervalInterruptMock<T>::max_interval = 0;

template <Timer T>
uint32_t IntervalInterruptMock<T>::min_interval = UINT32_MAX;

template <Timer T>
void IntervalInterrupt<T>::init()
{
    IntervalInterruptMock<T>::initialized = true;
}

template <Timer T>
void IntervalInterrupt<T>::setCallback(timer_callback fn)
{
    IntervalInterruptMock<T>::callback = fn;
}

template <Timer T>
void IntervalInterrupt<T>::setInterval(uint32_t value)
{
    IntervalInterruptMock<T>::running = true;

    if (value > IntervalInterruptMock<T>::max_interval)
    {
        IntervalInterruptMock<T>::max_interval = value;
    }
    else if (value < IntervalInterruptMock<T>::min_interval)
    {
        IntervalInterruptMock<T>::min_interval = value;
    }
}

template <Timer T>
void IntervalInterrupt<T>::stop()
{
    IntervalInterruptMock<T>::running = false;
}