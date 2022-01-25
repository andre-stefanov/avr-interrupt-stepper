#pragma once

#define CUSTOM_TIMER_INTERRUPT_IMPL
#include "TimerInterrupt.h"

enum class Timer : int
{
    TIMER_TEST = 1
};

template <Timer T>
struct TimerInterruptMock
{
    using REAL_TYPE = TimerInterrupt<Timer::TIMER_TEST>;

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
bool TimerInterruptMock<T>::running = false;

template <Timer T>
bool TimerInterruptMock<T>::initialized = false;

template <Timer T>
timer_callback TimerInterruptMock<T>::callback = nullptr;

template <Timer T>
const unsigned long int TimerInterrupt<T>::FREQ = F_CPU;

template <Timer T>
uint32_t TimerInterruptMock<T>::max_interval = 0;

template <Timer T>
uint32_t TimerInterruptMock<T>::min_interval = UINT32_MAX;

template <Timer T>
void TimerInterrupt<T>::init()
{
    TimerInterruptMock<T>::initialized = true;
}

template <Timer T>
void TimerInterrupt<T>::setCallback(timer_callback fn)
{
    TimerInterruptMock<T>::callback = fn;
}

template <Timer T>
void TimerInterrupt<T>::setInterval(uint32_t value)
{
    TimerInterruptMock<T>::running = true;

    if (value > TimerInterruptMock<T>::max_interval)
    {
        TimerInterruptMock<T>::max_interval = value;
    }
    else if (value < TimerInterruptMock<T>::min_interval)
    {
        TimerInterruptMock<T>::min_interval = value;
    }
}

template <Timer T>
void TimerInterrupt<T>::stop()
{
    TimerInterruptMock<T>::running = false;
}