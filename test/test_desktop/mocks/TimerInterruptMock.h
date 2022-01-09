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

    static void run_until_stopped()
    {
        while (running)
        {
            callback();
        }
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
}

template <Timer T>
void TimerInterrupt<T>::stop()
{
    TimerInterruptMock<T>::running = false;
}