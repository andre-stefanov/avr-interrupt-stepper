#pragma once

#include "TimerInterrupt.h"
#include "etl/delegate.h"

enum class Timer : int
{
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
};

template <Timer T>
struct TimerInterrupt_Delegate
{
    static etl::delegate<void()> init;
    static etl::delegate<void(uint32_t)> setInterval;
    static etl::delegate<void(timer_callback)> setCallback;
    static etl::delegate<void()> stop;
};

template <Timer T>
etl::delegate<void()> TimerInterrupt_Delegate<T>::init = etl::delegate<void()>();

template <Timer T>
etl::delegate<void(uint32_t)> TimerInterrupt_Delegate<T>::setInterval = etl::delegate<void(uint32_t)>();

template <Timer T>
etl::delegate<void(timer_callback)> TimerInterrupt_Delegate<T>::setCallback = etl::delegate<void(timer_callback)>();

template <Timer T>
etl::delegate<void()> TimerInterrupt_Delegate<T>::stop = etl::delegate<void()>();

template <Timer T>
const unsigned long int TimerInterrupt<T>::FREQ = F_CPU;

template <Timer T>
void TimerInterrupt<T>::init()
{
    if (TimerInterrupt_Delegate<T>::init.is_valid())
    {
        TimerInterrupt_Delegate<T>::init();
    }
}

template <Timer T>
void TimerInterrupt<T>::setInterval(uint32_t value)
{
    if (TimerInterrupt_Delegate<T>::setInterval.is_valid())
    {
        TimerInterrupt_Delegate<T>::setInterval(value);
    }
}

template <Timer T>
void TimerInterrupt<T>::setCallback(timer_callback fn)
{
    if (TimerInterrupt_Delegate<T>::setCallback.is_valid())
    {
        TimerInterrupt_Delegate<T>::setCallback(fn);
    }
}

template <Timer T>
void TimerInterrupt<T>::stop()
{
    if (TimerInterrupt_Delegate<T>::stop.is_valid())
    {
        TimerInterrupt_Delegate<T>::stop();
    }
}