#pragma once

#include "TimerInterrupt.h"

enum class Timer : int
{
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
};

template <Timer T>
const unsigned long int TimerInterrupt<T>::FREQ = F_CPU;

template<Timer T>
void TimerInterrupt<T>::init()
{
    
}

template<Timer T>
void TimerInterrupt<T>::setInterval(uint32_t value)
{
    
}

template<Timer T>
void TimerInterrupt<T>::stop()
{
    
}