#pragma once

#include <stdint.h>

typedef void (*timer_callback)(void);

enum class Timer : int;

template <Timer T>
class IntervalInterrupt
{
private:
    IntervalInterrupt() = delete;

public:
    const static unsigned long int FREQ;

    static void init();

    static void setCallback(timer_callback fn);

    static void setInterval(uint32_t value);

    static void stop();
};

#ifndef CUSTOM_TIMER_INTERRUPT_IMPL

#if defined(ARDUINO_ARCH_AVR)
#include "IntervalInterrupt_AVR.h"
#else
#include "IntervalInterrupt_Delegate.h"
#endif

#endif // CUSTOM_TIMER_INTERRUPT_IMPL