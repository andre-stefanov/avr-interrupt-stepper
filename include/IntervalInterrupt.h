#ifndef INTERVAL_INTERRUPT_H
#define INTERVAL_INTERRUPT_H

#include <stdint.h> // NOLINT(modernize-deprecated-headers)

typedef void (*timer_callback)();

enum class Timer : int;

template <Timer T>
class IntervalInterrupt
{
public:
    IntervalInterrupt() = delete;

    constexpr static int ID = static_cast<int>(T);

    const static uint32_t FREQ;

    static void init();

    static void setCallback(timer_callback fn);

    static void setInterval(uint32_t value);

    static void stop();
};

#ifndef CUSTOM_TIMER_INTERRUPT_IMPL

#if defined(ARDUINO_ARCH_AVR)
#include "IntervalInterrupt_AVR.h"
#elif defined(ARDUINO_ARCH_STM32)
#include "IntervalInterrupt_STM32.h"
#else
#include "IntervalInterrupt_Delegate.h"
#endif

#endif // CUSTOM_TIMER_INTERRUPT_IMPL

#endif // INTERVAL_INTERRUPT_H