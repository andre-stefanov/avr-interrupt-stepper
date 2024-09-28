#pragma once

#ifdef ARDUINO_ARCH_STM32

#include <IntervalInterrupt.h>
#include <stdint.h>
#include "HardwareTimer.h"

#define DEBUG_INTERRUPT_TIMING_PIN 0
#if DEBUG_INTERRUPT_TIMING_PIN != 0 && UNIT_TEST != 1
#include <Pin.h>
#define INTERRUPT_TIMING_START() Pin<DEBUG_INTERRUPT_TIMING_PIN>::high()
#define INTERRUPT_TIMING_END() Pin<DEBUG_INTERRUPT_TIMING_PIN>::low()
#else
#define INTERRUPT_TIMING_START()
#define INTERRUPT_TIMING_END()
#endif

#define DEBUG_INTERRUPT_SET_INTERVAL_PIN 0
#if DEBUG_INTERRUPT_SET_INTERVAL_PIN != 0 && UNIT_TEST != 1
#include <Pin.h>
#define SET_INTERVAL_TIMING_START() Pin<DEBUG_INTERRUPT_SET_INTERVAL_PIN>::high()
#define SET_INTERVAL_TIMING_END() Pin<DEBUG_INTERRUPT_SET_INTERVAL_PIN>::low()
#else
#define SET_INTERVAL_TIMING_START()
#define SET_INTERVAL_TIMING_END()
#endif


/**
 * @brief Enumeration of 16 bit timers on atmega2560
 */
enum class Timer : int
{
#if defined(STM32F0)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_14 = 14,
    TIMER_15 = 15,
    TIMER_16 = 16,
    TIMER_17 = 17
#elif defined(STM32F1)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_9 = 9,
    TIMER_10 = 10,
    TIMER_11 = 11,
    TIMER_12 = 12,
    TIMER_13 = 13,
    TIMER_14 = 14
#elif defined(STM32F2)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_9 = 9,
    TIMER_10 = 10,
    TIMER_11 = 11,
    TIMER_12 = 12,
    TIMER_13 = 13,
    TIMER_14 = 14
#elif defined(STM32F3)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_15 = 15,
    TIMER_16 = 16,
    TIMER_17 = 17,
    TIMER_20 = 20
#elif defined(STM32F4)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_9 = 9,
    TIMER_10 = 10,
    TIMER_11 = 11,
    TIMER_12 = 12,
    TIMER_13 = 13,
    TIMER_14 = 14
#elif defined(STM32F7)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_9 = 9,
    TIMER_10 = 10,
    TIMER_11 = 11,
    TIMER_12 = 12,
    TIMER_13 = 13,
    TIMER_14 = 14
#elif defined(STM32L0)
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_21 = 21,
    TIMER_22 = 22
#elif defined(STM32L1)
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_9 = 9,
    TIMER_10 = 10,
    TIMER_11 = 11
#elif defined(STM32L4)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_15 = 15,
    TIMER_16 = 16,
    TIMER_17 = 17
#elif defined(STM32H7)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_9 = 9,
    TIMER_10 = 10,
    TIMER_11 = 11,
    TIMER_12 = 12,
    TIMER_13 = 13,
    TIMER_14 = 14
#elif defined(STM32G0)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_14 = 14,
    TIMER_15 = 15,
    TIMER_16 = 16,
    TIMER_17 = 17
#elif defined(STM32G4)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_15 = 15,
    TIMER_16 = 16,
    TIMER_17 = 17,
    TIMER_20 = 20
#elif defined(STM32WB)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_16 = 16,
    TIMER_17 = 17
#elif defined(STM32MP1)
    TIMER_1 = 1,
    TIMER_2 = 2,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
    TIMER_6 = 6,
    TIMER_7 = 7,
    TIMER_8 = 8,
    TIMER_12 = 12,
    TIMER_15 = 15
#endif
};

constexpr TIM_TypeDef *timer_def_from_enum(const Timer timer)
{
    switch (timer)
    {
#if defined(STM32F0)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_14:
        return TIM14;
    case Timer::TIMER_15:
        return TIM15;
    case Timer::TIMER_16:
        return TIM16;
    case Timer::TIMER_17:
        return TIM17;
#elif defined(STM32F1)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_9:
        return TIM9;
    case Timer::TIMER_10:
        return TIM10;
    case Timer::TIMER_11:
        return TIM11;
    case Timer::TIMER_12:
        return TIM12;
    case Timer::TIMER_13:
        return TIM13;
    case Timer::TIMER_14:
        return TIM14;
#elif defined(STM32F2)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_9:
        return TIM9;
    case Timer::TIMER_10:
        return TIM10;
    case Timer::TIMER_11:
        return TIM11;
    case Timer::TIMER_12:
        return TIM12;
    case Timer::TIMER_13:
        return TIM13;
    case Timer::TIMER_14:
        return TIM14;
#elif defined(STM32F3)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_15:
        return TIM15;
    case Timer::TIMER_16:
        return TIM16;
    case Timer::TIMER_17:
        return TIM17;
    case Timer::TIMER_20:
        return TIM20;
#elif defined(STM32F4)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_9:
        return TIM9;
    case Timer::TIMER_10:
        return TIM10;
    case Timer::TIMER_11:
        return TIM11;
    case Timer::TIMER_12:
        return TIM12;
    case Timer::TIMER_13:
        return TIM13;
    case Timer::TIMER_14:
        return TIM14;
#elif defined(STM32F7)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_9:
        return TIM9;
    case Timer::TIMER_10:
        return TIM10;
    case Timer::TIMER_11:
        return TIM11;
    case Timer::TIMER_12:
        return TIM12;
    case Timer::TIMER_13:
        return TIM13;
    case Timer::TIMER_14:
        return TIM14;
#elif defined(STM32L0)
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_21:
        return TIM21;
    case Timer::TIMER_22:
        return TIM22;
#elif defined(STM32L1)
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_9:
        return TIM9;
    case Timer::TIMER_10:
        return TIM10;
    case Timer::TIMER_11:
        return TIM11;
#elif defined(STM32L4)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_15:
        return TIM15;
    case Timer::TIMER_16:
        return TIM16;
    case Timer::TIMER_17:
        return TIM17;
#elif defined(STM32H7)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_9:
        return TIM9;
    case Timer::TIMER_10:
        return TIM10;
    case Timer::TIMER_11:
        return TIM11;
    case Timer::TIMER_12:
        return TIM12;
    case Timer::TIMER_13:
        return TIM13;
    case Timer::TIMER_14:
        return TIM14;
#elif defined(STM32G0)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_14:
        return TIM14;
    case Timer::TIMER_15:
        return TIM15;
    case Timer::TIMER_16:
        return TIM16;
    case Timer::TIMER_17:
        return TIM17;
#elif defined(STM32G4)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_15:
        return TIM15;
    case Timer::TIMER_16:
        return TIM16;
    case Timer::TIMER_17:
        return TIM17;
    case Timer::TIMER_20:
        return TIM20;
#elif defined(STM32WB)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_16:
        return TIM16;
    case Timer::TIMER_17:
        return TIM17;
#elif defined(STM32MP1)
    case Timer::TIMER_1:
        return TIM1;
    case Timer::TIMER_2:
        return TIM2;
    case Timer::TIMER_3:
        return TIM3;
    case Timer::TIMER_4:
        return TIM4;
    case Timer::TIMER_5:
        return TIM5;
    case Timer::TIMER_6:
        return TIM6;
    case Timer::TIMER_7:
        return TIM7;
    case Timer::TIMER_8:
        return TIM8;
    case Timer::TIMER_12:
        return TIM12;
    case Timer::TIMER_15:
        return TIM15;
#endif
    default:
        return nullptr;
    }
}

template <Timer T>
struct IntervalInterrupt_STM32
{
    static HardwareTimer timer;

    static volatile uint16_t ovf_cnt;
    static volatile uint16_t ovf_left;
    static volatile uint16_t cmp_cnt;

    static volatile timer_callback callback;

    static inline __attribute__((always_inline)) void init()
    {
        timer.setPreloadEnable(false);
        timer.setPrescaleFactor(1);
    }

    static inline __attribute__((always_inline)) void set_interval(uint32_t value)
    {
        SET_INTERVAL_TIMING_START();

        ovf_cnt = (value >> 16);

        if (ovf_cnt)
        {
            ovf_left = ovf_cnt;
            timer.setOverflow(UINT16_MAX);
            timer.attachInterrupt(handle_overflow);
        }
        else
        {
            ovf_left = 0;
            timer.setOverflow((value & 0xFFFF) - 1);
            timer.attachInterrupt(handle_compare_match);
        }

        timer.refresh();
        timer.resume();

        SET_INTERVAL_TIMING_END();
    }

    static inline __attribute__((always_inline)) void handle_overflow()
    {
        INTERRUPT_TIMING_START();

        // decrement amount of left overflows
        ovf_left--;

        // check if this was the last overflow and we need to count the rest
        if (ovf_left == 0)
        {
            timer.attachInterrupt(handle_compare_match);
        }
        INTERRUPT_TIMING_END();
    }

    static inline __attribute__((always_inline)) void handle_compare_match()
    {
        INTERRUPT_TIMING_START();

        // check if we need to switch to overflows again (slow frequency)
        if (ovf_cnt)
        {
            // reset overflow countdown
            ovf_left = ovf_cnt;
            timer.attachInterrupt(handle_overflow);
        }

        // execute the callback
        callback();

        INTERRUPT_TIMING_END();
    }
};

template <Timer T>
HardwareTimer IntervalInterrupt_STM32<T>::timer = HardwareTimer(timer_def_from_enum(T));

template <Timer T>
volatile uint16_t IntervalInterrupt_STM32<T>::ovf_cnt = 0;

template <Timer T>
volatile uint16_t IntervalInterrupt_STM32<T>::ovf_left = 0;

template <Timer T>
volatile timer_callback IntervalInterrupt_STM32<T>::callback = nullptr;

template <Timer T>
void IntervalInterrupt<T>::init()
{
#if DEBUG_INTERRUPT_SET_INTERVAL_PIN
    Pin<DEBUG_INTERRUPT_SET_INTERVAL_PIN>::init();
#endif
#if DEBUG_INTERRUPT_TIMING_PIN
    Pin<DEBUG_INTERRUPT_TIMING_PIN>::init();
#endif

    IntervalInterrupt_STM32<T>::init();
}

template <Timer T>
inline __attribute__((always_inline)) void IntervalInterrupt<T>::setInterval(uint32_t value)
{
    IntervalInterrupt_STM32<T>::set_interval(value);
}

template <Timer T>
void inline __attribute__((always_inline)) IntervalInterrupt<T>::setCallback(timer_callback fn)
{
    IntervalInterrupt_STM32<T>::callback = fn;
}

template <Timer T>
inline __attribute__((always_inline)) void IntervalInterrupt<T>::stop()
{
    IntervalInterrupt_STM32<T>::timer.pause();
}

template <Timer T>
const uint32_t IntervalInterrupt<T>::FREQ = F_CPU;

#endif