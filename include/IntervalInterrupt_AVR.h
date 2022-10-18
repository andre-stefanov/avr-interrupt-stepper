#pragma once

#if defined(ARDUINO_ARCH_AVR)

#include <IntervalInterrupt.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define DEBUG_INTERRUPT_TIMING_PIN 46
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
    TIMER_1 = 1,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
};

template <Timer T>
struct IntervalInterrupt_AVR
{
    static volatile uint8_t ovf_cnt;
    static volatile uint8_t ovf_left;

    static volatile timer_callback callback;

    constexpr static inline __attribute__((always_inline)) volatile uint8_t *TCCRA()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return &TCCR1A;
        case Timer::TIMER_3:
            return &TCCR3A;
        case Timer::TIMER_4:
            return &TCCR4A;
        case Timer::TIMER_5:
            return &TCCR5A;
        default:
            return 0;
        }
    }

    constexpr static inline __attribute__((always_inline)) volatile uint8_t *TCCRB()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return &TCCR1B;
        case Timer::TIMER_3:
            return &TCCR3B;
        case Timer::TIMER_4:
            return &TCCR4B;
        case Timer::TIMER_5:
            return &TCCR5B;
        default:
            return 0;
        }
    }

    constexpr static inline __attribute__((always_inline)) volatile uint16_t *OCRA()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return &OCR1A;
        case Timer::TIMER_3:
            return &OCR3A;
        case Timer::TIMER_4:
            return &OCR4A;
        case Timer::TIMER_5:
            return &OCR5A;
        default:
            return 0;
        }
    }

    constexpr static inline __attribute__((always_inline)) volatile uint16_t *OCIE()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return OCIE1A;
        case Timer::TIMER_3:
            return OCIE3A;
        case Timer::TIMER_4:
            return OCIE4A;
        case Timer::TIMER_5:
            return OCIE5A;
        default:
            return 0;
        }
    }

    constexpr static inline __attribute__((always_inline)) volatile uint8_t *TIMSK()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return &TIMSK1;
        case Timer::TIMER_3:
            return &TIMSK3;
        case Timer::TIMER_4:
            return &TIMSK4;
        case Timer::TIMER_5:
            return &TIMSK5;
        default:
            return 0;
        }
    }

    constexpr static inline __attribute__((always_inline)) volatile uint16_t *TCNT()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return &TCNT1;
        case Timer::TIMER_3:
            return &TCNT3;
        case Timer::TIMER_4:
            return &TCNT4;
        case Timer::TIMER_5:
            return &TCNT5;
        default:
            return 0;
        }
    }

    static constexpr volatile uint8_t *TIFR()
    {
        switch (T)
        {
        case Timer::TIMER_1:
            return &TIFR1;
        case Timer::TIMER_3:
            return &TIFR3;
        case Timer::TIMER_4:
            return &TIFR4;
        case Timer::TIMER_5:
            return &TIFR5;
        default:
            return 0;
        }
    }

    static inline __attribute__((always_inline)) void init()
    {
#if DEBUG_INTERRUPT_SET_INTERVAL_PIN != 0
        Pin<DEBUG_INTERRUPT_SET_INTERVAL_PIN>::init();
#endif
#if DEBUG_INTERRUPT_TIMING_PIN != 0
        Pin<DEBUG_INTERRUPT_TIMING_PIN>::init();
#endif
        cli();                // disable interrupts
        *TCCRA() = 0;         // reset timer/counter control register A
        *TCCRB() = 0;         // set prescaler to 0 (disable this interrupt)
        *TCNT() = 0;          // reset counter
        *TIMSK() |= (1 << 0); // enable interrupt on counter overflow
        sei();                // reenable interrupts
    }

    static inline __attribute__((always_inline)) void setInterval(uint32_t value)
    {
        SET_INTERVAL_TIMING_START();

        value = value - 1;

        // lower 16 bit of value will be used for compare match. thus we are only interested
        // in higher 16 bit for amount of overflows
        ovf_cnt = static_cast<uint8_t>(value >> 16);

        // set compare match to the value of lower 16 bits
        *OCRA() = static_cast<uint16_t>((value & 0xFFFF) - 1);

        if (ovf_cnt == 0)
        {
            // if we don't need any overflow (high frequency), we have to enable compare interrupt now
            *TIMSK() |= (1 << 1); // enable interrupt on counter value compare match
            *TCCRB() |= (1 << 3); // enable CTC mode (clear timer on compare)
        }
        else
        {
            // otherwise we set overflow countdown to the calculated value (higher 16 bit)
            ovf_left = ovf_cnt;
        }

        // set prescaler to 1 (count at MCU frequency)
        *TCCRB() |= 1;

        SET_INTERVAL_TIMING_END();
    }

    static inline __attribute__((always_inline)) void stop()
    {
        // set prescaler to 0 (stop interrupts)
        *TCCRB() = 0;

        // set counter to 0
        *TCNT() = 0;
    }

    static inline __attribute__((always_inline)) void handle_overflow()
    {
        INTERRUPT_TIMING_START();

        // check if this was the last overflow and we need to count the rest
        if (--ovf_left == 0)
        {
            // Timer/Counter1 Output Compare A Match Interrupt Enable
            *TIMSK() |= (1 << 1);

            // enable CTC mode (clear timer on compare)
            *TCCRB() |= (1 << 3);

            // clear Output Compare Flag 1A because it was set during overflow mode.
            // Not doing so will lead to interrupt executing instantly after this
            *TIFR() |= (1 << 1);
        }
        INTERRUPT_TIMING_END();
    }

    static inline __attribute__((always_inline)) void handle_compare_match()
    {
        INTERRUPT_TIMING_START();

        // check if we need to switch to overflows again (slow frequency)
        if (ovf_cnt > 0)
        {
            // disable interrupt on counter value compare match
            *TIMSK() &= ~(1 << 1);

            // disable CTC mode (clear timer on compare)
            *TCCRB() &= ~(1 << 3);

            // reset overflow countdown
            ovf_left = ovf_cnt;
        }

        // execute the callback
        callback();

        INTERRUPT_TIMING_END();
    }
};

template <Timer T>
void IntervalInterrupt<T>::init()
{
    IntervalInterrupt_AVR<T>::init();
}

template <Timer T>
inline __attribute__((always_inline)) void IntervalInterrupt<T>::setInterval(uint32_t value)
{
    IntervalInterrupt_AVR<T>::setInterval(value);
}

template <Timer T>
void inline __attribute__((always_inline)) IntervalInterrupt<T>::setCallback(timer_callback fn)
{
    IntervalInterrupt_AVR<T>::callback = fn;
}

template <Timer T>
inline __attribute__((always_inline)) void IntervalInterrupt<T>::stop()
{
    IntervalInterrupt_AVR<T>::stop();
}

template <Timer T>
volatile uint8_t IntervalInterrupt_AVR<T>::ovf_cnt = 0;

template <Timer T>
volatile uint8_t IntervalInterrupt_AVR<T>::ovf_left = 0;

template <Timer T>
volatile timer_callback IntervalInterrupt_AVR<T>::callback = nullptr;

template <Timer T>
const unsigned long int IntervalInterrupt<T>::FREQ = F_CPU;

#endif