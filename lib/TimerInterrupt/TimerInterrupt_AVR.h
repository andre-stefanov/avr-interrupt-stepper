#pragma once

#ifdef ARDUINO_ARCH_AVR

#include <TimerInterrupt.h>
#include <stdint.h>
#include <avr/interrupt.h>

#define DEBUG_INTERRUPT_TIMING_PIN 49
#if DEBUG_INTERRUPT_TIMING_PIN != 0 && UNIT_TEST != 1
#include <Pin.h>
#define INTERRUPT_TIMING_START() Pin<DEBUG_INTERRUPT_TIMING_PIN>::high()
#define INTERRUPT_TIMING_END() Pin<DEBUG_INTERRUPT_TIMING_PIN>::low()
#else
#define INTERRUPT_TIMING_START()
#define INTERRUPT_TIMING_END()
#endif

/**
 * @brief Enumeration of 16 bit timers on atmega2560
 */
enum class Timer : int
{
#ifdef USE_STEPPER_TIMER_1
    TIMER_1 = 1,
#endif
#ifdef USE_STEPPER_TIMER_3
    TIMER_3 = 3,
#endif
#ifdef USE_STEPPER_TIMER_4
    TIMER_4 = 4,
#endif
#ifdef USE_STEPPER_TIMER_5
    TIMER_5 = 5,
#endif
};

template <Timer T>
struct TimerInterrupt_AVR
{
    static volatile uint8_t *const _tccra;
    static volatile uint8_t *const _tccrb;
    static volatile uint16_t *const _tcnt;
    static volatile uint8_t *const _timsk;
    static volatile uint16_t *const _ocra;
    static volatile uint8_t *const _tifr;

    static volatile uint16_t ovf_cnt;
    static volatile uint16_t ovf_left;

    static volatile timer_callback callback;

    static inline __attribute__((always_inline)) void handle_overflow()
    {
        INTERRUPT_TIMING_START();
        // decrement amount of left overflows
        ovf_left--;

        // check if this was the last overflow and we need to count the rest
        if (ovf_left == 0)
        {
            // Timer/Counter1 Output Compare A Match Interrupt Enable
            *_timsk |= (1 << 1);

            // enable CTC mode (clear timer on compare)
            *_tccrb |= (1 << 3);

            // clear Output Compare Flag 1A because it was set during overflow mode.
            // Not doing so will lead to interrupt executing instantly after this
            *_tifr |= (1 << 1);
        }
        INTERRUPT_TIMING_END();
    }

    static inline __attribute__((always_inline)) void handle_compare_match()
    {
        INTERRUPT_TIMING_START();

        // check if we need to switch to overflows again (slow frequency)
        if (ovf_cnt)
        {
            // disable interrupt on counter value compare match
            *_timsk &= ~(1 << 1);

            // disable CTC mode (clear timer on compare)
            *_tccrb &= ~(1 << 3);

            // reset overflow countdown
            ovf_left = ovf_cnt;
        }

        // execute the callback
        callback();

        INTERRUPT_TIMING_END();
    }
};

template <Timer T>
volatile uint16_t TimerInterrupt_AVR<T>::ovf_cnt = 0;

template <Timer T>
volatile uint16_t TimerInterrupt_AVR<T>::ovf_left = 0;

#if defined(USE_STEPPER_TIMER_3) && !defined(USE_STEPPER_TIMER_3_INITIALIZED)
#define USE_STEPPER_TIMER_3_INITIALIZED

template <>
volatile uint8_t *const TimerInterrupt_AVR<Timer::TIMER_3>::_tccra = &TCCR3A;
template <>
volatile uint8_t *const TimerInterrupt_AVR<Timer::TIMER_3>::_tccrb = &TCCR3B;
template <>
volatile uint8_t *const TimerInterrupt_AVR<Timer::TIMER_3>::_timsk = &TIMSK3;
template <>
volatile uint8_t *const TimerInterrupt_AVR<Timer::TIMER_3>::_tifr = &TIFR3;
template <>
volatile uint16_t *const TimerInterrupt_AVR<Timer::TIMER_3>::_tcnt = &TCNT3;
template <>
volatile uint16_t *const TimerInterrupt_AVR<Timer::TIMER_3>::_ocra = &OCR3A;

template <Timer T>
volatile timer_callback TimerInterrupt_AVR<T>::callback = nullptr;

ISR(TIMER3_OVF_vect) { TimerInterrupt_AVR<Timer::TIMER_3>::handle_overflow(); }
ISR(TIMER3_COMPA_vect) { TimerInterrupt_AVR<Timer::TIMER_3>::handle_compare_match(); }

#endif

template <Timer T>
const unsigned long int TimerInterrupt<T>::FREQ = F_CPU;

template <Timer T>
void TimerInterrupt<T>::init()
{
    cli();                                      // disable interrupts
    *TimerInterrupt_AVR<T>::_tccra = 0;         // reset timer/counter control register A
    *TimerInterrupt_AVR<T>::_tccrb = 0;         // set prescaler to 0 (disable this interrupt)
    *TimerInterrupt_AVR<T>::_tcnt = 0;          // reset counter
    *TimerInterrupt_AVR<T>::_timsk |= (1 << 0); // enable interrupt on counter overflow
    sei();                                      // reenable interrupts
}

template <Timer T>
inline __attribute__((always_inline)) void TimerInterrupt<T>::setInterval(uint32_t value)
{
    // lower 16 bit of value will be used for compare match. thus we are only interested
    // in higher 16 bit for amount of overflows
    TimerInterrupt_AVR<T>::ovf_cnt = (value >> 16);

    // set compare match to the value of lower 16 bits
    *TimerInterrupt_AVR<T>::_ocra = (value & 0xFFFF) - 1;

    if (TimerInterrupt_AVR<T>::ovf_cnt == 0)
    {
        // if we don't need any overflow (high frequency), we have to enable compare interrupt now
        cli();
        *TimerInterrupt_AVR<T>::_timsk |= (1 << 1); // enable interrupt on counter value compare match
        *TimerInterrupt_AVR<T>::_tccrb |= (1 << 3); // enable CTC mode (clear timer on compare)
        sei();
    }
    else
    {
        // otherwise we set overflow countdown to the calculated value (higher 16 bit)
        TimerInterrupt_AVR<T>::ovf_left = TimerInterrupt_AVR<T>::ovf_cnt;
    }

    // set prescaler to 1 (count at MCU frequency)
    *TimerInterrupt_AVR<T>::_tccrb |= 1;

    Pin<48>::pulse();
}

template <Timer T>
void inline __attribute__((always_inline)) TimerInterrupt<T>::setCallback(timer_callback fn)
{
    TimerInterrupt_AVR<T>::callback = fn;
}

template <Timer T>
inline __attribute__((always_inline)) void TimerInterrupt<T>::stop()
{
    // set prescaler to 0 (stop interrupts)
    *TimerInterrupt_AVR<T>::_tccrb = 0;

    // set counter to 0
    *TimerInterrupt_AVR<T>::_tcnt = 0;
    
    Pin<48>::pulse();
}

#endif