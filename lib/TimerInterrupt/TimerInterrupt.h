#pragma once

#if __has_include(<avr/io.h>)

#include <avr/io.h>
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

#define INLINE inline __attribute__((always_inline))

typedef void (*timer_callback)(void);

enum Timer
{
    TIMER_1 = 1,
    TIMER_3 = 3,
    TIMER_4 = 4,
    TIMER_5 = 5,
};

/**
 * @brief This class is capable of calling a callback function after a precise amount of clocks on an AVR MCU.
 * It is meant to be used for very time critical operations. 
 * 
 * It uses overflow and compare vectors to achieve long and short interrupt intervals. In case of a long interval
 * multiple overflows will be used until there are less than 2^16 clocks left until the interval end. Then the 
 * mode will be switched to a compare which will call the callback function on trigger.
 */
template <Timer T>
class TimerInterrupt
{
private:
    static volatile uint8_t *const _tccra;
    static volatile uint8_t *const _tccrb;
    static volatile uint16_t *const _tcnt;
    static volatile uint8_t *const _timsk;
    static volatile uint16_t *const _ocra;
    static volatile uint8_t *const _tifr;

    static volatile timer_callback _callback;
    static volatile uint16_t ovf_cnt;
    static volatile uint16_t ovf_left;

private:
    TimerInterrupt() {}

public:
    constexpr static auto FREQ = F_CPU;

    static void init()
    {
        cli();               // disable interrupts
        *_tccra = 0;         // reset timer/counter control register A
        *_tccrb = 0;         // set prescaler to 0 (disable this interrupt)
        *_tcnt = 0;          // reset counter
        *_timsk |= (1 << 0); // enable interrupt on counter overflow
        sei();               // reenable interrupts
    }

    static INLINE void setInterval(uint32_t value)
    {
        // lower 16 bit of value will be used for compare match. thus we are only interested
        // in higher 16 bit for amount of overflows
        ovf_cnt = (value >> 16);

        // set compare match to the value of lower 16 bits
        *_ocra = (value & 0xFFFF) - 1;

        if (!ovf_cnt)
        {
            // if we don't need any overflow (high frequency), we have to enable compare interrupt now
            cli();
            *_timsk |= (1 << 1); // enable interrupt on counter value compare match
            *_tccrb |= (1 << 3); // enable CTC mode (clear timer on compare)
            sei();
        }
        else
        {
            // otherwise we set overflow countdown to the calculated value (higher 16 bit)
            ovf_left = ovf_cnt;
        }

        // set prescaler to 1 (count at MCU frequency)
        *_tccrb |= 1;
    }

    static INLINE void stop()
    {
        // set prescaler to 0 (stop interrupts)
        *_tccrb = 0;
    }

    static INLINE void setCallback(timer_callback fn)
    {
        _callback = fn;
    }

    static INLINE void handle_overflow()
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

    static INLINE void handle_compare_match()
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
        _callback();

        INTERRUPT_TIMING_END();
    }
};

volatile uint8_t *const TCCRA(Timer timer);
volatile uint8_t *const TCCRB(Timer timer);
volatile uint8_t *const TIMSK(Timer timer);
volatile uint8_t *const TIFR(Timer timer);
volatile uint16_t *const TCNT(Timer timer);
volatile uint16_t *const OCRA(Timer timer);

template <Timer T>
volatile timer_callback TimerInterrupt<T>::_callback = nullptr;

template <Timer T>
volatile uint16_t TimerInterrupt<T>::ovf_cnt = 0;

template <Timer T>
volatile uint16_t TimerInterrupt<T>::ovf_left = 0;

template <Timer T>
volatile uint8_t *const TimerInterrupt<T>::_tccra = TCCRA(T);

template <Timer T>
volatile uint8_t *const TimerInterrupt<T>::_tccrb = TCCRB(T);

template <Timer T>
volatile uint8_t *const TimerInterrupt<T>::_timsk = TIMSK(T);

template <Timer T>
volatile uint8_t *const TimerInterrupt<T>::_tifr = TIFR(T);

template <Timer T>
volatile uint16_t *const TimerInterrupt<T>::_tcnt = TCNT(T);

template <Timer T>
volatile uint16_t *const TimerInterrupt<T>::_ocra = OCRA(T);

#endif