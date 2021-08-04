#pragma once

#include <avr/io.h>
#include <avr/interrupt.h>

typedef void (*timer_callback)(void);

// template <typename SIZE>
class PreciseInterrupt
{
private:
    volatile timer_callback callback;
    volatile uint16_t ovf_cnt = 0;
    volatile uint16_t ovf_left = 0;

public:
    PreciseInterrupt() {}

    void init()
    {
        cli();
        TCCR1A = 0;                                       // reset timer/counter control register A
        TCCR1B = (0 << CS12) | (0 << CS11) | (0 << CS10); // set prescaler to 0 (disable this interrupt)
        TCNT1 = 0;                                        // reset counter

        TIMSK1 |= (1 << TOIE1); // enable interrupt on counter overflow
        sei();
    }

    inline __attribute__((always_inline)) void setInterval(uint32_t value)
    {
        ovf_cnt = (value >> 16);
        OCR1A = (value & 0xFFFF) - 1;

        // if we don't need any overflow (high frequency), we have to enable compare interrupt now
        if (!ovf_cnt)
        {
            cli();
            TIMSK1 |= (1 << OCIE1A); // enable interrupt on counter value compare match
            TCCR1B |= (1 << WGM12);  // enable CTC mode (clear timer on compare)
            sei();
        }
        else
        {
            ovf_left = ovf_cnt;
        }

        TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // set prescaler to 1 (count at MCU frequency)
    }

    inline __attribute__((always_inline)) void stop()
    {
        TCCR1B = (0 << CS12) | (0 << CS11) | (0 << CS10); // set prescaler to 0 (stop interrupts)
    }

    inline __attribute__((always_inline)) void setFrequency(float frequency)
    {
        if (frequency > 0)
        {
            setInterval(F_CPU / frequency);
        }
        else
        {
            stop();
        }
    }

    void setCallback(timer_callback callback)
    {
        this->callback = callback;
    }

    inline __attribute__((always_inline)) void handle_overflow()
    {
        // decrement amount of left overflows
        ovf_left--;

        // check if this was the last overflow and we need to count the rest
        if (ovf_left == 0)
        {
            TIMSK1 |= (1 << OCIE1A); // Timer/Counter1 Output Compare A Match Interrupt Enable
            TCCR1B |= (1 << WGM12);  // enable CTC mode (clear timer on compare)

            // clear Output Compare Flag 1A because it was set during overflow mode. 
            // Not doing so will lead to interrupt executing instantly after this
            TIFR1 |= (1 << OCF1A); 
        }
    }

    inline __attribute__((always_inline)) void handle_compare_match()
    {
        // check if we need to switch to overflows again (slow frequency)
        if (ovf_cnt)
        {
            TIMSK1 &= ~(1 << OCIE1A); // disable interrupt on counter value compare match
            TCCR1B &= ~(1 << WGM12);  // disable CTC mode (clear timer on compare)
            ovf_left = ovf_cnt;
        }

        // execute the callback
        // FastPin<60>::high();
        callback();
        // FastPin<60>::low();
    }
};

PreciseInterrupt isr1;

ISR(TIMER1_OVF_vect)
{
    FastPin<60>::high();
    isr1.handle_overflow();
    FastPin<60>::low();
}

ISR(TIMER1_COMPA_vect)
{
    FastPin<60>::high();
    isr1.handle_compare_match();
    FastPin<60>::low();
}