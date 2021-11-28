#include "TimerInterrupt.h"

#if __has_include(<avr/io.h>)

volatile uint8_t * const TCCRA(Timer timer)
{
    switch (timer)
    {
    case TIMER_1:
        return &TCCR1A;
    case TIMER_3:
        return &TCCR3A;
    case TIMER_4:
        return &TCCR4A;
    case TIMER_5:
        return &TCCR5A;
    default:
        return 0;
    }
}

volatile uint8_t * const TCCRB(Timer timer)
{
    switch (timer)
    {
    case TIMER_1:
        return &TCCR1B;
    case TIMER_3:
        return &TCCR3B;
    case TIMER_4:
        return &TCCR4B;
    case TIMER_5:
        return &TCCR5B;
    default:
        return 0;
    }
}

volatile uint8_t * const TIMSK(Timer timer)
{
    switch (timer)
    {
    case TIMER_1:
        return &TIMSK1;
    case TIMER_3:
        return &TIMSK3;
    case TIMER_4:
        return &TIMSK4;
    case TIMER_5:
        return &TIMSK5;
    default:
        return 0;
    }
}

volatile uint8_t * const TIFR(Timer timer)
{
    switch (timer)
    {
    case TIMER_1:
        return &TIFR1;
    case TIMER_3:
        return &TIFR3;
    case TIMER_4:
        return &TIFR4;
    case TIMER_5:
        return &TIFR5;
    default:
        return 0;
    }
}

volatile uint16_t * const OCRA(Timer timer)
{
    switch (timer)
    {
    case TIMER_1:
        return &OCR1A;
    case TIMER_3:
        return &OCR3A;
    case TIMER_4:
        return &OCR4A;
    case TIMER_5:
        return &OCR5A;
    default:
        return 0;
    }
}

volatile uint16_t * const TCNT(Timer timer)
{
    switch (timer)
    {
    case TIMER_1:
        return &TCNT1;
    case TIMER_3:
        return &TCNT3;
    case TIMER_4:
        return &TCNT4;
    case TIMER_5:
        return &TCNT5;
    default:
        return 0;
    }
}

#endif