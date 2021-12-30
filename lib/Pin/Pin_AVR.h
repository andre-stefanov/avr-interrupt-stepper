#ifndef UNIT_TEST

#include <Arduino.h>

template <uint8_t PIN>
class Pin
{
private:
    static volatile uint8_t *_out_reg;
    static uint8_t _bit;
    static uint8_t _neg;

public:
    Pin() = delete;

    static void init()
    {
        _out_reg = portOutputRegister(digitalPinToPort(PIN));
        _bit = digitalPinToBitMask(PIN);
        _neg = ~_bit;
        *portModeRegister(digitalPinToPort(PIN)) |= _bit;
    }

    static inline __attribute__((always_inline)) void pulse()
    {
        high();
        low();
    }

    static inline __attribute__((always_inline)) void pulse(unsigned int width)
    {
        high();
        delayMicroseconds(width);
        low();
    }

    static inline __attribute__((always_inline)) void high()
    {
        *_out_reg |= _bit;
    }

    static inline __attribute__((always_inline)) void low()
    {
        *_out_reg &= _neg;
    }
};

template <uint8_t PIN>
volatile uint8_t *Pin<PIN>::_out_reg;
template <uint8_t PIN>
uint8_t Pin<PIN>::_bit;
template <uint8_t PIN>
uint8_t Pin<PIN>::_neg;

#endif