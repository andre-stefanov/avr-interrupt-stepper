#ifndef UNIT_TEST

#include <Arduino.h>

#define PT(PIN) if ((PIN) == 53) { PINB = B00000001; }
#define PH(PIN) if ((PIN) == 53) { PORTB |= B00000001; }
#define PL(PIN) if ((PIN) == 53) { PORTB &= B11111110; }

template <uint8_t PIN>
class FastPin
{
private:                                                     
    static volatile uint8_t *_out_reg;
    static uint8_t _bit;
    static uint8_t _neg;

public:
    FastPin() = delete;

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

template<uint8_t PIN> volatile uint8_t * FastPin<PIN>::_out_reg;
template<uint8_t PIN> uint8_t FastPin<PIN>::_bit;
template<uint8_t PIN> uint8_t FastPin<PIN>::_neg;

#endif