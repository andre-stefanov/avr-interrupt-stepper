#pragma once

#include <Arduino.h>

template <uint8_t PIN>
class FastPin
{
private:                                                     
    static volatile uint8_t *_out_reg;
    static volatile uint8_t *_mode_reg;
    static uint8_t _bit;

    static void init(const uint8_t mode)
    {
        _out_reg = portOutputRegister(digitalPinToPort(PIN));
        _mode_reg = portModeRegister(digitalPinToPort(PIN));
        _bit = digitalPinToBitMask(PIN);

        if (mode == INPUT)
        {
            *_mode_reg &= ~_bit;
            *_out_reg &= ~_bit;
        }
        else if (mode == INPUT_PULLUP)
        {
            *_mode_reg &= ~_bit;
            *_out_reg |= _bit;
        }
        else
        {
            *_mode_reg |= _bit;
        }
    }

public:
    FastPin() = delete;

    static void output()
    {
        init(OUTPUT);
    }

    static inline __attribute__((always_inline)) void pulse()
    {
        uint8_t reg_state = *_out_reg;
        *_out_reg |= _bit;
        *_out_reg = reg_state;
    }

    static inline __attribute__((always_inline)) void high()
    {
        *_out_reg |= _bit;
    }

    static inline __attribute__((always_inline)) void low()
    {
        *_out_reg &= ~_bit;
    }

};

template<uint8_t PIN> volatile uint8_t * FastPin<PIN>::_out_reg;
template<uint8_t PIN> volatile uint8_t * FastPin<PIN>::_mode_reg;
template<uint8_t PIN> uint8_t FastPin<PIN>::_bit;