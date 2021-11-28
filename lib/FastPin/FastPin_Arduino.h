#ifndef UNIT_TEST

#include <Arduino.h>

template <uint8_t PIN>
class FastPin
{
public:
    FastPin() = delete;

    static void init()
    {
        pinMode(PIN, OUTPUT);
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
        digitalWrite(PIN, HIGH);
    }

    static inline __attribute__((always_inline)) void low()
    {
        digitalWrite(PIN, LOW);
    }

};

#endif