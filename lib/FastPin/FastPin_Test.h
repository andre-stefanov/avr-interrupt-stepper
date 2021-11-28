#ifdef UNIT_TEST

#include "stdint.h"
#include "gmock/gmock.h"

using ::testing::StrictMock;

class MockPin
{
public:
    MOCK_METHOD(void, init, ());
    MOCK_METHOD(void, pulse, ());
    MOCK_METHOD(void, high, ());
    MOCK_METHOD(void, low, ());
};

template <uint8_t PIN>
class FastPin
{
public:
    // static bool initialized;
    // static bool value;
    // static unsigned long long pulseCnt;
    static StrictMock<MockPin> mock;

public:
    FastPin() = delete;

    static void init()
    {
        mock.init();
    }

    static inline __attribute__((always_inline)) void pulse()
    {
        mock.pulse();
    }

    static inline __attribute__((always_inline)) void pulse(unsigned int width)
    {
        high();
        low();
    }

    static inline __attribute__((always_inline)) void high()
    {
        mock.high();
    }

    static inline __attribute__((always_inline)) void low()
    {
        mock.low();
    }
};

template <uint8_t PIN>
StrictMock<MockPin> FastPin<PIN>::mock;

#endif