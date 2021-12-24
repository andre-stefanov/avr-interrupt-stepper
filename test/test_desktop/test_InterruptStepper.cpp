#include <unity.h>

#define MICROSTEPPING 64LL
#define SPR (400LL * MICROSTEPPING)
#define SIDEREAL 86164.0905f
#define TRANSMISSION 35.46611505122143

#define TRACKING_DEG_PER_SEC (TRANSMISSION * SPR / SIDEREAL)

#include "InterruptStepper.h"

template <uint8_t PIN>
struct TestPin
{
    static uint32_t toggle_count;
    static bool state;

    static void pulse()
    {
        high();
        low();
        toggle_count++;
    }

    static void high() { state = true; }

    static void low() { state = false; }

    static void reset()
    {
        toggle_count = 0;
        state = false;
    }
};

template <uint8_t PIN>
uint32_t TestPin<PIN>::toggle_count = 0;

template <uint8_t PIN>
bool TestPin<PIN>::state = false;

typedef void (*timer_callback)(void);

struct TestTimerInterrupt
{
    static bool initialized;
    static bool stopped;
    static uint32_t interval;
    static timer_callback callback;

    static void init()
    {
        initialized = true;
    }

    static void stop()
    {
        stopped = true;
    }

    static void setInterval(uint32_t value)
    {
        UnityPrintNumber(value);
        interval = value;
        stopped = false;
    }

    static void setCallback(timer_callback fn)
    {
        callback = fn;
    }
};

bool TestTimerInterrupt::initialized = false;
bool TestTimerInterrupt::stopped = true;
uint32_t TestTimerInterrupt::interval = 0;
timer_callback TestTimerInterrupt::callback = nullptr;

typedef InterruptStepper<TestTimerInterrupt, 64, Driver<TestPin<0>, TestPin<1>>> TestStepper;

void test_init_timer()
{
    TEST_ASSERT_FALSE(TestTimerInterrupt::initialized);
    TestStepper::init(1000.0f, 1000.0f);
    TEST_ASSERT_TRUE_MESSAGE(TestTimerInterrupt::initialized, "Interrupt was not initialized as expected");
}

void test_move_10000_sps()
{
    TestStepper::init(DEG_TO_RAD * 2, DEG_TO_RAD * 2);
    TestStepper::move(DEG_TO_RAD * 2, 10);

    TEST_ASSERT_EQUAL_UINT32(10, TestTimerInterrupt::interval);

    while (!TestTimerInterrupt::stopped)
    {
        TestTimerInterrupt::callback();
    }

    TEST_ASSERT_EQUAL_UINT32(10, TestPin<0>::toggle_count);
}

int main(int argc, char **argv)
{
    UNITY_BEGIN();
    RUN_TEST(test_init_timer);
    RUN_TEST(test_move_10000_sps);
    UNITY_END();

    return 0;
}