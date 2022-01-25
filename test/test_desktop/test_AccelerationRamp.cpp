#include <unity.h>

#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "Angle.h"
#include <AccelerationRamp.h>

template <uint8_t T_STAIRS, uint32_t T_FREQ, uint32_t T_SPR, uint32_t T_MAX_SPEED_mRAD, uint32_t T_ACCELERATION_mRAD>
class TestCase
{
    TestCase() = delete;

    typedef AccelerationRamp<T_STAIRS, T_FREQ, T_SPR, T_MAX_SPEED_mRAD, T_ACCELERATION_mRAD> TestRamp;

    constexpr static Angle MAX_SPEED = Angle::mrad_u32(T_MAX_SPEED_mRAD);
    constexpr static Angle ACCELERATION = Angle::mrad_u32(T_ACCELERATION_mRAD);

    static void test_stairs_count()
    {
        TEST_ASSERT_EQUAL_UINT8(T_STAIRS, TestRamp::STAIRS_COUNT);
    }

    static void test_steps_per_stair()
    {
        auto max_s_lim = (MAX_SPEED.rad() * MAX_SPEED.rad()) / (2 * (2 * M_PI / T_SPR) * ACCELERATION.rad());
        auto steps_per_stair = max_s_lim / T_STAIRS;
        TEST_ASSERT_LESS_OR_EQUAL(steps_per_stair, TestRamp::STEPS_PER_STAIR);
    }

    static void test_acceleration_intervals_decrementing()
    {
        for (size_t i = 0; i < T_STAIRS - 1; i++)
        {
            auto i1 = TestRamp::intervals[i];
            auto i2 = TestRamp::intervals[i + 1];

            TEST_ASSERT_GREATER_THAN_UINT32(i2, i1);
        }
    }

    static void test_maxAccelStairs_speed_0()
    {
        TEST_ASSERT_EQUAL_UINT8(0, TestRamp::maxAccelStairs(0.0f));
    }

    static void test_maxAccelStairs_speed_max()
    {
        TEST_ASSERT_EQUAL_UINT8(T_STAIRS - 1, TestRamp::maxAccelStairs(MAX_SPEED.rad()));
    }

    static void test_maxAccelStairs_speed_half()
    {
        TEST_ASSERT_EQUAL_UINT8(T_STAIRS / 4, TestRamp::maxAccelStairs(MAX_SPEED.rad() / 2));
    }

    static void test_getIntervalForSpeed_max()
    {
        constexpr double rad_per_step = 2.0 * M_PI / T_SPR;
        constexpr double step_freq = MAX_SPEED.rad() / rad_per_step;
        constexpr double interval = T_FREQ / step_freq;
        TEST_ASSERT_FLOAT_WITHIN(0.5, interval, TestRamp::getIntervalForSpeed(MAX_SPEED.rad()));
    }

    static void test_getIntervalForSpeed_half()
    {
        constexpr double rad_per_step = 2.0 * M_PI / T_SPR;
        constexpr double step_freq = MAX_SPEED.rad() / 2 / rad_per_step;
        constexpr double interval = T_FREQ / step_freq;
        TEST_ASSERT_FLOAT_WITHIN(1, interval, TestRamp::getIntervalForSpeed(MAX_SPEED.rad() / 2));
    }

    static void dump()
    {
        UNITY_PRINT_EOL();
        UnityPrint("Test case");
        UNITY_PRINT_EOL();
        UnityPrint("STAIRS: ");
        UnityPrintNumber(T_STAIRS);
        UNITY_PRINT_EOL();
        UnityPrint("SPR: ");
        UnityPrintNumber(T_SPR);
        UNITY_PRINT_EOL();
        UnityPrint("MAX_SPEED: ");
        UnityPrintFloat(MAX_SPEED.deg());
        UNITY_PRINT_EOL();
        UnityPrint("ACCELERATION: ");
        UnityPrintFloat(ACCELERATION.deg());
        UNITY_PRINT_EOL();
    }

public:
    static void run()
    {
        dump();

        RUN_TEST(test_stairs_count);
        RUN_TEST(test_steps_per_stair);
        RUN_TEST(test_acceleration_intervals_decrementing);
        RUN_TEST(test_maxAccelStairs_speed_0);
        RUN_TEST(test_maxAccelStairs_speed_max);
        RUN_TEST(test_maxAccelStairs_speed_half);
        RUN_TEST(test_getIntervalForSpeed_max);
        RUN_TEST(test_getIntervalForSpeed_half);
    }
};

void test_AccelerationRamp()
{
    constexpr auto TRANSMISSION = 35.46611505122143f;

    constexpr auto SPEED_SLEWING_2 = Angle::deg(2.0f) * TRANSMISSION;
    constexpr auto SPEED_SLEWING_4 = Angle::deg(4.0f) * TRANSMISSION;

    // constexpr auto SPEED_TRACKING = Angle::deg(360.0f / SIDEREAL) * TRANSMISSION;

    UNITY_BEGIN();

    TestCase<64U, F_CPU, 400U * 64U, SPEED_SLEWING_2.mrad_u32(), 2 * SPEED_SLEWING_2.mrad_u32()>::run();
    TestCase<64U, F_CPU, 400U * 64U, SPEED_SLEWING_4.mrad_u32(), 4 * SPEED_SLEWING_4.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 64U, SPEED_SLEWING_2.mrad_u32(), 2 * SPEED_SLEWING_2.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 64U, SPEED_SLEWING_4.mrad_u32(), 4 * SPEED_SLEWING_4.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 256U, SPEED_SLEWING_2.mrad_u32(), 2 * SPEED_SLEWING_2.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 256U, SPEED_SLEWING_4.mrad_u32(), 2 * SPEED_SLEWING_4.mrad_u32()>::run();

    // using TestRamp = TestCase<128U, F_CPU, 400U * 256U, SPEED_4.mrad_u32(), 2 * SPEED_4.mrad_u32()>::TestRamp;

    UNITY_END();
}
