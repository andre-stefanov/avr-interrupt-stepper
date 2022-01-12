#include <unity.h>

#define F_CPU 16000000

#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "utils.h"
#include "Constants.h"
#include <AccelerationRamp.h>

template <uint8_t T_STAIRS, uint32_t T_FREQ, uint32_t T_SPR, uint32_t T_MAX_SPEED_mRAD, uint32_t T_ACCELERATION_mRAD>
class TestCase
{
    TestCase() = delete;

public:
    typedef AccelerationRamp<T_STAIRS, T_FREQ, T_SPR, T_MAX_SPEED_mRAD, T_ACCELERATION_mRAD> TestRamp;

    constexpr static Angle MAX_SPEED = Angle::from_mrad_u32(T_MAX_SPEED_mRAD);
    constexpr static Angle ACCELERATION = Angle::from_mrad_u32(T_ACCELERATION_mRAD);

    static void test_stairs_count()
    {
        TEST_ASSERT_EQUAL_UINT8(T_STAIRS, TestRamp::STAIRS_COUNT);
    }

    static void test_steps_per_stair()
    {
        auto max_s_lim = (MAX_SPEED.rad() * MAX_SPEED.rad()) / (2 * (2 * 3.14159265358979323846f / T_SPR) * ACCELERATION.rad());
        auto steps_per_stair = max_s_lim / T_STAIRS;
        TEST_ASSERT_TRUE(is_pow2(TestRamp::STEPS_PER_STAIR));
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
        TEST_ASSERT_EQUAL_UINT8(T_STAIRS, TestRamp::maxAccelStairs(MAX_SPEED.rad()));
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
        TEST_ASSERT_FLOAT_WITHIN(1, interval, TestRamp::getIntervalForSpeed(MAX_SPEED.rad()));
    }

    static void test_getIntervalForSpeed_half()
    {
        constexpr double rad_per_step = 2.0 * M_PI / T_SPR;
        constexpr double step_freq = MAX_SPEED.rad() / 2 / rad_per_step;
        constexpr double interval = T_FREQ / step_freq;
        TEST_ASSERT_FLOAT_WITHIN(1, interval, TestRamp::getIntervalForSpeed(MAX_SPEED.rad() / 2));
    }

    static void run()
    {
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
    using namespace axis::ra;

    constexpr auto DEG_1 = Angle::from_deg(TRANSMISSION);
    constexpr auto DEG_4 = 4 * DEG_1;

    UNITY_BEGIN();

    TestCase<128U, F_CPU, 400U * 64U, DEG_4.mrad_u32(), 2 * DEG_4.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 64U, DEG_4.mrad_u32(), 4 * DEG_4.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 256U, DEG_4.mrad_u32(), 2 * DEG_4.mrad_u32()>::run();
    TestCase<128U, F_CPU, 400U * 256U, DEG_4.mrad_u32(), 2 * DEG_4.mrad_u32()>::run();

    // using TestRamp = TestCase<128U, F_CPU, 400U * 256U, DEG_4.mrad_u32(), 2 * DEG_4.mrad_u32()>::TestRamp;

    UNITY_END();
}
