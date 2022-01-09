#include <unity.h>

#define F_CPU 16000000

#include "Constants.h"
#include <AccelerationRamp.h>

#include <stdio.h>

using namespace axis::ra;

constexpr uint8_t STAIRS_COUNT = 64;

using TestRamp = AccelerationRamp<STAIRS_COUNT, F_CPU, SPR, SLEWING_SPEED.mrad_u32(), 4 * SLEWING_SPEED.mrad_u32()>;

void test_stairs_count()
{
    TEST_ASSERT_EQUAL_UINT8(STAIRS_COUNT, TestRamp::STAIRS_COUNT);
}

void test_steps_per_stair()
{
    TEST_ASSERT_EQUAL_UINT8(8, TestRamp::STEPS_PER_STAIR);
}

void test_acceleration_intervals_decrementing()
{
    for (size_t i = 0; i < STAIRS_COUNT - 1; i++)
    {
        auto i1 = TestRamp::intervals[i];
        auto i2 = TestRamp::intervals[i + 1];

        TEST_ASSERT_GREATER_THAN_UINT32(i2, i1);
    }
}

void test_maxAccelStairs_speed_0()
{
    TEST_ASSERT_EQUAL_UINT8(0, TestRamp::maxAccelStairs(0.0f));
}

void test_maxAccelStairs_speed_max()
{
    TEST_ASSERT_EQUAL_UINT8(64, TestRamp::maxAccelStairs(SLEWING_SPEED.rad()));
}

void test_maxAccelStairs_speed_half()
{
    TEST_ASSERT_EQUAL_UINT8(16, TestRamp::maxAccelStairs(SLEWING_SPEED.rad() / 2));
}

void test_AccelerationRamp()
{
    UNITY_BEGIN();

    RUN_TEST(test_stairs_count);
    RUN_TEST(test_steps_per_stair);
    RUN_TEST(test_acceleration_intervals_decrementing);
    RUN_TEST(test_maxAccelStairs_speed_0);
    RUN_TEST(test_maxAccelStairs_speed_max);
    RUN_TEST(test_maxAccelStairs_speed_half);

    UNITY_END();
}
