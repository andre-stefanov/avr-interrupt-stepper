#include <unity.h>

#include <AccelerationRamp.h>

#define SIDEREAL 86164.0905f
#define TRANSMISSION 35.46611505122143
#define TRACKING_DEG_PER_SEC (360 / SIDEREAL * TRANSMISSION)
#define SLEWING_DEG_PER_SEC (4 * TRANSMISSION)

void test_stairs_count_64()
{
    using TestRamp = AccelerationRamp<64, 16000000, 400 * 64, 10, 20>;
    TEST_ASSERT_EQUAL_UINT8(64, TestRamp::STAIRS_COUNT);
}

void test_steps_per_stair_pow2()
{
    using TestRamp = AccelerationRamp<64, 16000000, 400 * 64, (uint32_t)SLEWING_DEG_PER_SEC, (uint32_t)SLEWING_DEG_PER_SEC * 4>;
    uint8_t s = TestRamp::STEPS_PER_STAIR;
    bool isPow2 = s > 0 && (s & (s - 1)) == 0;
    TEST_ASSERT_TRUE(isPow2);
}

void test_acceleration_intervals()
{
    constexpr auto max_speed = SLEWING_DEG_PER_SEC;
    constexpr auto acceleration = SLEWING_DEG_PER_SEC * 4;
    using TestRamp = AccelerationRamp<64, 16000000, 400 * 64, (uint32_t)max_speed, (uint32_t)acceleration>;
    TestRamp ramp = TestRamp();
    for (size_t i = 1; i < TestRamp::STAIRS_COUNT; i++)
    {
        auto cur_delta = ramp.intervals[i - 1] - ramp.intervals[i];
        TEST_ASSERT_GREATER_THAN_UINT32(0, cur_delta);

        if (i > 1)
        {
            auto prev_delta = ramp.intervals[i - 2] - ramp.intervals[i - 1];
            TEST_ASSERT_LESS_THAN_UINT32(prev_delta, cur_delta);
        }
    }
}

int main(int argc, char const *argv[])
{
    UNITY_BEGIN();
    RUN_TEST(test_stairs_count_64);
    RUN_TEST(test_steps_per_stair_pow2);
    RUN_TEST(test_acceleration_intervals);
    UNITY_END();
    return 0;
}
