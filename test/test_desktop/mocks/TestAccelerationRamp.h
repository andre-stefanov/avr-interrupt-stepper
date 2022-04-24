#pragma once

#include <stdint.h>
#include <math.h>
#include "AccelerationRamp.h"

template <typename RAMP>
struct TestIntervals
{

    static bool called;
    static int64_t min_index;
    static int64_t max_index;

    constexpr TestIntervals() {}

    constexpr const uint32_t &operator[](const unsigned int i)
    {
        called = true;
        min_index = (i < min_index) ? i : min_index;
        max_index = (i > max_index) ? i : max_index;
        return RAMP::intervals[i];
    }

    constexpr const uint32_t &operator[](const unsigned int i) const
    {
        called = true;
        min_index = (i < min_index) ? i : min_index;
        max_index = (i > max_index) ? i : max_index;
        return RAMP::intervals[i];
    }
};

template <typename RAMP>
bool TestIntervals<RAMP>::called = false;

template <typename RAMP>
int64_t TestIntervals<RAMP>::min_index = INT64_MAX;

template <typename RAMP>
int64_t TestIntervals<RAMP>::max_index = INT64_MIN;

template <uint8_t STAIRS, uint32_t T_FREQ, uint32_t SPR, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
struct TestAccelerationRamp
{
    using REAL_TYPE = AccelerationRamp<STAIRS, T_FREQ, SPR, MAX_SPEED_mRAD, ACCELERATION_mRAD>;

    constexpr static uint8_t STAIRS_COUNT = REAL_TYPE::STAIRS_COUNT;
    constexpr static uint8_t STEPS_PER_STAIR = REAL_TYPE::STEPS_PER_STAIR;
    constexpr static uint8_t FIRST_STEP = REAL_TYPE::FIRST_STEP;
    constexpr static uint8_t LAST_STEP = REAL_TYPE::LAST_STEP;

    constexpr static TestIntervals intervals = TestIntervals<REAL_TYPE>();

    static constexpr inline __attribute__((always_inline)) uint32_t getIntervalForSpeed(float radPerSec)
    {
        return REAL_TYPE::getIntervalForSpeed(radPerSec);
    }

    static constexpr inline __attribute__((always_inline)) uint8_t maxAccelStairs(float radPerSec)
    {
        return REAL_TYPE::maxAccelStairs(radPerSec);
    }

    static void reset()
    {
        intervals.called = false;
        intervals.min_index = INT64_MAX;
        intervals.max_index = INT64_MIN;
    }
};
