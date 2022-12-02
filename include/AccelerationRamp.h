#ifndef ACCELERATION_RAMP_H
#define ACCELERATION_RAMP_H

#include <stdint.h> // NOLINT(modernize-deprecated-headers)
#include <math.h> // NOLINT(modernize-deprecated-headers)
#include "NewtonRaphson.h"

template<uint16_t N>
struct Intervals {
    uint32_t data[N];

    constexpr inline __attribute__((always_inline)) uint32_t &operator[](const unsigned int i) {
        return data[i];
    }

    constexpr inline __attribute__((always_inline)) const uint32_t &operator[](const unsigned int i) const {
        return data[i];
    }
};

/// @brief Helper for acceleration ramp calculations based on AVR466.
/// The calculations happen at compile time as far as possible.
///
/// @tparam STAIRS amount of speed stairs to be used during acceleration and/or deceleration
/// @tparam T_FREQ frequency of the used timer in Hz
/// @tparam SPR stepper steps per revolution (incl. microstepping)
/// @tparam MAX_SPEED_mRAD maximal possible speed in mrad/s
/// @tparam ACCELERATION_mRAD maximal possible speed in mrad/s/s
///
template<uint16_t STAIRS, uint32_t T_FREQ, uint32_t MAX_SPEED, uint32_t ACCELERATION>
class AccelerationRamp {
    template<typename T>
    constexpr static inline __attribute__((always_inline)) bool is_pow2(const T value) {
        return (value & (value - 1)) == 0;
    }

    static_assert(STAIRS > 0, "Amount of stairs has to be at least 1");
    static_assert(STAIRS <= UINT16_MAX / 2, "Amount of stairs has to be at most 2^15");
    static_assert(is_pow2(STAIRS), "Amount of stairs has to be power of 2");

    static_assert(T_FREQ > 0, "Timer frequency has to be greater than zero");

    static_assert(MAX_SPEED > 0, "Max speed has to be greater than zero");

    static_assert(ACCELERATION > 0, "Acceleration has to be greater than zero");

    template<typename T>
    constexpr static inline float f(T value)
    {
        return static_cast<float>(value);
    }

    constexpr static uint32_t MAX_STEPS_IDEAL = static_cast<uint32_t>(f(MAX_SPEED) * f(MAX_SPEED) / (2.0f * f(ACCELERATION)));
    constexpr static uint32_t ACCELERATION_UTIL = ACCELERATION * MAX_STEPS_IDEAL / STAIRS;

    constexpr static uint8_t floor_pow2_u8(const uint8_t value) {
        for (unsigned int i = 1; i < 256; i *= 2) {
            if (value >= i && value < i * 2) {
                return i;
            }
        }
        return UINT8_MAX;
    }

    constexpr static Intervals<STAIRS> calculateIntervals() {
        Intervals<STAIRS> result = {};
        constexpr float c0 = T_FREQ * NewtonRaphson::sqrt(2.0f / ACCELERATION_UTIL);
        result[0] = UINT32_MAX;
        for (uint16_t i = 1; i < STAIRS; ++i) {
            result[i] = (uint32_t) (c0 * (NewtonRaphson::sqrt((float) i + 1) - NewtonRaphson::sqrt((float) i)));
        }
        return result;
    }

    constexpr static uint32_t STEPS_PER_STAIR_IDEAL = MAX_STEPS_IDEAL / STAIRS;
    static_assert(STEPS_PER_STAIR_IDEAL <= UINT8_MAX);

public:
    AccelerationRamp() = delete;

    constexpr static Intervals<STAIRS> intervals = calculateIntervals();
    static_assert(intervals[0] > 0);

    constexpr static uint16_t STAIRS_COUNT = STAIRS;

    constexpr static uint8_t STEPS_PER_STAIR = floor_pow2_u8((uint8_t) STEPS_PER_STAIR_IDEAL);

    constexpr static uint32_t STEPS_TOTAL = static_cast<uint32_t>(STAIRS_COUNT - 1) * STEPS_PER_STAIR;

    static_assert(STEPS_PER_STAIR > 0, "Amount of steps per stair has to be greater than zero");
    static_assert(STEPS_PER_STAIR <= 128, "Amount of steps per stair has to be at most 128");
    static_assert(is_pow2(STEPS_PER_STAIR), "Amount of steps per stair has to be power of 2");

    static constexpr inline __attribute__((always_inline)) uint32_t interval(const uint16_t stair) {
        return intervals[stair];
    }

    static constexpr inline __attribute__((always_inline)) uint32_t getIntervalForSpeed(const float sps) {
        return static_cast<uint32_t>(T_FREQ / abs(sps));
    }

    static constexpr inline __attribute__((always_inline)) uint16_t maxAccelStairs(const float sps) {
        if (abs(sps) >= MAX_SPEED) {
            return STAIRS - 1;
        } else {
            return static_cast<uint16_t>(sps * sps / (2 * ACCELERATION_UTIL));
        }
    }
};

template<uint32_t T_FREQ>
class ConstantRamp {
public:

    constexpr static Intervals<0> intervals = {};

    constexpr static uint16_t STAIRS_COUNT = 0;

    constexpr static uint8_t STEPS_PER_STAIR = 0;

    constexpr static uint32_t STEPS_TOTAL = 0;

    static constexpr inline __attribute__((always_inline)) uint32_t interval(const uint16_t stair) {
        return 0;
    }

    static constexpr inline __attribute__((always_inline)) uint32_t getIntervalForSpeed(const float sps) {
        return static_cast<uint32_t>(T_FREQ / abs(sps));
    }

    static constexpr inline __attribute__((always_inline)) uint16_t maxAccelStairs(const float sps) {
        return 0;
    }

};

#endif // ACCELERATION_RAMP_H
