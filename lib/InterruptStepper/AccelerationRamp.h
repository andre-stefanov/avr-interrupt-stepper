#ifndef INTERRUPTSTEPPER_RAMP_H
#define INTERRUPTSTEPPER_RAMP_H

// #include "Arduino.h"
#include "NewtonRaphson.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// #ifndef RAD
// #define RAD(DEG) (DEG * 0.017453292519943295769236907684886)
// #endif

#ifndef sq
#define sq(x) ((x) * (x))
#endif

#define ALPHA(SPR) (2 * M_PI / SPR)
#define ALPHA_2(SPR) (2 * ALPHA(SPR))
#define ALPHA_T(SPR, FREQ) (FREQ * ALPHA(SPR))

/// @brief Helper for acceleration ramp calculations based on AVR466.
/// The calculations happen at compile time as far as possible.
///
/// @tparam STAIRS amount of speed stairs to be used during acceleration and/or deceleration
/// @tparam T_FREQ frequency of the used timer in Hz
/// @tparam STEPPER_SPR stepper steps per revolution (FULL_STEP_SPR * MICROSTEPPING)
/// @tparam MAX_SPEED maximal possible speed in 100*rad/s
/// @tparam ACCELERATION maximal possible speed in 100*rad/s/s
///
template <uint8_t STAIRS, uint32_t T_FREQ, uint32_t STEPPER_SPR, uint32_t MAX_SPEED, uint32_t ACCELERATION>
class AccelerationRamp
{
private:
    constexpr static auto maxSpeed = MAX_SPEED / 100.0;
    constexpr static auto acceleration = ACCELERATION / 100.0;

    constexpr static auto MAX_ACCEL_STEPS = (uint32_t)(sq(maxSpeed) / (ALPHA_2(STEPPER_SPR) * acceleration));

    constexpr static uint8_t roundPow2(uint8_t value)
    {
        for (size_t i = 1; i <= 256; i *= 2)
        {
            uint8_t a = i / 2;
            uint8_t b = i;

            if (value > a && value < b)
            {
                uint8_t diffA = value - a;
                uint8_t diffB = value - b;
                return (diffA < diffB) ? a : b;
            }
        }
        return 1;
    }

public:
    constexpr static auto MIN_INTERVAL = (uint32_t)(ALPHA_T(STEPPER_SPR, T_FREQ) / maxSpeed);

    static_assert(MIN_INTERVAL > 0);

    constexpr static auto STAIRS_COUNT = STAIRS;

    static_assert(STAIRS_COUNT > 0);

    constexpr static auto STEPS_PER_STAIR = roundPow2(MAX_ACCEL_STEPS / STAIRS);

    static_assert(STEPS_PER_STAIR > 0);
    static_assert((STEPS_PER_STAIR & (STEPS_PER_STAIR - 1)) == 0, "Steps per stair has to be power of 2");

    constexpr static auto STEPS_COUNT = STAIRS_COUNT * STEPS_PER_STAIR;

    uint32_t intervals[STAIRS];

    constexpr AccelerationRamp() : intervals()
    {
        intervals[0] = (uint32_t)(T_FREQ * NewtonRaphson::sqrt(ALPHA_2(STEPPER_SPR) / (acceleration * STEPS_PER_STAIR)));
        for (uint16_t i = 1; i < STAIRS; ++i)
        {
            intervals[i] = (uint32_t)(intervals[0] * (NewtonRaphson::sqrt(i + 1) - NewtonRaphson::sqrt(i)));
        }
    }

    static constexpr inline uint32_t getIntervalForSpeed(const float radPerSec)
    {
        return (uint32_t)(ALPHA_T(STEPPER_SPR, T_FREQ) / radPerSec + 0.5);
    }

    static const inline uint8_t maxAccelStairs(const float radPerSec)
    {
        return (uint8_t)(sq(radPerSec) / (ALPHA_2(STEPPER_SPR) * acceleration) + 0.5);
    }
};

#endif //INTERRUPTSTEPPER_RAMP_H
