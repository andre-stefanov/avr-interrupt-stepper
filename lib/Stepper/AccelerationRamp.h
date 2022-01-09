#ifndef ACCELERATION_RAMP_H
#define ACCELERATION_RAMP_H

#include <stdint.h>
#include "NewtonRaphson.h"

#define ALPHA(SPR) (2 * 3.14159265358979323846f / SPR)
#define ALPHA_2(SPR) (2 * ALPHA(SPR))
#define ALPHA_T(SPR, FREQ) (FREQ * ALPHA(SPR))

template <typename T, uint8_t N>
struct Intervals
{
    T data[N];

    constexpr T &operator[](unsigned int i)
    {
        return data[i];
    }

    constexpr const T &operator[](unsigned int i) const
    {
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
template <uint8_t STAIRS, uint32_t T_FREQ, uint32_t SPR, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
class AccelerationRamp
{
private:
    static_assert(STAIRS > 0, "Amount of stairs has to be at least 1");
    static_assert(STAIRS <= 128, "Amount of stairs has to be at most 128");
    static_assert((STAIRS & (STAIRS - 1)) == 0, "Amount of stairs has to be power of 2");

    static_assert(T_FREQ > 0, "Timer frequency has to be greater than zero");

    static_assert(SPR > 0, "Steps per revolution has to be greater than zero");

    static_assert(MAX_SPEED_mRAD > 0, "Max speed has to be greater than zero");
    
    static_assert(ACCELERATION_mRAD > 0, "Acceleration has to be greater than zero");

    /**
     * @brief Requested max speed in rad/s
     * 
     */
    constexpr static float MAX_SPEED_RAD = MAX_SPEED_mRAD / 1000.0f;

    /**
     * @brief Requested acceleration in rad/s/s
     */
    constexpr static float ACCELERATION_RAD = ACCELERATION_mRAD / 1000.0f;

    /**
     * @brief Maximal amount of steps needed to reach max speed.
     */
    constexpr static float MAX_STEPS_LIM = (MAX_SPEED_RAD * MAX_SPEED_RAD) / (ALPHA_2(SPR) * ACCELERATION_RAD);

    /**
     * @brief Acceleration used for calculations. This will be greater or equal to ACCELERATION_RAD in order to reduce lookup
     * table size.
     */
    constexpr static float UTIL_ACCELERATION_RAD = ACCELERATION_RAD * MAX_STEPS_LIM / STAIRS;

    constexpr static uint8_t toPow2(uint8_t value)
    {
        for (unsigned int i = 1; i <= 256; i *= 2)
        {
            if (value > (i / 2) && value < i)
            {
                return i / 2;
            }
        }
        return 1;
    }

    constexpr static Intervals<uint32_t, STAIRS> calculateIntervals()
    {
        Intervals<uint32_t, STAIRS> result = {};
        result[0] = (uint32_t)(T_FREQ * NewtonRaphson::sqrt(ALPHA_2(SPR) / (UTIL_ACCELERATION_RAD)));
        for (uint16_t i = 1; i < STAIRS; ++i)
        {
            result[i] = (uint32_t)(result[0] * (NewtonRaphson::sqrt(i + 1) - NewtonRaphson::sqrt(i)));
        }
        return result;
    }

    AccelerationRamp() = delete;

public:
    constexpr static uint32_t MIN_INTERVAL = (uint32_t)(ALPHA_T(SPR, T_FREQ) / MAX_SPEED_RAD + 0.5f);
    static_assert(MIN_INTERVAL > 0);

    constexpr static uint8_t STAIRS_COUNT = STAIRS;
    static_assert(STAIRS_COUNT > 0);

    constexpr static uint8_t STEPS_PER_STAIR = toPow2((uint8_t)(UTIL_ACCELERATION_RAD / ACCELERATION_RAD));

    static_assert(STEPS_PER_STAIR > 0, "Amount of steps per stair has to be greater than zero");
    static_assert(STEPS_PER_STAIR <= 128, "Amount of steps per stair has to be at most 128");
    static_assert((STEPS_PER_STAIR & (STEPS_PER_STAIR - 1)) == 0, "Amount of steps per stair has to be power of 2");

    constexpr static uint16_t STEPS_COUNT = static_cast<uint16_t>(STAIRS_COUNT) * static_cast<uint16_t>(STEPS_PER_STAIR);

    constexpr static Intervals<uint32_t, STAIRS> intervals = calculateIntervals();

    static constexpr inline uint32_t getIntervalForSpeed(float radPerSec)
    {
        return (uint32_t)(ALPHA_T(SPR, T_FREQ) / radPerSec + 0.5);
    }

    static constexpr uint8_t maxAccelStairs(float radPerSec)
    {
        return (uint8_t)((radPerSec * radPerSec) / (ALPHA_2(SPR) * UTIL_ACCELERATION_RAD));
    }
};

#endif //ACCELERATION_RAMP_H
