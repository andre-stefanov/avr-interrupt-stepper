#ifndef INTERRUPTSTEPPER_RAMP_H
#define INTERRUPTSTEPPER_RAMP_H

#include "Arduino.h"
#include "NewtonRaphson.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#ifndef RAD
#define RAD(DEG) (DEG * 0.017453292519943295769236907684886)
#endif

#ifndef sq
#define sq(x) ((x) * (x))
#endif

#define ALPHA(SPR) (2 * M_PI / SPR)
#define ALPHA_2(SPR) (2 * ALPHA(SPR))
#define ALPHA_T(SPR, FREQ) (FREQ * ALPHA(SPR))

template <uint8_t STAIRS, uint32_t T_FREQ, uint32_t STEPPER_SPR, uint32_t MAX_SPEED, uint32_t ACCELERATION>
class AccelerationRamp
{

private:
    constexpr static auto MAX_ACCEL_STEPS = (uint32_t)(sq(RAD(MAX_SPEED)) / (ALPHA_2(STEPPER_SPR) * RAD(ACCELERATION)));

public:
    constexpr static auto STEPS_PER_STAIR = (uint32_t)(MAX_ACCEL_STEPS / STAIRS);

    constexpr static auto MIN_INTERVAL = (uint32_t)(ALPHA_T(STEPPER_SPR, T_FREQ) / RAD(MAX_SPEED));

    constexpr static auto STAIRS_COUNT = STAIRS;

    constexpr static auto STEPS_COUNT = STAIRS_COUNT * STEPS_PER_STAIR;

    uint32_t intervals[STAIRS];

    constexpr AccelerationRamp() : intervals()
    {
        intervals[0] = (uint32_t)(T_FREQ * NewtonRaphson::sqrt(ALPHA_2(STEPPER_SPR) / RAD(ACCELERATION * STEPS_PER_STAIR)));
        for (uint16_t i = 1; i < STAIRS; ++i)
        {
            intervals[i] = (uint32_t)(intervals[0] * (NewtonRaphson::sqrt(i + 1) - NewtonRaphson::sqrt(i)));
        }
    }

    static const inline uint32_t getIntervalForSpeed(const float degPerSec)
    {
        return (uint32_t)(ALPHA_T(STEPPER_SPR, T_FREQ) / RAD(degPerSec) + 0.5);
    }

    static const inline uint32_t maxAccelSteps(const float degPerSec)
    {
        return (uint32_t)(sq(RAD(degPerSec)) / (ALPHA_2(STEPPER_SPR) * RAD(ACCELERATION)) + 0.5);
    }
};

#endif //INTERRUPTSTEPPER_RAMP_H
