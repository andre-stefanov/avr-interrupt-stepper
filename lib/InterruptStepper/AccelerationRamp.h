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

#define SQ(x) (x * x)

#define ALPHA(SPR)          (2 * M_PI / SPR)
#define ALPHA_2(SPR)        (2 * ALPHA(SPR))
#define ALPHA_T(SPR, FREQ)  (FREQ * ALPHA(SPR))

template<uint8_t STAIRS, uint32_t T_FREQ, uint32_t STEPPER_SPR, uint32_t MAX_SPEED, uint32_t ACCELERATION>
struct AccelerationRamp {

    constexpr static auto MAX_ACCEL_STEPS = (uint32_t) (
            SQ(RAD(MAX_SPEED)) /
            (ALPHA_2(STEPPER_SPR) * RAD(ACCELERATION))
    );

    constexpr static auto STEPS_PER_STAIR = (uint32_t) (
            MAX_ACCEL_STEPS / STAIRS
    );

    constexpr static auto MIN_COUNTER = (uint32_t) (
            ALPHA_T(STEPPER_SPR, T_FREQ) / RAD(MAX_SPEED)
    );

    constexpr static auto STAIRS_COUNT = STAIRS;

    uint32_t counters[STAIRS];

    constexpr AccelerationRamp() : counters() {
        counters[0] = (uint32_t)(T_FREQ * NewtonRaphson::sqrt(ALPHA_2(STEPPER_SPR) / RAD(ACCELERATION * STEPS_PER_STAIR)));
        for (uint16_t i = 1; i < STAIRS; ++i) {
            counters[i] = (uint32_t)(counters[0] * (NewtonRaphson::sqrt(i + 1) - NewtonRaphson::sqrt(i)));
        }
    }

    static inline uint32_t getCounterForSpeed(float speed)
    {
        return (uint32_t)(ALPHA_T(STEPPER_SPR, T_FREQ) / RAD(speed) + 0.5);
    }
};

#endif //INTERRUPTSTEPPER_RAMP_H
