#pragma once

#include <Arduino.h>
#include "Pin.h"

#define RAMP_STAIRS 64

typedef void (*StepperCallback)(void);

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
class Stepper
{
public:
    typedef AccelerationRamp<RAMP_STAIRS, INTERRUPT::FREQ, DRIVER::SPR, (uint32_t)(DEG_TO_RAD *MAX_SPEED * 100), (uint32_t)(DEG_TO_RAD *ACCELERATION * 100)> Ramp;

    static constexpr Ramp ramp = Ramp();
    static uint8_t ramp_stair;
    static uint8_t ramp_stair_step;

    static uint32_t run_interval;

    static uint32_t movement_step;
    static uint32_t movement_accel_end;
    static uint32_t movement_decel_start;

    static StepperCallback cb_complete;

    /**
     * @brief Static class. We don't need constructor.
     */
    Stepper() = delete;

public:
    /**
     * @brief Stop movement immediately and invoke onComplete interval if set.
     */
    static inline void stop()
    {
        INTERRUPT::stop();
        INTERRUPT::setCallback(nullptr);

        ramp_stair = 0;
        ramp_stair_step = 0;

        movement_step = 0;
        movement_accel_end = UINT32_MAX;
        movement_decel_start = UINT32_MAX;
        run_interval = UINT32_MAX;

        if (cb_complete != nullptr)
        {
            cb_complete();
            cb_complete = nullptr;
        }
    }

    static void accelerate_handler()
    {
        // always step first to ensure best accuracy.
        // other calculations should be done as quick as possible below.
        DRIVER::step();

        // increase current movement step index
        movement_step++;

        // should decelerate immediately without getting into the RUN phase
        if (movement_step == movement_decel_start)
        {
            // start deceleration
            INTERRUPT::setCallback(decelerate_handler);
        }
        // did not reach end of ramp step yet, continue stepping at same rate
        else if (ramp_stair_step < Ramp::STEPS_PER_STAIR - 1)
        {
            ramp_stair_step++;
        }
        // reached end of the stair, but not end of the ramp
        else if (ramp_stair < Ramp::STAIRS_COUNT - 1)
        {
            uint32_t next_ramp_interval = ramp.intervals[++ramp_stair];

            // reached target speed before ramp end
            if (run_interval > next_ramp_interval)
            {
                INTERRUPT::setInterval(run_interval);
                INTERRUPT::setCallback(run_handler);

                movement_decel_start -= movement_step;
                ramp_stair--;
            }
            // switch to next ramp stair
            else
            {
                INTERRUPT::setInterval(next_ramp_interval);
                ramp_stair_step = 0;
            }
        }
        // reached end of the ramp
        else
        {
            // switch to run phase
            INTERRUPT::setInterval(run_interval);
            INTERRUPT::setCallback(run_handler);

            ramp_stair = Ramp::STAIRS_COUNT - 1;
            ramp_stair_step = Ramp::STEPS_PER_STAIR - 1;
        }
    }

    static void run_handler()
    {
        DRIVER::step();

        if (++movement_step == movement_decel_start)
        {
            if (ramp_stair > 0 && ramp_stair_step > 0)
            {
                INTERRUPT::setInterval(ramp.intervals[ramp_stair]);
                INTERRUPT::setCallback(decelerate_handler);
            }
            else
            {
                stop();
            }
        }
    }

    static void decelerate_handler()
    {
        DRIVER::step();
        movement_step++;

        // did not reach end of the ramp stair
        if (ramp_stair_step > 0)
        {
            ramp_stair_step--;
        }
        // did not reach last deceleration ramp stair
        else if (ramp_stair > 0)
        {
            ramp_stair--;
            ramp_stair_step = Ramp::STEPS_PER_STAIR - 1;

            INTERRUPT::setInterval(ramp.intervals[ramp_stair]);
        }
        // reached end of the decel ramp, stop
        else
        {
            stop();
        }
    }

    struct MovementSpec
    {
        const uint32_t steps;
        const uint32_t run_interval;
        const uint8_t full_accel_stairs;

        constexpr MovementSpec(const float degPerSec, const uint32_t steps = UINT32_MAX)
            : steps(steps),
              run_interval(Ramp::getIntervalForSpeed(degPerSec)),
              full_accel_stairs(Ramp::maxAccelStairs(degPerSec))
        {
            // nothing to do here, all values have been initialized
        }
    };

    static inline void move(float degPerSec, uint32_t steps = UINT32_MAX, StepperCallback onComplete = nullptr)
    {
        move(MovementSpec(degPerSec, steps));
    }

    static void move(MovementSpec spec, StepperCallback onComplete = nullptr)
    {
        Pin<52>::high();

        // we disable the interrupt callback
        INTERRUPT::setCallback(nullptr);

        cb_complete = onComplete;

        // Set target speed/interval
        run_interval = spec.run_interval;
        uint8_t accel_stairs_delta = spec.full_accel_stairs - ramp_stair;

        // if (accel_stairs_delta > 0)
        // {
        //     noInterrupts();
        //     movement_step = 0;
        //     movement_decel_start = steps - ((full_accel_stairs - accel_stairs_delta) * Ramp::STEPS_PER_STAIR);

        //     INTERRUPT::setCallback(accelerate_handler);
        //     INTERRUPT::setInterval(ramp.intervals[ramp_stair + 1]);
        //     interrupts();
        //     return;
        // }

        // no acceleration required
        if (accel_stairs_delta == 0)
        {
            movement_decel_start = spec.steps;

            INTERRUPT::setInterval(run_interval);
            INTERRUPT::setCallback(run_handler);
        }
        // acceleration required
        else
        {
            // speed is lower than max speed
            if (run_interval > ramp.intervals[Ramp::STAIRS_COUNT - 1])
            {
                // movement to short, RUN should be skipped
                if (spec.steps < spec.full_accel_stairs * 2 * Ramp::STEPS_PER_STAIR)
                {
                    movement_decel_start = spec.steps / 2;
                }
                else
                {
                    movement_decel_start = spec.steps;
                }
            }
            // if overal movement is shorter than a complete (accel+decel) ramp,
            // deceleration has to start earlier (on the half way to the target position).
            // Otherwise deceleration start point is (steps - deceleration steps)
            else if (spec.steps < (Ramp::STEPS_COUNT * 2))
            {
                movement_decel_start = spec.steps / 2;
            }
            else
            {
                movement_decel_start = spec.steps - (Ramp::STEPS_PER_STAIR * Ramp::STAIRS_COUNT);
            }

            INTERRUPT::setCallback(accelerate_handler);
            INTERRUPT::setInterval(ramp.intervals[0]);
        }

        Pin<52>::low();
    }
};

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint8_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::ramp_stair = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint8_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::ramp_stair_step = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::run_interval = UINT32_MAX;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::movement_step = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::movement_accel_end = UINT32_MAX;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::movement_decel_start = UINT32_MAX;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
StepperCallback Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::cb_complete = nullptr;