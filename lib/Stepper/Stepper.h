#pragma once

#define PROFILE_STEPPER 1
#if PROFILE_STEPPER && defined(ARDUINO)
#include "Pin.h"
#define PROFILE_MOVE_BEGIN() Pin<52>::high()
#define PROFILE_MOVE_END() Pin<52>::low()
#else
#define PROFILE_MOVE_BEGIN()
#define PROFILE_MOVE_END()
#endif

#include "AccelerationRamp.h"
#include "etl/delegate.h"
#include "utils.h"
#include <stdint.h>

#define RAMP_STAIRS 128

using StepperCallback = etl::delegate<void()>;

/**
 * @brief 
 * 
 * @tparam INTERRUPT 
 * @tparam DRIVER 
 * @tparam MAX_SPEED_mRAD maximal possible speed in mrad/s 
 * @tparam ACCELERATION_mRAD maximal possible speed in mrad/s/s
 */
template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
class Stepper
{
public:
    typedef AccelerationRamp<RAMP_STAIRS, INTERRUPT::FREQ, DRIVER::SPR, MAX_SPEED_mRAD, ACCELERATION_mRAD> Ramp;

private:
    static volatile uint8_t ramp_stair;
    static volatile uint8_t ramp_stair_step;

    static uint32_t run_interval;

    static volatile uint8_t pre_decel_stairs_left;
    static volatile uint16_t accel_steps_left;
    static volatile uint32_t run_steps_left;

    static StepperCallback cb_complete;

    /**
     * @brief Static class. We don't need constructor.
     */
    Stepper() = delete;

    /**
     * @brief Stop movement immediately and invoke onComplete interval if set.
     */
    static inline void terminate()
    {
        INTERRUPT::stop();
        INTERRUPT::setCallback(nullptr);

        ramp_stair = 0;
        ramp_stair_step = 0;

        if (cb_complete.is_valid())
        {
            cb_complete();
        }
    }

    static inline void startMovement(
        const uint8_t pre_decel_stairs,
        const uint16_t accel_steps,
        const uint32_t run_steps,
        const uint32_t initial_interval,
        timer_callback initial_handler)
    {
        pre_decel_stairs_left = pre_decel_stairs;
        accel_steps_left = accel_steps;
        run_steps_left = run_steps;

        INTERRUPT::setInterval(initial_interval);
        INTERRUPT::setCallback(initial_handler);
    }

public:
    static void accelerate_handler()
    {
        // always step first to ensure best accuracy.
        // other calculations should be done as quick as possible below.
        DRIVER::step();

        accel_steps_left--;

        // reached max amount of acceleration steps
        if (accel_steps_left == 0)
        {
            if (run_steps_left == 0)
            {
                INTERRUPT::setCallback(decelerate_handler);
            }
            else
            {
                INTERRUPT::setInterval(run_interval);
                INTERRUPT::setCallback(run_handler);
            }
        }
        // did not finish current stair yet
        else if (ramp_stair_step < Ramp::STEPS_PER_STAIR - 1)
        {
            ramp_stair_step++;
        }
        // acceleration not finished, switch to next speed
        else
        {
            INTERRUPT::setInterval(Ramp::intervals[++ramp_stair]);
            ramp_stair_step = 0;
        }
    }

    static void run_handler()
    {
        DRIVER::step();

        if (--run_steps_left == 0)
        {
            if (ramp_stair == 0)
            {
                terminate();
            }
            else
            {
                INTERRUPT::setInterval(Ramp::intervals[ramp_stair]);
                INTERRUPT::setCallback(decelerate_handler);
            }
        }
    }

    static void decelerate_handler()
    {
        DRIVER::step();

        if (ramp_stair_step > 0)
        {
            ramp_stair_step--;
        }
        else if (ramp_stair > 0)
        {
            ramp_stair--;
            ramp_stair_step = Ramp::STEPS_PER_STAIR - 1;
            INTERRUPT::setInterval(Ramp::intervals[ramp_stair]);
        }
        else
        {
            terminate();
        }
    }

    struct MovementSpec
    {
        const uint32_t steps;
        const uint32_t run_interval;
        const uint8_t full_accel_stairs;

        constexpr MovementSpec(const float radPerSec, const uint32_t steps = UINT32_MAX)
            : steps(steps),
              run_interval(Ramp::getIntervalForSpeed(radPerSec)),
              full_accel_stairs(Ramp::maxAccelStairs(radPerSec))
        {
            // nothing to do here, all values have been initialized
        }
    };

    static void move(float radPerSec, uint32_t steps, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(radPerSec, steps));
    }

    static void move(MovementSpec spec, StepperCallback onComplete = StepperCallback())
    {
        PROFILE_MOVE_BEGIN();

        cb_complete = onComplete;

        // Set target speed/interval
        run_interval = spec.run_interval;

        bool reverseDir = false;
        if (reverseDir)
        {
        }
        else
        {
            // run at requested speed directly
            if (spec.full_accel_stairs == ramp_stair)
            {
                startMovement(0, 0, spec.steps, spec.run_interval, run_handler);
            }
            // accelerate to requested speed
            if (spec.full_accel_stairs > ramp_stair)
            {
                // steps which will be needed for acceleration
                uint8_t accel_stairs = spec.full_accel_stairs - ramp_stair;

                if (accel_steps_left == 0 && run_steps_left > 0)
                {
                    accel_stairs--;
                }

                uint16_t accel_steps = accel_stairs * Ramp::STEPS_PER_STAIR;

                // steps which will be needed for acceleration and deceleration
                uint32_t accel_decel_steps = (spec.full_accel_stairs + accel_stairs) * Ramp::STEPS_PER_STAIR;

                // movement to short, wont reach full speed
                if (accel_decel_steps >= spec.steps)
                {
                    startMovement(
                        0,
                        spec.steps / 2,
                        0,
                        Ramp::intervals[ramp_stair],
                        accelerate_handler);
                }
                else
                {
                    startMovement(
                        0,
                        accel_steps,
                        spec.steps - accel_decel_steps,
                        Ramp::intervals[ramp_stair],
                        accelerate_handler);
                }
            }
            // decelerate to requested speed
            else
            {
            }
        }

        PROFILE_MOVE_END();
    }
};

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
uint8_t volatile Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::ramp_stair = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
uint8_t volatile Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::ramp_stair_step = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::run_interval = UINT32_MAX;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
StepperCallback Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::cb_complete = StepperCallback();

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
uint8_t volatile Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::pre_decel_stairs_left = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
uint16_t volatile Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::accel_steps_left = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
uint32_t volatile Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::run_steps_left = 0;