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
#include "etl/type_def.h"
#include <stdint.h>
#include "Angle.h"

#define RAMP_STAIRS 64

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
    using Ramp = AccelerationRamp<RAMP_STAIRS, INTERRUPT::FREQ, DRIVER::SPR, MAX_SPEED_mRAD, ACCELERATION_mRAD>;

private:
    static volatile bool dir;
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

    static void start_movement(
        const bool direction,
        const uint8_t pre_decel_stairs,
        const uint16_t accel_steps,
        const uint32_t run_steps,
        const uint32_t run_interval)
    {
        using Class = Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>;

        Class::pre_decel_stairs_left = pre_decel_stairs;
        Class::accel_steps_left = accel_steps;
        Class::run_steps_left = run_steps;
        Class::run_interval = run_interval;

        if (pre_decel_stairs > 0)
        {
            Class::ramp_stair_step = Ramp::LAST_STEP;
            INTERRUPT::setInterval(Ramp::intervals[Class::ramp_stair]);
            INTERRUPT::setCallback(pre_decelerate_handler);
        }
        else if (accel_steps > 0)
        {
            Class::dir = direction;
            DRIVER::dir(dir);
            INTERRUPT::setInterval(Ramp::intervals[Class::ramp_stair]);
            INTERRUPT::setCallback(accelerate_handler);
        }
        else
        {
            Class::dir = direction;
            DRIVER::dir(dir);
            INTERRUPT::setInterval(run_interval);
            INTERRUPT::setCallback(run_handler);
        }
    }

    static void pre_decelerate_handler()
    {
        DRIVER::step();

        if (ramp_stair_step > 0)
        {
            ramp_stair_step--;
        }
        else if (pre_decel_stairs_left > 0)
        {
            pre_decel_stairs_left--;
            ramp_stair--;
            ramp_stair_step = Ramp::LAST_STEP;
            INTERRUPT::setInterval(Ramp::intervals[ramp_stair]);
        }
        else if (accel_steps_left > 0)
        {
            dir = !dir;
            DRIVER::dir(dir);
            ramp_stair = 0;
            ramp_stair_step = 0;
            INTERRUPT::setInterval(Ramp::intervals[0]);
            INTERRUPT::setCallback(accelerate_handler);
        }
        else
        {
            DRIVER::dir(dir);
            INTERRUPT::setInterval(run_interval);
            INTERRUPT::setCallback(run_handler);
        }
    }

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
        else if (ramp_stair_step < Ramp::LAST_STEP)
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
            ramp_stair_step = Ramp::LAST_STEP;
            INTERRUPT::setInterval(Ramp::intervals[ramp_stair]);
        }
        else
        {
            terminate();
        }
    }

public:
    struct MovementSpec
    {
        const bool dir;
        const uint32_t steps;
        const uint32_t run_interval;
        const uint8_t full_accel_stairs;

        constexpr MovementSpec(const Angle speed, const Angle distance)
            : dir(distance.rad() > 0),
              steps(abs(distance.rad()) / (2 * 3.14159265358979323846 / DRIVER::SPR)),
              run_interval(Ramp::getIntervalForSpeed(speed.rad())),
              full_accel_stairs(Ramp::maxAccelStairs(speed.rad()))
        {
            // nothing to do here, all values have been initialized
        }
    };

    static void move(const Angle speed, const Angle distance, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, distance), onComplete);
    }

    static void move(MovementSpec spec, StepperCallback onComplete = StepperCallback())
    {
        PROFILE_MOVE_BEGIN();

        cb_complete = onComplete;

        // if direction differs from current one, stop decelerated and move into other direction
        // add steps needed for deceleration to the new movement to correct position
        if (spec.dir != dir)
        {
            const uint16_t accel_steps = static_cast<uint16_t>(spec.full_accel_stairs) * static_cast<uint16_t>(Ramp::STEPS_PER_STAIR);
            const uint32_t run_steps = spec.steps + (static_cast<uint16_t>(ramp_stair) * static_cast<uint16_t>(Ramp::STEPS_PER_STAIR));
            start_movement(spec.dir, ramp_stair, accel_steps, run_steps, spec.run_interval);
        }
        else
        {
            // run at requested speed directly
            if (spec.full_accel_stairs == ramp_stair)
            {
                start_movement(spec.dir, 0, 0, spec.steps, spec.run_interval);
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

                // steps which will be needed for acceleration and deceleration
                const uint32_t accel_decel_steps = (spec.full_accel_stairs + accel_stairs) * Ramp::STEPS_PER_STAIR;

                // movement to short, wont reach full speed
                if (accel_decel_steps >= spec.steps)
                {
                    Pin<53>::pulse();
                    start_movement(
                        spec.dir,
                        0,
                        spec.steps / 2,
                        0,
                        Ramp::intervals[ramp_stair]);
                }
                else // perform a full accel + run + decel ramp
                {
                    start_movement(
                        spec.dir,
                        0,
                        static_cast<uint16_t>(accel_stairs) * static_cast<uint16_t>(Ramp::STEPS_PER_STAIR),
                        spec.steps - accel_decel_steps,
                        spec.run_interval);
                }
            }
            // decelerate to requested speed
            else
            {
                // if distance is shorter than full deceleration, need to correct in opposite direction
                uint16_t full_decel_steps = static_cast<uint16_t>(ramp_stair) * static_cast<uint16_t>(Ramp::STEPS_PER_STAIR);
                if (spec.steps < full_decel_steps)
                {
                    start_movement(
                        !spec.dir,
                        ramp_stair,
                        (full_decel_steps - spec.steps) / 2,
                        0,
                        spec.run_interval);
                }
                else // decelerate to desired speed and later decelerate to stop
                {
                    const uint16_t decel_stairs = ramp_stair - spec.full_accel_stairs;
                    start_movement(
                        spec.dir,
                        decel_stairs,
                        0,
                        spec.steps - decel_stairs - (static_cast<uint16_t>(spec.full_accel_stairs) * static_cast<uint16_t>(Ramp::STEPS_PER_STAIR)),
                        spec.run_interval);
                }
            }
        }

        PROFILE_MOVE_END();
    }
};

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED_mRAD, uint32_t ACCELERATION_mRAD>
bool volatile Stepper<INTERRUPT, DRIVER, MAX_SPEED_mRAD, ACCELERATION_mRAD>::dir = true;

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