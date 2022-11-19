#pragma once

#define PROFILE_STEPPER 0
#if PROFILE_STEPPER && defined(ARDUINO)
#include "Pin.h"
#define PROFILE_MOVE_BEGIN() Pin<45>::high()
#define PROFILE_MOVE_END() Pin<45>::low()
#else
#define PROFILE_MOVE_BEGIN() do {} while(0)
#define PROFILE_MOVE_END() do {} while(0)
#endif

#ifndef noInterrupts
#define noInterrupts() do {} while(0)
#endif

#ifndef interrupts
#define interrupts() do {} while(0)
#endif

#include "etl/delegate.h"

#include "AccelerationRamp.h"
#include <stdint.h> // NOLINT(modernize-deprecated-headers)
#include <math.h> // NOLINT(modernize-deprecated-headers)
#include "Angle.h"

#define RUN_BLOCK_SIZE 128

using StepperCallback = etl::delegate<void()>;

/**
 * @brief
 *
 * @tparam INTERRUPT
 * @tparam DRIVER
 * @tparam MAX_SPEED_mRAD maximal possible speed in mrad/s
 * @tparam ACCELERATION_mRAD maximal possible speed in mrad/s/s
 */
template <typename INTERRUPT, typename DRIVER, typename RAMP>
class Stepper
{
public:
    /**
     * @brief Static class. We don't need constructor.
     */
    Stepper() = delete;

    static constexpr Angle ANGLE_PER_STEP = Angle::deg(360.0f) / DRIVER::SPR;

private:

    static volatile int32_t pos;

    static volatile int8_t run_dir;
    static volatile int8_t cur_dir;
    static volatile uint16_t ramp_stair;

    static volatile uint32_t run_interval;

    static volatile uint16_t pre_decel_stairs_left;
    static volatile uint16_t accel_stairs_left;
    static volatile uint32_t run_steps_left;
    static volatile uint32_t run_full_blocks_left;
    static volatile uint8_t run_rest_block_steps;

    static volatile uint8_t multi_steps_made;

    static StepperCallback cb_complete;

    static void pre_decelerate_multistep_handler()
    {
        DRIVER::step();

        // check if this was last step of a multistep block
        if (++multi_steps_made == RAMP::STEPS_PER_STAIR)
        {
            pos += (cur_dir > 0) ? RAMP::STEPS_PER_STAIR : -RAMP::STEPS_PER_STAIR;
            multi_steps_made = 0;

            // did not reach end of pre-deceleration, switch to next stair
            if (--pre_decel_stairs_left > 0)
            {
                INTERRUPT::setInterval(RAMP::interval(--ramp_stair));
            }
            // pre-deceleration finished, it was a direction switch, accelerate
            else if (accel_stairs_left > 0)
            {
                ramp_stair = 1;
                cur_dir = run_dir;
                DRIVER::dir(cur_dir > 0);

                INTERRUPT::setCallback(accelerate_multistep_handler);
                INTERRUPT::setInterval(RAMP::interval(1));
            }
            // pre-deceleration finished, no need to accelerate, run
            else
            {
                // set dir in case this deceleration was a direction change with a slow run speed afterwards
                cur_dir = run_dir;
                DRIVER::dir(cur_dir > 0);
                
                if (run_steps_left > 0)
                {
                    INTERRUPT::setCallback(run_slow_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                else if (run_full_blocks_left > 0)
                {
                    INTERRUPT::setCallback(run_full_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                else if (run_rest_block_steps > 0)
                {
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                else
                {
                    terminate();
                }
            }
        }
    }

    static void accelerate_multistep_handler()
    {
        DRIVER::step();

        if (++multi_steps_made == RAMP::STEPS_PER_STAIR) // last step of multistep block
        {
            pos += (cur_dir > 0) ? RAMP::STEPS_PER_STAIR : -RAMP::STEPS_PER_STAIR;
            multi_steps_made = 0;

            // finished acceleration
            if (--accel_stairs_left == 0)
            {
                // switch to run phase (full blocks)
                if (run_full_blocks_left > 0)
                {
                    INTERRUPT::setCallback(run_full_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                // switch to run phase (rest)
                else if (run_rest_block_steps > 0)
                {
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                // decelerate, no run phase needed
                else
                {
                    INTERRUPT::setCallback(decelerate_multistep_handler);
                }
            }
            // continue acceleration
            else
            {
                INTERRUPT::setInterval(RAMP::interval(++ramp_stair));
            }
        }
    }

    static void run_slow_handler()
    {
        DRIVER::step();

        pos += cur_dir;

        if (--run_steps_left == 0)
        {
            terminate();
        }
    }

    static void run_rest_multistep_handler()
    {
        DRIVER::step();

        if (++multi_steps_made == run_rest_block_steps)
        {
            pos += (cur_dir > 0) ? run_rest_block_steps : -run_rest_block_steps;
            run_rest_block_steps = 0;
            multi_steps_made = 0;

            // no deceleration needed
            if (ramp_stair == 0)
            {
                terminate();
            }
            // decelerate
            else
            {
                INTERRUPT::setInterval(RAMP::interval(ramp_stair));
                INTERRUPT::setCallback(decelerate_multistep_handler);
            }
        }
    }

    static void run_full_multistep_handler()
    {
        DRIVER::step();

        if (++multi_steps_made == RUN_BLOCK_SIZE)
        {
            pos += (cur_dir > 0) ? RUN_BLOCK_SIZE : -RUN_BLOCK_SIZE;
            multi_steps_made = 0;

            if (--run_full_blocks_left == 0)
            {
                if (run_rest_block_steps > 0)
                {
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                }
                // no deceleration needed
                else if (ramp_stair == 0)
                {
                    terminate();
                }
                // decelerate
                else
                {
                    INTERRUPT::setInterval(RAMP::interval(ramp_stair));
                    INTERRUPT::setCallback(decelerate_multistep_handler);
                }
            }
            // continue multistep run
            else
            {
                // nothing to do here
            }
        }
    }

    static void decelerate_multistep_handler()
    {
        // always step first to ensure the best accuracy.
        // other calculations should be done as quick as possible below.
        DRIVER::step();

        if (++multi_steps_made == RAMP::STEPS_PER_STAIR)
        {
            pos += (cur_dir > 0) ? RAMP::STEPS_PER_STAIR : -RAMP::STEPS_PER_STAIR;
            multi_steps_made = 0;

            if (--ramp_stair == 0)
            {
                terminate();
            }
            else
            {
                INTERRUPT::setInterval(RAMP::interval(ramp_stair));
            }
        }
    }

public:

    static void init()
    {
        DRIVER::init();
        INTERRUPT::init();
    }

    /**
     * @brief Stop movement immediately and invoke onComplete interval if set.
     */
    static void terminate(bool callCallback = true)
    {
        INTERRUPT::stop();
        INTERRUPT::setCallback(nullptr);

        run_dir = 0;
        cur_dir = 0;
        ramp_stair = 0;

        run_interval = 0;

        pre_decel_stairs_left = 0;
        accel_stairs_left = 0;
        run_steps_left = 0;
        run_full_blocks_left = 0;
        run_rest_block_steps = 0;

        multi_steps_made = 0;

        if (callCallback && cb_complete.is_valid())
        {
            cb_complete();
        }

        cb_complete = StepperCallback();
    }

    static void reset()
    {
        pos = 0;

        cur_dir = 0;
        run_dir = 0;
        ramp_stair = 0;

        run_interval = 0;

        pre_decel_stairs_left = 0;
        accel_stairs_left = 0;
        run_steps_left = 0;
        run_full_blocks_left = 0;
        run_rest_block_steps = 0;

        multi_steps_made = 0;
    }

    static Angle getPositionAngle()
    {
        return Angle::deg(360.0f / DRIVER::SPR) * getPosition();
    }

    static int32_t getPosition()
    {
        noInterrupts();
        const uint32_t cur_pos = pos;
        const uint8_t cur_multistep = multi_steps_made;
        interrupts();

        return cur_pos + (cur_multistep * cur_dir);
    }

    static void setPosition(const int32_t value)
    {
        noInterrupts();
        pos = value;
        interrupts();
    }

    static void setPositionAngle(const Angle &value)
    {
        auto new_pos = static_cast<int32_t>(lround(value / ANGLE_PER_STEP));

        noInterrupts();
        pos = new_pos;
        interrupts();
    }

    static uint32_t distanceToGo()
    {
        noInterrupts();
        const uint16_t pre_decel_stairs = pre_decel_stairs_left;
        const uint16_t accel_stairs = pre_decel_stairs_left;
        const uint32_t run_rest_blocks = run_full_blocks_left;
        const uint8_t run_rest_block = run_rest_block_steps;
        const uint32_t run_steps = run_steps_left;
        const uint16_t cur_ramp_stair = ramp_stair;
        interrupts();

        const uint16_t decel_stairs = cur_ramp_stair - pre_decel_stairs + accel_stairs;

        uint32_t steps_left = 0;
        // pre-decel steps
        steps_left += static_cast<uint32_t>(pre_decel_stairs) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
        // accel steps
        steps_left += static_cast<uint32_t>(accel_stairs) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
        // run blocks steps
        steps_left += static_cast<uint32_t>(run_rest_blocks) * static_cast<uint32_t>(RUN_BLOCK_SIZE);
        // run rest block steps
        steps_left += run_rest_block;
        // run slow steps
        steps_left += run_steps;
        // decel steps
        steps_left += static_cast<uint32_t>(decel_stairs) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);

        return steps_left;
    }

    static bool isRunning()
    {
        return cur_dir != 0;
    }

    struct MovementSpec
    {
        const int32_t steps;
        const uint32_t run_interval;
        const uint16_t accel_stair;

        constexpr MovementSpec(
            const int32_t steps,
            const uint32_t run_interval,
            const uint16_t accel_stair)
            : steps(steps),
              run_interval(run_interval),
              accel_stair(accel_stair) {}

        constexpr MovementSpec(const Angle speed, const Angle distance)
            : steps(speed.rad() > 0 ? static_cast<int32_t>(distance / ANGLE_PER_STEP) : -static_cast<int32_t>(distance / ANGLE_PER_STEP)),
              run_interval(RAMP::getIntervalForSpeed(speed.rad())),
              accel_stair(RAMP::maxAccelStairs(speed.rad())) {}

        constexpr MovementSpec(const Angle speed, const int32_t steps)
            : steps(speed.rad() > 0 ? steps : -steps),
              run_interval(RAMP::getIntervalForSpeed(speed.rad())),
              accel_stair(RAMP::maxAccelStairs(speed.rad())) {}

        constexpr static MovementSpec time(const Angle &speed, const uint32_t time_ms)
        {
            auto steps = static_cast<int32_t>(speed / ANGLE_PER_STEP / 1000 * static_cast<float>(time_ms));
            uint32_t run_interval = RAMP::getIntervalForSpeed(speed.rad());
            uint16_t full_accel_stairs = RAMP::maxAccelStairs(speed.rad());
            return MovementSpec(steps, run_interval, full_accel_stairs);
        }
    };

    static void stop()
    {
        INTERRUPT::stop();

        if (ramp_stair > 0)
        {
            multi_steps_made = 0;

            INTERRUPT::setCallback(decelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
        }
        else
        {
            terminate();
        }
    }

    static void stop(StepperCallback onComplete)
    {
        cb_complete = onComplete;
        stop();
    }

    static void moveTime(const Angle speed, const uint32_t time_ms, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec::time(speed, time_ms), onComplete);
    }

    static void moveTo(const Angle speed, const Angle target, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, target - (Angle::deg(360.0f / DRIVER::SPR) * pos)), onComplete);
    }

    static void moveTo(const Angle speed, const int32_t target, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, target - pos), onComplete);
    }

    static void moveTo(const float stepsPerSecond, const int32_t target, StepperCallback onComplete = StepperCallback())
    {
        moveTo(ANGLE_PER_STEP * stepsPerSecond, target, onComplete);
    }

    static void moveBy(const Angle speed, const Angle distance, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, distance), onComplete);
    }

    static void moveBy(const Angle speed, const uint32_t steps, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, steps), onComplete);
    }

    static void moveBy(const float stepsPerSecond, const uint32_t steps, StepperCallback onComplete = StepperCallback())
    {
        moveBy(ANGLE_PER_STEP * stepsPerSecond, steps, onComplete);
    }

    static void move(MovementSpec spec, StepperCallback onComplete = StepperCallback())
    {
        PROFILE_MOVE_BEGIN();

        // TODO: this should happen as early as possible (e.g. in moveBy)
        INTERRUPT::stop();

        if (cur_dir != 0)
        {
            pos += multi_steps_made * static_cast<int32_t>(cur_dir);
        }

        // reset values describing state of previous movement
        run_interval = 0;
        pre_decel_stairs_left = 0;
        accel_stairs_left = 0;
        run_steps_left = 0;
        run_full_blocks_left = 0;
        run_rest_block_steps = 0;
        multi_steps_made = 0;

        cb_complete = onComplete;

        /**
         * 1. check if target pos (relative) smaller than stop pos
         *   - pre decel
         *   - calculate ramp by adding pre decel steps (reversed direction)
         *   - decide if slow or multistep run
         * 2. check if direct run needed (no accel/decel, same ramp stair)
         *   - decide if slow or multistep run
         * 3. check if target speed higher
         *   - calculate ramp taking current stair into consideration.
         * 4. check if target speed lower
         *   - calculate ramp taking current stair into consideration.
         *   - decide if slow or multistep run
         */

        const auto abs_stop_steps_needed = static_cast<int32_t>(ramp_stair) * static_cast<int32_t>(RAMP::STEPS_PER_STAIR);
        const auto stop_steps_needed = abs_stop_steps_needed * cur_dir;

        // movement target can't be reached even by stopping/decelerating
        // need correction or revert of the direction
        if ((cur_dir * spec.steps) < (cur_dir * stop_steps_needed) && ramp_stair > 0)
        {
            run_dir = -cur_dir;

            pre_decel_stairs_left = ramp_stair;
            uint32_t steps = abs(stop_steps_needed - spec.steps);

            uint32_t accel_steps = spec.accel_stair * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            uint32_t accel_decel_steps = accel_steps * 2;

            // reversed movement needs acceleration
            if (spec.accel_stair > 0)
            {
                uint32_t stairs_possible = steps / RAMP::STEPS_PER_STAIR;
                uint32_t max_stair_possible = stairs_possible / 2;

                // no full ramp possible
                if (spec.accel_stair >= max_stair_possible)
                {
                    accel_stairs_left = max_stair_possible;
                    run_rest_block_steps = steps - (max_stair_possible * (RAMP::STEPS_PER_STAIR * 2));
                    run_interval = RAMP::interval(accel_stairs_left);
                }
                // full ramp possible
                else
                {
                    accel_stairs_left = spec.accel_stair;

                    const auto abs_run_steps = static_cast<uint32_t>(steps - accel_decel_steps);

                    // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
                    run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
                    // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
                    run_rest_block_steps = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

                    run_interval = RAMP::interval(spec.run_interval);
                    // TODO
                }
            }
            // reversed movement slow, run does not need acceleration, but reverse movement does
            else
            {
                run_steps_left = steps;
                run_interval = spec.run_interval;
            }

            INTERRUPT::setCallback(pre_decelerate_multistep_handler);
            INTERRUPT::setInterval(ramp_stair);
        }
        // requested 0 steps and we can stop immediately
        else if (spec.steps == 0)
        {
            // no steps to go, and we don't have to decelerate -> terminate
            terminate();
        }
        // requested speed is similar (on same acceleration ramp stair) as we already are, run directly
        else if (spec.accel_stair == ramp_stair)
        {
            const auto abs_run_steps = static_cast<uint32_t>(abs(spec.steps) - abs_stop_steps_needed);

            if (cur_dir == 0)
            {
                if (spec.steps > 0)
                {
                    cur_dir = 1;
                    run_dir = 1;
                    DRIVER::dir(true);
                }
                else
                {
                    cur_dir = -1;
                    run_dir = -1;
                    DRIVER::dir(false);
                }
            }

            // run directly (slow)
            if (ramp_stair == 0)
            {
                run_steps_left = abs_run_steps;

                INTERRUPT::setCallback(run_slow_handler);
                INTERRUPT::setInterval(spec.run_interval);
            }
            // run directly (fast)
            else
            {
                // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
                run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
                // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
                run_rest_block_steps = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

                if (run_full_blocks_left > 0)
                {
                    INTERRUPT::setCallback(run_full_multistep_handler);
                }
                else
                {
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                }
                INTERRUPT::setInterval(spec.run_interval);
            }
        }
        // requested speed is slower (lower acceleration ramp stair), need to pre-decelerate first then run
        else if (spec.accel_stair < ramp_stair)
        {
            run_interval = spec.run_interval;

            // pre-decelerate, then run (calculate ramp without accel)
            pre_decel_stairs_left = ramp_stair - spec.accel_stair;
            const uint32_t pre_decel_steps = pre_decel_stairs_left * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const uint32_t abs_run_steps = abs(spec.steps) - pre_decel_steps;

            if (spec.accel_stair == 0)
            {
                run_steps_left = abs_run_steps;
            }
            else
            {
                // perform multi steps in run phase
                // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
                run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
                // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
                run_rest_block_steps = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);
            }

            INTERRUPT::setCallback(pre_decelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
        }
        // requested speed is faster (higher acceleration ramp stair), need to accelerate first then run
        else
        {
            if (spec.steps > 0)
            {
                run_dir = 1;
                cur_dir = 1;
                DRIVER::dir(true);
            }
            else
            {
                run_dir = -1;
                cur_dir = -1;
                DRIVER::dir(false);
            }

            // TODO: optimize these calculations

            const auto required_accel_stairs = spec.accel_stair - ramp_stair;

            const auto req_accel_steps = static_cast<uint32_t>(required_accel_stairs) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const auto req_decel_steps = static_cast<uint32_t>(spec.accel_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const auto req_accel_decel_steps = req_accel_steps + req_decel_steps;
            const uint32_t abs_steps = (spec.steps >= 0) ? spec.steps : -spec.steps;

            // full ramp not possible
            if (abs_steps <= req_accel_decel_steps)
            {
                const uint32_t accel_steps_made = static_cast<uint32_t>(ramp_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
                accel_stairs_left = static_cast<uint16_t>((abs_steps - accel_steps_made) / (static_cast<uint32_t>(RAMP::STEPS_PER_STAIR) * 2U));

                if (ramp_stair > 0 && accel_stairs_left > 0)
                {
                    run_interval = RAMP::interval(ramp_stair + accel_stairs_left);
                }
                else
                {
                    run_steps_left = abs_steps;

                    INTERRUPT::setCallback(run_slow_handler);
                    INTERRUPT::setInterval(RAMP::interval(1));

                    return;
                }
            }
            // full ramp
            else
            {
                accel_stairs_left = RAMP::STAIRS_COUNT - 1 - ramp_stair;
                run_interval = spec.run_interval;
            }

            const uint32_t abs_run_steps = abs_steps - ((ramp_stair + accel_stairs_left) * RAMP::STEPS_PER_STAIR * 2);

            // perform multi steps in run phase
            // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
            run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
            // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
            run_rest_block_steps = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

            INTERRUPT::setCallback(accelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(++ramp_stair));
        }
    }
};

template <typename INTERRUPT, typename DRIVER, typename RAMP>
int32_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::pos = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
int8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_dir = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
int8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::cur_dir = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint16_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::ramp_stair = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint32_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_interval = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
StepperCallback Stepper<INTERRUPT, DRIVER, RAMP>::cb_complete = StepperCallback();

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint16_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::pre_decel_stairs_left = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint16_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::accel_stairs_left = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint32_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_steps_left = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint32_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_full_blocks_left = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_rest_block_steps = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::multi_steps_made = 0;