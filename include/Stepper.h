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

#include "AccelerationRamp.h"
#include "etl/delegate.h"
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
    static volatile uint16_t ramp_stair;

    static volatile uint32_t run_interval;

    static volatile uint16_t pre_decel_stairs_left;
    static volatile uint16_t accel_stairs_left;
    static volatile uint32_t run_steps_left;
    static volatile uint32_t run_full_blocks_left;
    static volatile uint8_t run_rest_block_steps_left;

    static volatile uint8_t multi_steps_left;

    static StepperCallback cb_complete;

    /**
     * @brief Stop movement immediately and invoke onComplete interval if set.
     */
    static void terminate()
    {
        INTERRUPT::stop();
        INTERRUPT::setCallback(nullptr);

        run_dir = 0;
        ramp_stair = 0;
        run_interval = 0;

        if (cb_complete.is_valid())
        {
            cb_complete();
        }
    }

    static void start_movement(
        const int8_t direction,
        const uint16_t pre_decel_stairs,
        const uint16_t accel_stairs,
        const uint32_t run_steps,
        const uint32_t new_run_interval)
    {
        // TODO: calculate pos in case last movement was not finished yet

        run_dir = direction;
        pre_decel_stairs_left = pre_decel_stairs;
        accel_stairs_left = accel_stairs;
        run_steps_left = run_steps;
        run_interval = new_run_interval;

        // we need to pre-decelerate first (for direction change or lower speed)
        if (pre_decel_stairs > 0)
        {
            multi_steps_left = RAMP::STEPS_PER_STAIR;
            INTERRUPT::setCallback(pre_decelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
        }
        // accelerate (run speed is higher)
        else if (accel_stairs > 0)
        {
            // set direction in case stepper was idle or slow with different direction
            DRIVER::dir(run_dir > 0);

            multi_steps_left = RAMP::STEPS_PER_STAIR;

            INTERRUPT::setCallback(accelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(++ramp_stair));
        }
        // run directly, requested speed is similar to current one
        else if (run_steps > 0)
        {
            // set direction in case stepper was idle or slow with different direction
            DRIVER::dir(run_dir > 0);

            multi_steps_left = (run_steps >= RUN_BLOCK_SIZE) ? RUN_BLOCK_SIZE : run_steps;

            INTERRUPT::setCallback(run_full_multistep_handler);
            INTERRUPT::setInterval(new_run_interval);
        }
        // decelerate until stopped
        else if (ramp_stair > 0)
        {
            multi_steps_left = RAMP::STEPS_PER_STAIR;

            INTERRUPT::setCallback(decelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
        }
        // stop immediately
        else
        {
            INTERRUPT::stop();
            INTERRUPT::setCallback(nullptr);
        }
    }

    static void pre_decelerate_multistep_handler()
    {
        DRIVER::step();

        // check if this was last step of a multistep block
        if (--multi_steps_left == 0)
        {
            pos += (run_dir > 0) ? RAMP::STEPS_PER_STAIR : -RAMP::STEPS_PER_STAIR;

            // did not reach end of pre-deceleration, switch to next stair
            if (--pre_decel_stairs_left > 0)
            {
                multi_steps_left = RAMP::STEPS_PER_STAIR;

                INTERRUPT::setInterval(RAMP::interval(--ramp_stair));
            }
            // pre-deceleration finished, it was a direction switch, accelerate
            else if (accel_stairs_left > 0)
            {
                ramp_stair = 1;
                run_dir *= -1;
                DRIVER::dir(run_dir > 0);

                multi_steps_left = RAMP::STEPS_PER_STAIR;

                INTERRUPT::setCallback(accelerate_multistep_handler);
                INTERRUPT::setInterval(RAMP::interval(1));
            }
            // pre-deceleration finished, no need to accelerate, run
            else
            {
                // set dir in case this deceleration was a direction change with a slow run speed afterwards
                DRIVER::dir(run_dir > 0);
                
                if (run_steps_left > 0)
                {
                    INTERRUPT::setCallback(run_slow_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                else if (run_full_blocks_left > 0)
                {
                    multi_steps_left = RUN_BLOCK_SIZE;
                    INTERRUPT::setCallback(run_full_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                else
                {
                    multi_steps_left = run_rest_block_steps_left;
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
            }
        }
    }

    static void accelerate_multistep_handler()
    {
        DRIVER::step();

        if (--multi_steps_left == 0) // last step of multistep block
        {
            pos += (run_dir > 0) ? RAMP::STEPS_PER_STAIR : -RAMP::STEPS_PER_STAIR;

            // finished acceleration
            if (--accel_stairs_left == 0)
            {
                // switch to run phase (full blocks)
                if (run_full_blocks_left > 0)
                {
                    multi_steps_left = RUN_BLOCK_SIZE;
                    INTERRUPT::setCallback(run_full_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                // switch to run phase (rest)
                else if (run_rest_block_steps_left > 0)
                {
                    multi_steps_left = run_rest_block_steps_left;
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                    INTERRUPT::setInterval(run_interval);
                }
                // decelerate, no run phase needed
                else
                {
                    multi_steps_left = RAMP::STEPS_PER_STAIR;

                    INTERRUPT::setCallback(decelerate_multistep_handler);
                }
            }
            // continue acceleration
            else
            {
                multi_steps_left = RAMP::STEPS_PER_STAIR;

                INTERRUPT::setInterval(RAMP::interval(++ramp_stair));
            }
        }
    }

    static void run_slow_handler()
    {
        DRIVER::step();

        pos += (run_dir > 0) ? 1 : -1;

        if (--run_steps_left == 0)
        {
            terminate();
        }
    }

    static void run_rest_multistep_handler()
    {
        DRIVER::step();

        if (--multi_steps_left == 0)
        {
            pos += (run_dir > 0) ? run_rest_block_steps_left : -run_rest_block_steps_left;
            run_rest_block_steps_left = 0;

            // no deceleration needed
            if (ramp_stair == 0)
            {
                terminate();
            }
            // decelerate
            else
            {
                multi_steps_left = RAMP::STEPS_PER_STAIR;
                INTERRUPT::setInterval(RAMP::interval(ramp_stair));
                INTERRUPT::setCallback(decelerate_multistep_handler);
            }
        }
    }

    static void run_full_multistep_handler()
    {
        DRIVER::step();

        if (--multi_steps_left == 0)
        {
            static uint32_t b = 0; b++;
            pos += (run_dir > 0) ? RUN_BLOCK_SIZE : -RUN_BLOCK_SIZE;

            if (--run_full_blocks_left == 0)
            {
                if (run_rest_block_steps_left > 0)
                {
                    multi_steps_left = run_rest_block_steps_left;
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
                    multi_steps_left = RAMP::STEPS_PER_STAIR;
                    INTERRUPT::setInterval(RAMP::interval(ramp_stair));
                    INTERRUPT::setCallback(decelerate_multistep_handler);
                }
            }
            else
            {
                multi_steps_left = RUN_BLOCK_SIZE;
            }
        }
    }

    static void decelerate_multistep_handler()
    {
        // always step first to ensure the best accuracy.
        // other calculations should be done as quick as possible below.
        DRIVER::step();

        if (--multi_steps_left == 0)
        {
            pos += (run_dir > 0) ? RAMP::STEPS_PER_STAIR : -RAMP::STEPS_PER_STAIR;

            if (--ramp_stair == 0)
            {
                terminate();
            }
            else
            {
                multi_steps_left = RAMP::STEPS_PER_STAIR;
                INTERRUPT::setInterval(RAMP::interval(ramp_stair));
            }
        }
    }

public:
    static Angle position()
    {
        noInterrupts();
        const uint32_t cur_pos = pos;
        interrupts();

        return Angle::deg(360.0f / DRIVER::SPR) * cur_pos;
    }

    static void position(const Angle &value)
    {
        auto new_pos = static_cast<int32_t>((DRIVER::SPR / (2.0f * 3.14159265358979323846f)) * value.rad() + 0.5f);

        noInterrupts();
        pos = new_pos;
        interrupts();
    }

    static bool isRunning()
    {
        return run_dir != 0;
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
            start_movement(run_dir, 0, 0, 0, 0);
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

    static void moveBy(const Angle speed, const Angle distance, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, distance), onComplete);
    }

    static void moveBy(const Angle speed, const uint32_t steps, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec(speed, steps), onComplete);
    }

private:

    static void inline __attribute__((always_inline)) accelerate()
    {
        multi_steps_left = RAMP::STEPS_PER_STAIR;

        INTERRUPT::setCallback(accelerate_multistep_handler);
        INTERRUPT::setInterval(RAMP::interval(++ramp_stair));
    }

    /*
     * calculate ramp without taking into account run direction
     */
    static void inline __attribute__((always_inline)) calculate_ramp(const MovementSpec &spec)
    {
//        // perform acceleration to requested speed
//        accel_stairs_left = RAMP::STAIRS_COUNT - 1 - ramp_stair;
//
//        // amount of steps to run requested minus acceleration and deceleration steps
//        mv_run_steps = spec.steps - accel_steps - total_decel_steps;
//
//        // perform multi steps in run phase
//        run_full_blocks_left = mv_run_steps / RUN_BLOCK_SIZE;
//        run_rest_block_steps_left = static_cast<uint8_t>(mv_run_steps % RUN_BLOCK_SIZE);
//
//        // we can run at requested interval
//        run_interval = spec.run_interval;
    }

public:
    static void move(MovementSpec spec, StepperCallback onComplete = StepperCallback())
    {
        PROFILE_MOVE_BEGIN();

        INTERRUPT::stop();

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
        const auto stop_steps_needed = static_cast<int32_t>(abs_stop_steps_needed) * run_dir;

        // movement target can't be reached even by stopping/decelerating
        // need correction or revert of the direction
        if (spec.steps < stop_steps_needed && ramp_stair > 0)
        {
            // reverse/compensate
            // pre-decelerate until stopped, then ramp
            terminate();
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
            const auto abs_run_steps = static_cast<uint32_t>(abs(spec.steps));

            if (spec.steps > 0)
            {
                run_dir = 1;
                DRIVER::dir(true);
            }
            else
            {
                run_dir = -1;
                DRIVER::dir(false);
            }

            // run directly
            if (ramp_stair == 0)
            {
                run_steps_left = abs_run_steps;

                INTERRUPT::setCallback(run_slow_handler);
                INTERRUPT::setInterval(spec.run_interval);
            }
            else
            {
                // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
                run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
                // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
                run_rest_block_steps_left = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

                if (run_rest_block_steps_left > 0)
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
            const uint32_t abs_run_steps = abs(spec.steps) - abs_stop_steps_needed;

            pre_decel_stairs_left = ramp_stair - spec.accel_stair;

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
                run_rest_block_steps_left = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);
            }

            multi_steps_left = RAMP::STEPS_PER_STAIR;

            INTERRUPT::setCallback(pre_decelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
        }
        // requested speed is faster (higher acceleration ramp stair), need to accelerate first then run
        else
        {
            if (spec.steps > 0)
            {
                run_dir = 1;
                DRIVER::dir(true);
            }
            else
            {
                run_dir = -1;
                DRIVER::dir(false);
            }

            // TODO: optimize these calculations

            const auto required_accel_stairs = spec.accel_stair - ramp_stair;

            const auto req_accel_steps = static_cast<uint32_t>(required_accel_stairs) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const auto req_decel_steps = static_cast<uint32_t>(spec.accel_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const auto req_accel_decel_steps = req_accel_steps + req_decel_steps;
            const auto abs_steps = static_cast<uint32_t>(abs(spec.steps));

            // full ramp not possible
            if (abs_steps <= req_accel_decel_steps)
            {
                const uint32_t accel_steps_made = static_cast<uint32_t>(ramp_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
                accel_stairs_left = (abs_steps - accel_steps_made) / RAMP::STEPS_PER_STAIR / 2;
                run_interval = RAMP::interval(ramp_stair + accel_stairs_left);
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
            run_rest_block_steps_left = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

            accelerate();
        }

//        return;
//
//        // zero steps movement, stop
//        if (spec.steps == 0)
//        {
//            // TODO
//        }
//        // revert direction
//        else if (spec.dir != run_dir && ramp_stair > 0)
//        {
//            // decelerate until full stop to move in other direction
//            mv_pre_decel_stairs = ramp_stair;
//
//            // we need to add steps made in pre deceleration to achieve accurate end position
//            const uint32_t mv_steps_after_pre_decel = spec.steps + (static_cast<uint16_t>(mv_pre_decel_stairs) * RAMP::STEPS_PER_STAIR);
//
//            // amount of steps needed (ideally) for full acceleration after stop
//            const uint16_t full_accel_steps = static_cast<uint32_t>(spec.accel_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
//
//            // amount of steps needed (ideally) for full acceleration + deceleration
//            const uint32_t accel_decel_steps = static_cast<uint32_t>(full_accel_steps) * 2;
//
//            // if final movement not long enough for a full acceleration + deceleration
//            if (mv_steps_after_pre_decel < accel_decel_steps)
//            {
//                // acceleration has to end before reaching its maximum
//                const uint32_t mv_accel_steps = mv_steps_after_pre_decel / 2;
//
//                mv_accel_stairs = mv_accel_steps / RAMP::STEPS_PER_STAIR;
//
//                mv_run_steps = mv_steps_after_pre_decel - (mv_accel_stairs * 2 * RAMP::STEPS_PER_STAIR);
//            }
//            // enough steps for a full ramp (accelleration + run + decelleration)
//            else
//            {
//                // acceleration will reach maximum
//                mv_accel_stairs = RAMP::STAIRS_COUNT - 1;
//
//                // there will be some steps with max speed
//                mv_run_steps = spec.steps - (mv_accel_stairs * 2 * RAMP::STEPS_PER_STAIR);
//
//                // run at requested speed
//                mv_run_interval = spec.run_interval;
//            }
//        }
//        // small speed change (if at all), no need for pre-deceleration or acceleration
//        else if (spec.accel_stair == ramp_stair)
//        {
//            run_dir = spec.dir;
//
//            // set direction in case stepper was idle or slow with different direction
//            DRIVER::dir(run_dir > 0);
//
//            run_steps_left = spec.steps - (static_cast<uint16_t>(ramp_stair) * RAMP::STEPS_PER_STAIR);
//
//            INTERRUPT::setCallback(run_slow_handler);
//            INTERRUPT::setInterval(spec.run_interval);
//        }
//        // target speed is slower than current speed, we need to decelerate first
//        else if (spec.accel_stair < ramp_stair)
//        {
//            // steps needed for pre-deceleration and full deceleration
//            const uint32_t total_decel_steps = static_cast<uint16_t>(ramp_stair) * RAMP::STEPS_PER_STAIR;
//
//            // not enough steps to reach target position at full stop, we need a correction in opposite direction
//            if (spec.steps < total_decel_steps)
//            {
//                // pre decelerate to full stop (overshooting actual target position)
//                mv_pre_decel_stairs = ramp_stair;
//
//                // specified steps were not enough for a full deceleration.
//                // compensation will be shorter than steps needed for full deceleration.
//                // compensate with a partial ramp (under max speed)
//                const uint32_t compensation_steps = spec.steps - total_decel_steps;
//                mv_accel_stairs = compensation_steps / 2 / RAMP::STEPS_PER_STAIR;
//                mv_run_steps = compensation_steps - (mv_accel_stairs * 2 * RAMP::STEPS_PER_STAIR);
//            }
//            else
//            {
//                // we need to decelerate by the diff of current and target speed first
//                mv_pre_decel_stairs = ramp_stair - spec.accel_stair;
//
//                // steps at target speed are rest of all steps minus steps needed for both decelerations
//                mv_run_steps = spec.steps - total_decel_steps;
//            }
//        }
//        // requested speed higher than current one, need to accelerate
//        else
//        {
//            // steps needed for instant deceleration
//            const uint32_t immediate_decel_steps = static_cast<uint32_t>(ramp_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
//
//            // steps needed for full deceleration at the end of actual ramp (if fully applicable)
//            const uint32_t total_decel_steps = static_cast<uint32_t>(spec.accel_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
//
//            // steps needed for acceleration between current and requested speed
//            const uint32_t accel_steps = total_decel_steps - immediate_decel_steps;
//
//            // not enough steps to even immediately decelerate, need to compensate backwards
//            if (spec.steps < immediate_decel_steps)
//            {
//                // first decelerate to complete stop
//                mv_pre_decel_stairs = ramp_stair;
//
//                // amount of steps to compensate into opposite direction
//                // uint16_t compensate_steps = spec.steps + immediate_decel_steps;
//
//                // specified steps were not enough for an immediate deceleration. compensation will be shorter.
//                // compensate with a partial ramp (only accel + decel)
//                // mv_accel_steps = compensate_steps / 2; TODO
//
//                // TODO
//                // if amount of steps to compensate is odd, we have to add 1 step to the run phase
//                // if (compensate_steps & 0x1)
//                // {
//                //     mv_run_steps = 1;
//                //     mv_run_interval = RAMP::interval(ramp_stair + (mv_accel_steps / RAMP::STEPS_PER_STAIR));
//                // }
//            }
//            // not enough steps for full acceleration to requested speed and later deceleration to full stop.
//            else if (spec.steps < accel_steps + total_decel_steps)
//            {
//                // only accelerate as much to decelerate to requested position
//                // mv_accel_steps = spec.steps / 2; TODO
//
//                // make just one run step in case spec.steps is odd
//                // if (spec.steps & 0x1)
//                // {
//                //     mv_run_steps = 1;
//                //     if (spec.steps == 1)
//                //     {
//                //         mv_run_interval = RAMP::interval(RAMP::LAST_STEP);
//                //     }
//                //     else
//                //     {
//                //         mv_run_interval = RAMP::interval(ramp_stair + (mv_accel_steps / RAMP::STEPS_PER_STAIR));
//                //     }
//                // }
//            }
//            // enough steps for "full" ramp
//            else
//            {
//                run_dir = spec.dir;
//
//                // set direction in case stepper was idle or slow with different direction
//                DRIVER::dir(run_dir > 0);
//
//                // perform acceleration to requested speed
//                accel_stairs_left = RAMP::STAIRS_COUNT - 1 - ramp_stair;
//
//                // amount of steps to run requested minus acceleration and deceleration steps
//                mv_run_steps = spec.steps - accel_steps - total_decel_steps;
//
//                // perform multi steps in run phase
//                run_full_blocks_left = mv_run_steps / RUN_BLOCK_SIZE;
//                run_rest_block_steps_left = static_cast<uint8_t>(mv_run_steps % RUN_BLOCK_SIZE);
//
//                // we can run at requested interval
//                run_interval = spec.run_interval;
//
//                accelerate();
//            }
//        }
//
////        start_movement(spec.dir, mv_pre_decel_stairs, mv_accel_stairs, mv_run_steps, mv_run_interval);
//
//        PROFILE_MOVE_END();
    }
};

template <typename INTERRUPT, typename DRIVER, typename RAMP>
int32_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::pos = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
int8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_dir = 1;

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
uint8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::run_rest_block_steps_left = 0;

template <typename INTERRUPT, typename DRIVER, typename RAMP>
uint8_t volatile Stepper<INTERRUPT, DRIVER, RAMP>::multi_steps_left = 0;