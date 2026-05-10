#pragma once

#ifndef PROFILE_STEPPER
#define PROFILE_STEPPER 0
#endif
#ifndef PROFILE_STEPPER_PIN
#define PROFILE_STEPPER_PIN 45
#endif

#if PROFILE_STEPPER && defined(ARDUINO)
#include "Pin.h"
#define PROFILE_MOVE_BEGIN() Pin<PROFILE_STEPPER_PIN>::high()
#define PROFILE_MOVE_END() Pin<PROFILE_STEPPER_PIN>::low()
#else
#define PROFILE_MOVE_BEGIN() \
    do                       \
    {                        \
    } while (0)
#define PROFILE_MOVE_END() \
    do                     \
    {                      \
    } while (0)
#endif

#ifndef noInterrupts
#define noInterrupts() \
    do                 \
    {                  \
    } while (0)
#endif

#ifndef interrupts
#define interrupts() \
    do               \
    {                \
    } while (0)
#endif

#include <stdint.h> // NOLINT(modernize-deprecated-headers)

#include "etl/delegate.h"

#include "AccelerationRamp.h"

/**
 * @brief Number of constant-speed steps tracked as one logical run block.
 *
 * Fast moves still emit exactly one hardware step per interrupt. The planner simply commits
 * constant-speed progress in RUN_BLOCK_SIZE chunks so the interrupt handlers can update position
 * and counters less often.
 */
#define RUN_BLOCK_SIZE 128

/**
 * @brief Completion callback invoked when a move terminates.
 *
 * The callback runs synchronously inside `terminate()`. When a move finishes from an interrupt
 * callback, the completion handler therefore also runs in interrupt context.
 */
using StepperCallback = etl::delegate<void()>;

/**
 * @brief Interrupt-driven static stepper planner and executor.
 *
 * One template instantiation represents one motor. All state lives in static members so the class
 * can be used without allocating an object and can be called directly from interrupt callbacks.
 *
 * A move is modeled as up to four phases:
 * 1. pre-deceleration from the currently active speed, used when slowing down or reversing
 * 2. acceleration toward the requested speed
 * 3. constant-speed run
 * 4. deceleration to a stop
 *
 * Position is only committed when a complete stair or run block finishes. Steps already emitted in
 * the current block are held in `multi_steps_made` so the interrupt handlers can stay short while
 * `getPosition()` and `distanceToGo()` still report the in-flight progress.
 *
 * @tparam INTERRUPT Timer backend. It must expose `ID`, `init()`, `stop()`, `setCallback()`, and
 * `setInterval()`.
 * @tparam DRIVER Stepper output backend. It must expose `init()`, `step()`, `dir()`, and
 * `setInverted()`.
 * @tparam RAMP Ramp model that maps speed requests to timer intervals and acceleration stairs.
 */
template <typename INTERRUPT, typename DRIVER, typename RAMP>
class Stepper
{
public:
    constexpr static int TIMER_ID = INTERRUPT::ID;

    /**
     * @brief Static class. We don't need constructor.
     */
    Stepper() = delete;

private:
    static volatile int32_t pos; ///< Last committed absolute position in steps.

    static volatile int8_t run_dir; ///< Direction requested for the upcoming run phase.
    static volatile int8_t cur_dir; ///< Direction currently being stepped. Zero means idle.
    static volatile uint16_t ramp_stair; ///< Active ramp stair. Zero means no accelerated ramp.

    static volatile uint32_t run_interval; ///< Timer interval used during the constant-speed run phase.

    static volatile uint16_t pre_decel_stairs_left; ///< Stairs still needed before the requested profile can start.
    static volatile uint16_t accel_stairs_left; ///< Stairs still to climb after pre-deceleration or start-up.
    static volatile uint32_t run_steps_left; ///< Remaining single-step run distance for slow moves.
    static volatile uint32_t run_full_blocks_left; ///< Remaining full constant-speed run blocks.
    static volatile uint8_t run_rest_block_steps; ///< Tail steps after the full run blocks have completed.

    static volatile uint8_t multi_steps_made; ///< Steps already emitted inside the active stair or block.

    static StepperCallback cb_complete; ///< Completion callback consumed by `terminate()`.

    /**
     * @brief Consistent copy of the volatile planner state.
     *
     * Query helpers use this structure so they can reason about one coherent snapshot instead of a
     * mix of old and new values while the interrupt handler is running.
     */
    struct StateSnapshot
    {
        int32_t pos;
        int8_t cur_dir;
        uint16_t ramp_stair;
        uint16_t pre_decel_stairs_left;
        uint16_t accel_stairs_left;
        uint32_t run_steps_left;
        uint32_t run_full_blocks_left;
        uint8_t run_rest_block_steps;
        uint8_t multi_steps_made;
    };

    /**
     * @brief Copy the volatile state while interrupts are disabled.
     *
     * On embedded targets this prevents tearing between related counters. On desktop test builds
     * the `noInterrupts()` and `interrupts()` macros collapse to no-ops.
     */
    static StateSnapshot stateSnapshot()
    {
        noInterrupts();
        const StateSnapshot state = {
            pos,
            cur_dir,
            ramp_stair,
            pre_decel_stairs_left,
            accel_stairs_left,
            run_steps_left,
            run_full_blocks_left,
            run_rest_block_steps,
            multi_steps_made,
        };
        interrupts();

        return state;
    }

    /**
     * @brief Sum only the queued constant-speed run portion of a move.
     *
     * This excludes the active acceleration or deceleration ramps and any partially completed block.
     */
    static uint32_t queuedRunSteps(const StateSnapshot &state)
    {
        return state.run_steps_left + (state.run_full_blocks_left * static_cast<uint32_t>(RUN_BLOCK_SIZE)) + state.run_rest_block_steps;
    }

    /**
     * @brief Reconstruct how many steps are still owed by the active move.
     *
     * The planner stores each phase separately, so the remaining distance is derived by summing the
     * unfinished part of the active phase plus every queued phase behind it. `multi_steps_made`
     * represents steps already emitted on the motor pins but not yet folded into the phase counters,
     * so each branch subtracts it exactly once from the phase that is currently executing.
     */
    static uint32_t stepsRemaining(const StateSnapshot &state)
    {
        const uint32_t steps_per_stair = static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
        const uint32_t partial_steps = static_cast<uint32_t>(state.multi_steps_made);

        if (state.pre_decel_stairs_left > 0)
        {
            // Remaining profile: finish the current pre-deceleration stair, then any acceleration
            // stairs for the new target speed, then the queued run segment, then the final
            // deceleration ramp from the future peak stair back to zero.
            const uint32_t pre_decel_steps =
                (static_cast<uint32_t>(state.pre_decel_stairs_left) * steps_per_stair) - partial_steps;
            const uint32_t accel_steps = static_cast<uint32_t>(state.accel_stairs_left) * steps_per_stair;
            const uint32_t decel_stairs =
                static_cast<uint32_t>(state.ramp_stair - state.pre_decel_stairs_left + state.accel_stairs_left);

            return pre_decel_steps + accel_steps + queuedRunSteps(state) + (decel_stairs * steps_per_stair);
        }

        if (state.accel_stairs_left > 0)
        {
            // Remaining profile: finish acceleration, execute the queued run segment, then descend
            // the mirrored deceleration ramp. The peak stair is `ramp_stair + accel_stairs_left - 1`.
            const uint32_t accel_steps =
                (static_cast<uint32_t>(state.accel_stairs_left) * steps_per_stair) - partial_steps;
            const uint32_t decel_stairs =
                static_cast<uint32_t>(state.ramp_stair) + state.accel_stairs_left - 1U;

            return accel_steps + queuedRunSteps(state) + (decel_stairs * steps_per_stair);
        }

        if (state.run_steps_left > 0)
        {
            // Slow runs commit one step per interrupt, so the counter already reflects the exact
            // remaining run distance.
            return state.run_steps_left;
        }

        if (state.run_full_blocks_left > 0)
        {
            // While running in full blocks, subtract the already emitted steps from the current
            // block once, then add the queued tail block and the deceleration ramp still to come.
            return (state.run_full_blocks_left * static_cast<uint32_t>(RUN_BLOCK_SIZE)) - partial_steps + state.run_rest_block_steps + (static_cast<uint32_t>(state.ramp_stair) * steps_per_stair);
        }

        if (state.run_rest_block_steps > 0)
        {
            // The tail block behaves like a shortened full block, followed by the deceleration ramp.
            return static_cast<uint32_t>(state.run_rest_block_steps) - partial_steps + (static_cast<uint32_t>(state.ramp_stair) * steps_per_stair);
        }

        if (state.ramp_stair > 0)
        {
            // Only the deceleration ramp is left.
            return (static_cast<uint32_t>(state.ramp_stair) * steps_per_stair) - partial_steps;
        }

        return 0;
    }

    /**
     * @brief Interrupt handler for the initial deceleration phase.
     *
     * This phase is entered when the currently active speed is too high for the newly requested
     * move, including reversals. Each completed stair either continues decelerating, switches to
     * acceleration in the opposite direction, or enters the run phase directly.
     */
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

    /**
     * @brief Interrupt handler for the acceleration phase.
     *
     * Each completed stair either advances to the next faster interval or hands over to the run
     * phase once the requested peak stair has been reached.
     */
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

    /**
     * @brief Interrupt handler for low-speed moves that execute one logical step per callback.
     */
    static void run_slow_handler()
    {
        DRIVER::step();

        pos += cur_dir;

        if (--run_steps_left == 0)
        {
            terminate();
        }
    }

    /**
     * @brief Interrupt handler for the final partial constant-speed block.
     *
     * Once the tail block is consumed, the planner either terminates immediately or enters the
     * deceleration ramp if a non-zero peak stair still needs to be unwound.
     */
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

    /**
     * @brief Interrupt handler for full RUN_BLOCK_SIZE constant-speed blocks.
     */
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

    /**
     * @brief Interrupt handler for the final deceleration ramp.
     *
     * The handler keeps the pulse timing accurate by emitting the hardware step first and only then
     * updating the bookkeeping and choosing the next interval.
     */
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
    /**
     * @brief Initialize the driver backend and the timer backend.
     */
    static void init()
    {
        DRIVER::init();
        INTERRUPT::init();
    }

    /**
     * @brief Abort the current plan immediately and optionally invoke the completion callback.
     *
     * This is the hard-stop primitive. Unlike `stop()`, it does not try to decelerate gracefully;
     * it clears the planner state right away and then calls the stored callback if requested.
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

    /**
     * @brief Forward the direction inversion flag to the motor driver backend.
     */
    static void setInverted(bool value)
    {
        DRIVER::setInverted(value);
    }

    /**
     * @brief Reset all planner counters and the committed position to zero.
     *
     * This function does not stop the timer backend first, so it is intended for use while the
     * stepper is already idle.
     */
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

    /**
     * @brief Return the current absolute position, including the in-flight partial block.
     */
    static int32_t getPosition()
    {
        const StateSnapshot state = stateSnapshot();

        return state.pos + (static_cast<int32_t>(state.multi_steps_made) * static_cast<int32_t>(state.cur_dir));
    }

    /**
     * @brief Override the committed absolute position.
     *
     * Only the base position is overwritten. If a move is in progress, the still-uncommitted steps
     * tracked in `multi_steps_made` will continue to be added on top.
     */
    static void setPosition(const int32_t value)
    {
        noInterrupts();
        pos = value;
        interrupts();
    }

    /**
     * @brief Return the non-negative number of steps still queued in the active plan.
     */
    static uint32_t distanceToGo()
    {
        return stepsRemaining(stateSnapshot());
    }

    /**
     * @brief Return whether the planner is currently emitting steps.
     */
    static bool isRunning()
    {
        return stateSnapshot().cur_dir != 0;
    }

    /**
     * @brief Immutable move request consumed by `move()`.
     *
     * The planner works in motor steps plus ramp metadata. Helper factories translate from more
     * convenient speed-based inputs into this normalized representation.
     */
    struct MovementSpec
    {
        const int32_t steps; ///< Signed relative target distance in motor steps.
        const uint32_t run_interval; ///< Timer interval used for the constant-speed run phase.
        const uint16_t accel_stair; ///< Highest acceleration stair the requested speed may reach.

        MovementSpec() = delete;

        /**
         * @brief Build a normalized move request from explicit planner values.
         */
        constexpr MovementSpec(
            const int32_t steps,
            const uint32_t runInterval,
            const uint16_t accelStair) : steps(steps),
                                         run_interval(runInterval),
                                         accel_stair(accelStair) {}

        /**
         * @brief Create a move request for a fixed relative distance.
         *
         * The sign of `speed` determines the sign of the resulting step count. `steps` is expected
         * to be a positive magnitude.
         */
        constexpr static MovementSpec distance(const float speed, const int32_t steps)
        {
            return MovementSpec(
                (speed >= 0.0f) ? steps : -steps,
                RAMP::getIntervalForSpeed(speed),
                RAMP::maxAccelStairs(speed));
        }

        /**
         * @brief Create a move request for a time window at the requested speed.
         *
         * The distance is computed as `speed * time / 1000` and truncated toward zero because the
         * planner can only emit whole motor steps.
         */
        constexpr static MovementSpec time(const float speed, const uint32_t time_ms)
        {
            auto steps = static_cast<int32_t>(speed * static_cast<float>(time_ms) / 1000.0f);
            uint32_t run_interval = RAMP::getIntervalForSpeed(speed);
            uint16_t full_accel_stairs = RAMP::maxAccelStairs(speed);
            return MovementSpec(steps, run_interval, full_accel_stairs);
        }
    };

    /**
     * @brief Request a controlled stop from the current motion state.
     *
     * If the motor is currently on an acceleration ramp, the planner switches to the matching
     * deceleration callback. If no ramp is active, the move is terminated immediately.
     */
    static void stop()
    {
        INTERRUPT::stop();

        if (ramp_stair > 0)
        {
            // Commit the partial block so the deceleration ramp starts from the exact current
            // position and with a clean `multi_steps_made` counter.
            pos += multi_steps_made * static_cast<int32_t>(cur_dir);
            multi_steps_made = 0;

            INTERRUPT::setCallback(decelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
        }
        else
        {
            terminate();
        }
    }

    /**
     * @brief Request a controlled stop and replace the completion callback.
     */
    static void stop(StepperCallback onComplete)
    {
        cb_complete = onComplete;
        stop();
    }

    /**
     * @brief Start or re-plan a practically unbounded move at the requested speed.
     *
     * The move uses `INT32_MAX - 1` as a large sentinel distance and is expected to be ended by a
     * later call to `stop()`, `terminate()`, or another `move...()` function.
     */
    static void move(const float sps, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec::distance(sps, INT32_MAX - 1), onComplete);
    }

    /**
     * @brief Move at the requested speed for approximately `time_ms` milliseconds.
     *
     * Duration is converted to an integer step count before planning, so the final elapsed time is
     * quantized to whole motor steps.
     */
    static void moveTime(const float sps, const uint32_t time_ms, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec::time(sps, time_ms), onComplete);
    }

    /**
     * @brief Plan a move toward an absolute target position.
     *
     * The method first converts the absolute target into a relative move from the current reported
     * position and guards the subtraction against signed 32-bit overflow.
     */
    static void moveTo(const float sps, const int32_t target, StepperCallback onComplete = StepperCallback())
    {
        int32_t position = getPosition();

        // prevent overflow in negative direction
        if (position > 0 && target < INT32_MIN + position)
        {
            move(MovementSpec::distance(sps, INT32_MIN), onComplete);
            return;
        }

        // prevent overflow in positive direction
        if (position < 0 && target > INT32_MAX + position)
        {
            move(MovementSpec::distance(sps, INT32_MAX), onComplete);
            return;
        }

        // move to requested target
        move(MovementSpec::distance(sps, target - position), onComplete);
    }

    /**
     * @brief Plan a relative move by an explicit signed step count.
     */
    static void
    moveBy(const float stepsPerSecond, const int32_t steps, StepperCallback onComplete = StepperCallback())
    {
        move(MovementSpec::distance(stepsPerSecond, steps), onComplete);
    }

    /**
     * @brief Plan or re-plan a move from the current execution state.
     *
     * Calling this while a move is already active is supported. The function first commits any
     * partially executed block into `pos`, clears the queued phases from the previous request, and
     * then derives a new profile from the current ramp stair and the requested `MovementSpec`.
     *
     * The planner distinguishes four cases:
     * 1. the current speed is too high to hit the target directly, so it must pre-decelerate
     * 2. the requested speed already matches the current stair, so it can continue directly
     * 3. the requested speed is slower, so it pre-decelerates and then runs
     * 4. the requested speed is faster, so it accelerates first and then runs
     */
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

        // `ramp_stair * STEPS_PER_STAIR` is the distance needed to unwind the currently active
        // deceleration ramp back to rest. The signed version expresses that same distance in the
        // direction the motor is currently moving.
        const auto abs_stop_steps_needed =
            static_cast<int32_t>(ramp_stair) * static_cast<int32_t>(RAMP::STEPS_PER_STAIR);
        const auto stop_steps_needed = abs_stop_steps_needed * cur_dir;

        // movement target can't be reached even by stopping/decelerating
        // need correction or revert of the direction
        if ((cur_dir * spec.steps) < (cur_dir * stop_steps_needed) && ramp_stair > 0)
        {
            run_dir = -cur_dir;

            pre_decel_stairs_left = ramp_stair;

            // The requested relative target lies before the point where the current motion could
            // stop. The planner therefore spends `stop_steps_needed` reaching zero speed and then
            // covers the remaining distance in the opposite direction.
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
                    // Only a triangular profile fits after the direction change. The planner picks
                    // the highest reachable stair and leaves any odd remainder as the center run.
                    accel_stairs_left = max_stair_possible;
                    run_rest_block_steps = steps - (max_stair_possible * (RAMP::STEPS_PER_STAIR * 2));
                    run_interval = RAMP::interval(accel_stairs_left);
                }
                // full ramp possible
                else
                {
                    accel_stairs_left = spec.accel_stair;

                    // Reserve the symmetric acceleration and deceleration ramps first. Whatever is
                    // left becomes the constant-speed run segment.
                    const auto abs_run_steps = static_cast<uint32_t>(steps - accel_decel_steps);

                    // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
                    run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
                    // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
                    run_rest_block_steps = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

                    run_interval = spec.run_interval;
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
            INTERRUPT::setInterval(RAMP::interval(ramp_stair));
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
            // The current stair already matches the requested speed, so only the constant-speed run
            // segment has to be planned. The reserved stop distance will be spent later by the final
            // deceleration ramp.
            const auto abs_run_steps = static_cast<uint32_t>(abs(spec.steps) - abs_stop_steps_needed);

            if (cur_dir == 0 || ramp_stair == 0)
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
                else if (run_rest_block_steps > 0)
                {
                    INTERRUPT::setCallback(run_rest_multistep_handler);
                }
                else
                {
                    INTERRUPT::setCallback(decelerate_multistep_handler);
                }
                INTERRUPT::setInterval(spec.run_interval);
            }
        }
        // requested speed is slower (lower acceleration ramp stair), need to pre-decelerate first then run
        else if (spec.accel_stair < ramp_stair)
        {
            run_interval = spec.run_interval;

            // The move must first descend from the current stair to the requested stair. Those
            // deceleration steps are reserved up front; the rest becomes the run segment.
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

            // The planner now chooses the highest reachable stair for this move. If the requested
            // peak stair does not fit into the available distance, it falls back to the highest
            // triangular profile that does fit.

            const auto required_accel_stairs = spec.accel_stair - ramp_stair;

            const auto req_accel_steps =
                static_cast<uint32_t>(required_accel_stairs) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const auto req_decel_steps =
                static_cast<uint32_t>(spec.accel_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
            const auto req_accel_decel_steps = req_accel_steps + req_decel_steps;
            const uint32_t abs_steps = (spec.steps >= 0) ? spec.steps : -spec.steps;

            // full ramp not possible
            if (abs_steps <= req_accel_decel_steps)
            {
                // We are already at `ramp_stair`, so only the still-missing acceleration stairs can
                // be added before a symmetric deceleration has to begin.
                const uint32_t accel_steps_made =
                    static_cast<uint32_t>(ramp_stair) * static_cast<uint32_t>(RAMP::STEPS_PER_STAIR);
                accel_stairs_left = static_cast<uint16_t>((abs_steps - accel_steps_made) /
                                                          (static_cast<uint32_t>(RAMP::STEPS_PER_STAIR) * 2U));

                if (ramp_stair > 0 || accel_stairs_left > 0)
                {
                    run_interval = RAMP::interval(ramp_stair + accel_stairs_left);
                }
                else
                {
                    run_steps_left = abs_steps;

                    INTERRUPT::setCallback(run_slow_handler);
                    INTERRUPT::setInterval(RAMP::interval(1));

                    PROFILE_MOVE_END();
                    return;
                }
            }
            // full ramp
            else
            {
                accel_stairs_left = required_accel_stairs;
                run_interval = spec.run_interval;
            }

            // After reserving both ramp halves around the chosen peak stair, the remaining distance
            // becomes the constant-speed run segment.
            const uint32_t abs_run_steps = abs_steps - ((ramp_stair + accel_stairs_left) * RAMP::STEPS_PER_STAIR * 2);

            // perform multi steps in run phase
            // will evaluate to 0 for run_steps < RUN_BLOCK_SIZE
            run_full_blocks_left = abs_run_steps / RUN_BLOCK_SIZE;
            // will evaluate to 0 for run_steps == n * RUN_BLOCK_SIZE
            run_rest_block_steps = static_cast<uint8_t>(abs_run_steps % RUN_BLOCK_SIZE);

            INTERRUPT::setCallback(accelerate_multistep_handler);
            INTERRUPT::setInterval(RAMP::interval(++ramp_stair));
        }

        PROFILE_MOVE_END();
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