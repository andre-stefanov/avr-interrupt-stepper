#ifndef __InterruptStepper_H__
#define __InterruptStepper_H__

#include <math.h>
#include <TimerInterrupt.h>
#include <Driver.h>

#define ALPHA (2 * M_PI / SPR)
#define ALPHA_2 (2 * ALPHA)
#define TIMER_FREQ F_CPU
#define TIMER_INTERVAL (1.0 / TIMER_FREQ)
#define ALPHA_T (ALPHA * F_CPU)

#define DEBUG_STEPPER_TIMING_PIN 48
#if DEBUG_STEPPER_TIMING_PIN != 0
#include <FastPin.h>
#define STEPPER_TIMING_START() FastPin<DEBUG_STEPPER_TIMING_PIN>::high()
#define STEPPER_TIMING_END() FastPin<DEBUG_STEPPER_TIMING_PIN>::low()
#else
#define STEPPER_TIMING_START()
#define STEPPER_TIMING_END()
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

enum RampState
{
    STOP,
    ACCEL,
    RUN,
    DECEL,
};

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
class InterruptStepper
{
private:
    typedef TimerInterrupt<TIMER> INTERRUPT;

    InterruptStepper() = delete;

    static RampState state;
    static uint32_t min_c;
    static uint32_t ramp_level_delays[RAMP_LEVELS];
    static uint32_t steps_per_level;
    static uint32_t cur_step;
    static uint8_t cur_ramp_level;
    static uint8_t cur_ramp_level_step;
    static uint32_t decel_start;

public:
    /**
     * @brief Initialize the stepper code with acceleration ramp timings based on AVR466
     * 
     * @param maxSpeed maximal ramp speed (deg/s)
     * @param acceleration ramp acceleration and deceleration (deg/s/s)
     */
    static void init(float maxSpeed, float acceleration)
    {
        // interrupt delay at max speed
        min_c = static_cast<uint32_t>(ALPHA_T / maxSpeed + 0.5);

        // amount of steps needed to reach max speed
        float max_s_lim = (maxSpeed * maxSpeed) / (ALPHA_2 * acceleration);

        // first ideal delay
        float c0 = TIMER_FREQ * sqrtf(ALPHA_2 / acceleration);

        // amount of steps per ramp level rounded down
        steps_per_level = static_cast<uint32_t>(max_s_lim / RAMP_LEVELS);

        // amount of steps left in current ramp level
        uint32_t steps_left = 0;

        // accumulated ramp level time (sum of all step delays in this ramp level)
        float ramp_level_time = c0;

        // amount of steps already processed for this ramp calculation
        uint32_t steps_processed = 0;

        // calculate ramp values
        for (uint8_t level = 0; level < RAMP_LEVELS; level++)
        {
            // increment amount of steps left in this ramp level
            steps_left += steps_per_level;

            // for all steps left in this ramp level
            while (steps_left > 0)
            {
                // increment processed step amount
                steps_processed++;

                // calculate time based on AVR466
                ramp_level_time += c0 * (sqrtf(steps_processed + 1) - sqrtf(steps_processed));

                // decrement amount of left steps
                steps_left--;
            }

            // calculate actual delay (in clocks) between steps in this ramp level
            ramp_level_delays[level] = static_cast<uint32_t>(ramp_level_time / steps_per_level);

            // reset calculated ramp level time so we can accumulate again in next iteration
            ramp_level_time = 0;
        }

        INTERRUPT::init();
    }

    static void move(float speed, uint32_t steps = UINT32_MAX)
    {
        cur_step = steps;
        DRIVER::dir(speed >= 0.f);
        state = RUN;
        INTERRUPT::setCallback(move_handler);
        INTERRUPT::setInterval(static_cast<uint32_t>(ALPHA_T / speed + 0.5));
    }

    static void moveAccelerated(int32_t steps, float speed)
    {
        state = ACCEL;
        DRIVER::dir(speed >= 0.f);
        cur_ramp_level = 0;
        cur_ramp_level_step = 0;

        // TODO: allow slew + tracking either by adapting the speed or adding aditional steps to compensate the difference

        if (static_cast<uint32_t>(steps / 2) < steps_per_level * 64)
        {
            decel_start = steps / 2;
        }
        else
        {
            decel_start = steps - (steps_per_level * 64);
        }

        INTERRUPT::setCallback(move_accelerated_handler);
        INTERRUPT::setInterval(ramp_level_delays[0]);
    }

    static inline __attribute__((always_inline)) void move_handler()
    {
        DRIVER::step();
        if (--cur_step == 0)
        {
            INTERRUPT::stop();
        }
    }

    static inline __attribute__((always_inline)) void move_accelerated_handler()
    {
        STEPPER_TIMING_START();
        DRIVER::step();
        cur_step++;

        // this could be optimized even further by setting interrupt callback to appropriate functions depending on the state
        switch (state)
        {
        case ACCEL:
            if (cur_step >= decel_start) // if there is no RUN phase in this motion
            {
                // start deceleration
                state = DECEL;

                cur_ramp_level_step = steps_per_level - cur_ramp_level_step;
            }
            else if (++cur_ramp_level_step == steps_per_level) // we reached the end of this ramp level
            {
                // reset current step in the level to 0
                cur_ramp_level_step = 0;

                if (++cur_ramp_level < RAMP_LEVELS) // if we did not reach the end of the ramp yet
                {
                    // set timer interval to the next ramp level step delay
                    INTERRUPT::setInterval(ramp_level_delays[cur_ramp_level]);
                }
                else // we reached end of the acceleration, switch to max speed mode
                {
                    // set timer interval to the max speed step delay
                    INTERRUPT::setInterval(min_c);

                    // switch to the next ramp state (RUN)
                    state = RUN;

                    // prepare ramp level index for deceleration (last element)
                    cur_ramp_level = RAMP_LEVELS - 1;
                }
            }
            break;
        case RUN:
            // start deceleration if we current step matches deceleration start step
            if (cur_step >= decel_start)
            {
                // set interrupt interval to the last ramp level of the acceleration
                INTERRUPT::setInterval(ramp_level_delays[cur_ramp_level]);
                state = DECEL;
            }
            break;
        case DECEL:
            if (++cur_ramp_level_step == steps_per_level) // we reached the end of this ramp level
            {
                if (cur_ramp_level-- > 0) // if we did not reach the end of the ramp yet
                {
                    INTERRUPT::setInterval(ramp_level_delays[cur_ramp_level]);
                    cur_ramp_level_step = 0;
                }
                else // we reached end of the ramp, stop
                {
                    state = STOP;
                }
            }
            break;
        case STOP:
            INTERRUPT::stop();
            cur_ramp_level = 0;
            cur_ramp_level_step = 0;
            break;
        }

        STEPPER_TIMING_END();
    }
}; //InterruptStepper

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
RampState InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::state = STOP;

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint32_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::min_c = 0;

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint32_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::ramp_level_delays[RAMP_LEVELS];

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint32_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::cur_step = 0;

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint32_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::steps_per_level = 0;

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint8_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::cur_ramp_level = 0;

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint8_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::cur_ramp_level_step = 0;

template <Timer TIMER, uint8_t RAMP_LEVELS, typename DRIVER>
uint32_t InterruptStepper<TIMER, RAMP_LEVELS, DRIVER>::decel_start = 0;

#endif //__InterruptStepper_H__
