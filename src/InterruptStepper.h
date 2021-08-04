#ifndef __InterruptStepper_H__
#define __InterruptStepper_H__

#include <math.h>
#include "FastPin.h"
#include "PreciseInterrupt.h"

#define ALPHA (2 * M_PI / SPR)
#define ALPHA_2 (2 * ALPHA)
#define TIMER_FREQ F_CPU
#define TIMER_INTERVAL (1.0 / TIMER_FREQ)
#define ALPHA_T (ALPHA * F_CPU)

enum RampState
{
    STOP,
    ACCEL,
    RUN,
    DECEL
};

template <uint8_t RAMP_LEVELS>
class InterruptStepper
{
private:
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

        for (uint8_t level = 0; level < RAMP_LEVELS; level++)
        {
            steps_left += steps_per_level;
            while (steps_left > 0)
            {
                steps_processed++;
                ramp_level_time += c0 * (sqrtf(steps_processed + 1) - sqrtf(steps_processed));
                steps_left--;
            }

            ramp_level_delays[level] = static_cast<uint32_t>(ramp_level_time / steps_per_level);
            ramp_level_time = 0;
        }
    }

    static void move(int64_t steps, float speed)
    {
        // TODO: handle direction
        cur_step = 0;
        state = RUN;
        isr1.setCallback(move_handler);
        isr1.setInterval(static_cast<uint32_t>(ALPHA_T / speed + 0.5));
    }

    static void moveAccelerated(int32_t steps, float speed)
    {
        state = ACCEL;
        cur_ramp_level = 0;
        cur_ramp_level_step = 0;

        // TODO: allow slew +/- tracking

        if (static_cast<uint32_t>(steps / 2) < steps_per_level * 64)
        {
            decel_start = steps / 2;
        }
        else
        {
            decel_start = steps - (steps_per_level * 64);
        }

        isr1.setCallback(move_accelerated_handler);
        isr1.setInterval(ramp_level_delays[0]);
    }

    static inline __attribute__((always_inline)) void move_handler()
    {
        FastPin<54>::pulse();
    }

    static inline __attribute__((always_inline)) void move_accelerated_handler()
    {
        FastPin<54>::pulse();
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
                    isr1.setInterval(ramp_level_delays[cur_ramp_level]);
                }
                else // we reached end of the acceleration, switch to max speed mode
                {
                    // set timer interval to the max speed step delay
                    isr1.setInterval(min_c);

                    // switch to the next ramp state (RUN)
                    state = RUN;

                    // prepare ramp level index for deceleration (last element)
                    cur_ramp_level = RAMP_LEVELS - 1;
                }
            }
            break;
        case RUN:
            if (cur_step >= decel_start) // start deceleration
            {
                isr1.setInterval(ramp_level_delays[cur_ramp_level]);
                state = DECEL;
            }
            break;
        case DECEL:
            if (++cur_ramp_level_step == steps_per_level) // we reached the end of this ramp level
            {
                if (cur_ramp_level-- > 0) // if we did not reach the end of the ramp yet
                {
                    isr1.setInterval(ramp_level_delays[cur_ramp_level]);
                    cur_ramp_level_step = 0;
                }
                else // we reached end of the ramp, stop
                {
                    isr1.setInterval(min_c);
                    state = STOP;
                }
            }
            break;
        case STOP:
            isr1.setCallback(nullptr);
            isr1.stop();
            cur_ramp_level = 0;
            cur_ramp_level_step = 0;
            break;
        }
    }

}; //InterruptStepper

template <uint8_t RAMP_LEVELS>
RampState InterruptStepper<RAMP_LEVELS>::state = STOP;

template <uint8_t RAMP_LEVELS>
uint32_t InterruptStepper<RAMP_LEVELS>::min_c = 0;

template <uint8_t RAMP_LEVELS>
uint32_t InterruptStepper<RAMP_LEVELS>::ramp_level_delays[RAMP_LEVELS];

template <uint8_t RAMP_LEVELS>
uint32_t InterruptStepper<RAMP_LEVELS>::cur_step = 0;

template <uint8_t RAMP_LEVELS>
uint32_t InterruptStepper<RAMP_LEVELS>::steps_per_level = 0;

template <uint8_t RAMP_LEVELS>
uint8_t InterruptStepper<RAMP_LEVELS>::cur_ramp_level = 0;

template <uint8_t RAMP_LEVELS>
uint8_t InterruptStepper<RAMP_LEVELS>::cur_ramp_level_step = 0;

template <uint8_t RAMP_LEVELS>
uint32_t InterruptStepper<RAMP_LEVELS>::decel_start = 0;

#endif //__InterruptStepper_H__
