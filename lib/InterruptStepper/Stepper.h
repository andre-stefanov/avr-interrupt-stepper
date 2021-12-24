#ifndef __InterruptStepper_H__
#define __InterruptStepper_H__

#include "Arduino.h"
#include "AccelerationRamp.h"
#include <math.h>

typedef uint32_t steps_t;

#define DEBUG_STEPPER_TIMING_PIN 48
#if DEBUG_STEPPER_TIMING_PIN != 0 && UNIT_TEST != 1
#include <Pin.h>
#define STEPPER_TIMING_START() Pin<DEBUG_STEPPER_TIMING_PIN>::high()
#define STEPPER_TIMING_END() Pin<DEBUG_STEPPER_TIMING_PIN>::low()
#else
#define STEPPER_TIMING_START()
#define STEPPER_TIMING_END()
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886
#endif

#define RAMP_STAIRS 64

enum RampState
{
    IDLE,
    ACCEL,
    RUN,
    DECEL,
};

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
class Stepper
{

private:
    Stepper() = delete;
public:
    typedef AccelerationRamp<RAMP_STAIRS, INTERRUPT::FREQ, DRIVER::SPR, MAX_SPEED, ACCELERATION> Ramp;
    constexpr static Ramp ramp = Ramp();
    
    static RampState state;
    static uint32_t target_interval;
    static int8_t cur_dir;
    static uint32_t cur_step;
    static uint8_t cur_ramp_stair;
    static uint8_t cur_ramp_stair_step;
    static uint32_t decel_start;

public:

    static void move(float speed, uint32_t steps = UINT32_MAX)
    {
        // evaluates to -1, 0 and 1 where:
        // -1 = CCW
        // 0  = STOP
        // 1  = CW
        int8_t target_dir = (0.f < speed) - (speed < 0.f);

        if (state != IDLE)
        {
            if (target_dir != cur_dir)
            {
                // TODO: Change direction and speed (decelerate - accelerate)
            }
            else
            {
                // TODO: Change speed
            }
        }
        else
        {
            // Accelerate
            state = ACCEL;

            // Set driver/stepper direction
            cur_dir = target_dir;
            DRIVER::dir(target_dir > 0);

            // Set target speed/interval
            target_interval = (uint32_t)(ALPHA_T(DRIVER::SPR, INTERRUPT::FREQ) / speed + 0.5);

            // TODO: is it really needed? At this point index should be already at 0
            // Reset ramp related index
            cur_ramp_stair = 0;
            cur_ramp_stair_step = 0;

            // if overal movement is shorter than a complete (accel+decel) ramp,
            // deceleration has to start earlier (on the half way to the target position).
            // Otherwise deceleration start point is (steps - deceleration steps)
            if ((steps / 2) < Ramp::STEPS_PER_STAIR * RAMP_STAIRS)
            {
                decel_start = steps / 2;
            }
            else
            {
                decel_start = steps - (Ramp::STEPS_PER_STAIR * RAMP_STAIRS);
            }

            INTERRUPT::setCallback(move_accelerated_handler);
            INTERRUPT::setInterval(ramp.counters[0]);
        }
    }

    static void move_accelerated_handler()
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

                cur_ramp_stair_step = Ramp::STEPS_PER_STAIR - cur_ramp_stair_step;
            }
            else if (++cur_ramp_stair_step == Ramp::STEPS_PER_STAIR) // we reached the end of this ramp stair
            {
                // reset current step in the acceleration ramp stair to 0
                cur_ramp_stair_step = 0;

                if (++cur_ramp_stair < Ramp::STAIRS_COUNT) // if we did not reach the end of the ramp yet
                {
                    // set timer interval to the next ramp stair step interval
                    INTERRUPT::setInterval(ramp.counters[cur_ramp_stair]);
                }
                else // we reached end of the acceleration, switch to max speed mode
                {
                    // set timer interval to the max speed step delay
                    INTERRUPT::setInterval(target_interval);

                    // switch to the next ramp state (RUN)
                    state = RUN;

                    // prepare ramp stair index for deceleration (last element)
                    cur_ramp_stair = Ramp::STAIRS_COUNT - 1;
                }
            }
            break;
        case RUN:
            // start deceleration if we current step matches deceleration start step
            if (cur_step >= decel_start)
            {
                // set interrupt interval to the last ramp stair of the acceleration
                INTERRUPT::setInterval(ramp.counters[cur_ramp_stair]);
                state = DECEL;
            }
            break;
        case DECEL:
            if (++cur_ramp_stair_step == Ramp::STEPS_PER_STAIR) // we reached the end of this ramp stair
            {
                if (cur_ramp_stair-- > 0) // if we did not reach the end of the ramp yet
                {
                    INTERRUPT::setInterval(ramp.counters[cur_ramp_stair]);
                    cur_ramp_stair_step = 0;
                }
                else // we reached end of the ramp, stop
                {
                    state = IDLE;
                }
            }
            break;
        case IDLE:
            INTERRUPT::stop();
            cur_ramp_stair = 0;
            cur_ramp_stair_step = 0;
            break;
        }

        STEPPER_TIMING_END();
    }
}; //InterruptStepper

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
int8_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::cur_dir = 1;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::target_interval = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
RampState Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::state = IDLE;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::cur_step = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint8_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::cur_ramp_stair = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint8_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::cur_ramp_stair_step = 0;

template <typename INTERRUPT, typename DRIVER, uint32_t MAX_SPEED, uint32_t ACCELERATION>
uint32_t Stepper<INTERRUPT, DRIVER, MAX_SPEED, ACCELERATION>::decel_start = 0;

#endif //__InterruptStepper_H__
