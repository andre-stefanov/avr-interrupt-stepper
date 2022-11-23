#include "Arduino.h"

#include "TMCStepper.h"

#include "Configuration.h"

TMC2209Stepper* stepper;

void configureRaDriver()
{

    pinMode(38, OUTPUT);
    digitalWrite(30, HIGH);

    stepper = new TMC2209Stepper(63, 40, 0.11f, 0b00);
    stepper->beginSerial(19200);

    stepper->mstep_reg_select(true);
    stepper->pdn_disable(true);
    stepper->toff(0);
    stepper->rms_current(1000, 1.0f); // holdMultiplier = 1 to set ihold = irun
    stepper->toff(1);
    stepper->I_scale_analog(false);
    stepper->en_spreadCycle(0);
    stepper->blank_time(24);
    stepper->semin(0); // disable CoolStep so that current is consistent
    stepper->microsteps(256);
    stepper->intpol(true);
    stepper->TCOOLTHRS(0xFFFFF);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    configureRaDriver();

    using axis = config::Ra;
    using ramp = config::Ra::ramp_slew;
    using stepper = axis::stepper_slew;

    stepper::init();
    
    stepper::stop();
    stepper::moveBy(axis::SPEED_SLEWING, ramp::STEPS_TOTAL * 10);

    // constexpr auto movement = stepper::MovementSpec::distance(axis::SPEED_SLEWING, ramp::STEPS_TOTAL * 10);
    // Serial.println(movement.run_interval);

    // Serial.println();
    // for (uint16_t i = 0; i < 255; i++)
    // {
    //     Serial.println(ramp::interval(i));
    // }
    
}

void loop()
{
    do { } while (1);
}

#ifdef ARDUINO_ARCH_AVR
ISR(TIMER3_OVF_vect) { IntervalInterrupt_AVR<Timer::TIMER_3>::handle_overflow(); }
ISR(TIMER3_COMPA_vect) { IntervalInterrupt_AVR<Timer::TIMER_3>::handle_compare_match(); }
ISR(TIMER4_OVF_vect) { IntervalInterrupt_AVR<Timer::TIMER_4>::handle_overflow(); }
ISR(TIMER4_COMPA_vect) { IntervalInterrupt_AVR<Timer::TIMER_4>::handle_compare_match(); }
#endif