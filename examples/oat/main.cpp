#include "Arduino.h"

// #include "Mount.h"
// #include "TMCStepper.h"

// Mount mount;

// using pin_step = Pin<A0>;
// using interrupt = IntervalInterrupt<Timer::TIMER_3>;

// void step()
// {
//     pin_step::pulse();
// }

// TMC2209Stepper *stepper = nullptr;

// void configureRaDriver()
// {

//     pinMode(38, OUTPUT);
//     digitalWrite(30, HIGH);

//     stepper = new TMC2209Stepper(63, 40, 0.11f, 0b00);
//     stepper->beginSerial(19200);

//     stepper->mstep_reg_select(true);
//     stepper->pdn_disable(true);
//     stepper->toff(0);
//     stepper->rms_current(1000, 1.0f); // holdMultiplier = 1 to set ihold = irun
//     stepper->toff(1);
//     stepper->I_scale_analog(false);
//     stepper->en_spreadCycle(0);
//     stepper->blank_time(24);
//     stepper->semin(0); // disable CoolStep so that current is consistent
//     stepper->microsteps(128);
//     stepper->intpol(true);
//     stepper->TCOOLTHRS(0xFFFFF);
// }

void setup()
{
    Serial.begin(115200);
    while(!Serial);

    // mount.setup();
    // mount.track(true);

    // pin_step::init();
    // interrupt::init();

    // interrupt::setCallback(&step);
    // interrupt::setInterval(800UL);

    for (uint16_t i = 1; i <= 255; i++)
    {
        Serial.println(config::Ra::ramp_slew::interval(i));
    }
}

void loop()
{
    // mount.guide<Mount::Ra>(false, 500);
    // mount.guide<Mount::Dec>(false, 500);

    // delay(750);
    // mount.guide<Mount::Ra>(true, 500);
    // mount.guide<Mount::Dec>(true, 500);

    // delay(750);
    // mount.slewTo<Mount::Ra>(Angle::deg(-1.0f));
    // mount.slewTo<Mount::Dec>(Angle::deg(-1.0f));

    // delay(1000);
    // mount.startSlewing<Mount::Ra>(true);
    // mount.startSlewing<Mount::Dec>(true);

    // delay(500);
    // mount.stopSlewing<Mount::Ra>();
    // mount.stopSlewing<Mount::Dec>();

    do
    {
        // if (IntervalInterrupt_AVR<Timer::TIMER_3>::callback != nullptr)
        // {
        //     IntervalInterrupt_AVR<Timer::TIMER_3>::callback();
        // }
        // finish
    } while (1);
}