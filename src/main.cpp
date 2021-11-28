#define SPR (400LL * 64LL)
#define SIDEREAL (86164.0905f)
#define TRANSMISSION (35.46611505122143)

#define INTERRUPT_TIMER_1 1

#include "FastPin.h"
#include "InterruptStepper.h"
#include "TimerInterrupt.h"

#include <Arduino.h>

#define RA_RAMP_LEVELS 64

#define PIN_STEP 46
#define PIN_DIR 47

#define DEBUG_LOOP_TIMING_PIN 50
#define DEBUG_SETUP_TIMING_PIN 51

typedef Driver<PIN_STEP, PIN_DIR> RaDriver;

namespace axis
{
  typedef InterruptStepper<TIMER_1, RA_RAMP_LEVELS, RaDriver> ra;
}

void setup()
{
  // Serial.begin(115200);
  // delay(200);

  FastPin<46>::init();
  FastPin<47>::init();
  FastPin<48>::init();
  FastPin<49>::init();
  FastPin<50>::init();
  FastPin<51>::init();
  FastPin<52>::init();
  FastPin<53>::init();

  FastPin<DEBUG_SETUP_TIMING_PIN>::high();

  // FastPin<49>::pulse();

  // InterruptRA::init();
  // InterruptRA::setCallback(step);
  // InterruptRA::setFrequency(TRANSMISSION * SPR / SIDEREAL);
  // InterruptRA::setFrequency(200.0f);

  // tracking speed in deg
  float tracking_speed = TRANSMISSION * SPR / SIDEREAL;
  // slew at full stepper speed (600 RPM)
  // float sleweing_deg_per_sec = 2;
  float slewing_speed = 10 * 2 * TRANSMISSION;

  // Serial.println("=================================");

  axis::ra::init(DEG_TO_RAD * slewing_speed, DEG_TO_RAD * slewing_speed * 2);
  // axis::ra::moveAccelerated(40LL * 1000LL, DEG_TO_RAD * slewing_speed);
  axis::ra::move(DEG_TO_RAD * tracking_speed, 1000);
  // InterruptStepper<InterruptRA, 64>::moveAccelerated(1000, slewing_speed);

  FastPin<DEBUG_SETUP_TIMING_PIN>::low();
}

void loop()
{
  while (1)
  {
    FastPin<DEBUG_LOOP_TIMING_PIN>::pulse();
  }
}

ISR(TIMER1_OVF_vect) { TimerInterrupt<TIMER_1>::handle_overflow(); }

ISR(TIMER1_COMPA_vect) { TimerInterrupt<TIMER_1>::handle_compare_match(); }
