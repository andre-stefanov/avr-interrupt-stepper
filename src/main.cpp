#define SIDEREAL 86164.0905f
#define TRANSMISSION 35.46611505122143

#define TRACKING_DEG_PER_SEC (360 / SIDEREAL * TRANSMISSION)

#define SLEWING_DEG_PER_SEC (4 * TRANSMISSION)

#include "Pin.h"
#include "Driver.h"
#include "Stepper.h"
#include "TimerInterrupt.h"
#include "AccelerationRamp.h"

#include <Arduino.h>

#define RA_RAMP_LEVELS 64

#define PIN_RA_STEP 46
#define PIN_RA_DIR 47

#define DEBUG_LOOP_TIMING_PIN 50
#define DEBUG_SETUP_TIMING_PIN 51

namespace axis
{
  namespace ra
  {
    auto transmission = 35.46611505122143;
    using interrupt = TimerInterrupt<TIMER_1>;
    using driver = Driver<Pin<PIN_RA_STEP>, Pin<PIN_RA_DIR>>;
    using stepper = Stepper<
        interrupt,
        driver,
        (uint32_t)SLEWING_DEG_PER_SEC,
        (uint32_t)(2 * SLEWING_DEG_PER_SEC)>;
  }
}

void setup()
{
  Serial.begin(115200);
  // delay(1000);

  // constexpr auto freq = F_CPU;
  // constexpr auto transmission = 35.46611505122143;
  // constexpr auto spr = 400LL * 256LL;
  // constexpr auto max_speed = (uint32_t)(2 * transmission);
  // constexpr auto acceleration = (uint32_t)(4 * transmission);

  // typedef AccelerationRamp<64, freq, spr, max_speed, acceleration> RARamp;
  // constexpr auto ramp = RARamp();
  // Serial.println();
  // for (int i = 0; i < 64; i++)
  // {
  //   if (i % 8 == 0)
  //   {
  //     if (i > 0)
  //       Serial.println();
  //   }
  //   else
  //   {
  //     Serial.print(", ");
  //   }
  //   Serial.print(axis::ra::stepper::ramp.counters[i]);
  // }
  // Serial.println();

  Pin<46>::init();
  Pin<47>::init();
  Pin<48>::init();
  Pin<49>::init();
  Pin<50>::init();
  Pin<51>::init();
  Pin<52>::init();
  Pin<53>::init();

  Pin<DEBUG_SETUP_TIMING_PIN>::high();

  axis::ra::interrupt::init();

  axis::ra::stepper::move(DEG_TO_RAD * SLEWING_DEG_PER_SEC, 10000);
  // axis::ra::stepper::move(DEG_TO_RAD * TRACKING_DEG_PER_SEC);

  Pin<DEBUG_SETUP_TIMING_PIN>::low();
}

void loop()
{
  while (1)
  {
    Pin<DEBUG_LOOP_TIMING_PIN>::pulse();
  }
}

ISR(TIMER1_OVF_vect) { TimerInterrupt<TIMER_1>::handle_overflow(); }

ISR(TIMER1_COMPA_vect) { TimerInterrupt<TIMER_1>::handle_compare_match(); }
