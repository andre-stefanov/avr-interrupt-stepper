#define SIDEREAL 86164.0905f
#define TRANSMISSION 35.46611505122143

#define TRACKING_DEG_PER_SEC (360 / SIDEREAL * TRANSMISSION)

#define SLEWING_DEG_PER_SEC (4 * TRANSMISSION)

#include "TimerInterrupt.h"
#include "AccelerationRamp.h"
#include "Pin.h"
#include "Driver.h"
#include "Stepper.h"

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
        (uint32_t)(4 * SLEWING_DEG_PER_SEC)>;
  }
}

void setup()
{
  Serial.begin(115200);
  // delay(1000);

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

  // axis::ra::stepper::move(TRACKING_DEG_PER_SEC, 10);

  axis::ra::stepper::move(SLEWING_DEG_PER_SEC, 3000);
  delay(600);
  axis::ra::stepper::move(SLEWING_DEG_PER_SEC, 1000);
  delay(400);
  axis::ra::stepper::move(SLEWING_DEG_PER_SEC / 4, 1000);
  delay(500);
  axis::ra::stepper::move(SLEWING_DEG_PER_SEC / 4, 100);
  delay(200);
  axis::ra::stepper::move(TRACKING_DEG_PER_SEC, 10);

  // axis::ra::stepper::move(SLEWING_DEG_PER_SEC, 3000);
  // delay(200);
  // axis::ra::stepper::move(SLEWING_DEG_PER_SEC / 4, 3000);

  Pin<DEBUG_SETUP_TIMING_PIN>::low();
}

void loop()
{
  do
  {
    Pin<DEBUG_LOOP_TIMING_PIN>::pulse();
  } while (1);
}

ISR(TIMER1_OVF_vect) { TimerInterrupt<TIMER_1>::handle_overflow(); }
ISR(TIMER1_COMPA_vect) { TimerInterrupt<TIMER_1>::handle_compare_match(); }
