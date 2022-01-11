#include "Pin.h"
#include "Driver.h"
#include "TimerInterrupt.h"
#include "Stepper.h"

#include "utils.h"

constexpr auto SIDEREAL = 86164.0905f;
namespace axis
{
  namespace ra
  {
    constexpr float TRANSMISSION = 35.46611505122143f;
    
    constexpr uint32_t SPR = 400U * 128U;
    constexpr Angle TRACKING = Angle::from_deg(360.0f / SIDEREAL) * TRANSMISSION;
    constexpr Angle SLEWING = Angle::from_deg(4.0f) * TRANSMISSION;

    using pinStep = Pin<46>;
    using pinDir = Pin<47>;
    using interrupt = TimerInterrupt<Timer::TIMER_3>;
    using driver = Driver<SPR, pinStep, pinDir>;
    using stepper = Stepper<
        interrupt,
        driver,
        SLEWING.mrad_u32(),
        4 * SLEWING.mrad_u32()>;
  }
}

using namespace axis::ra;

stepper::MovementSpec specs[] = {
    stepper::MovementSpec(SLEWING.rad(), 10000),
    // stepper::MovementSpec(SLEWING.rad(), 1000),
    // stepper::MovementSpec(SLEWING.rad() / 4, 1000),
    // stepper::MovementSpec(SLEWING.rad() / 4, 100),
    // stepper::MovementSpec(TRACKING.rad(), 5),
    // stepper::MovementSpec(TRACKING.rad() * 0.5, 5),
    // stepper::MovementSpec(TRACKING.rad(), 5),
    // stepper::MovementSpec(TRACKING.rad() * 1.5, 5),
    // stepper::MovementSpec(TRACKING.rad(), 5),
};

unsigned int started = 0;
volatile unsigned int completed = 0;
volatile unsigned long next_start = 0;

unsigned long int millis();

void onComplete()
{
  completed++;
  next_start = millis() + 100;
}

#ifdef ARDUINO
#include <Arduino.h>

#define DEBUG_LOOP_TIMING_PIN 50
#define DEBUG_SETUP_TIMING_PIN 51

void setup()
{
  pinStep::init();
  pinDir::init();
  Pin<48>::init();
  Pin<49>::init();
  Pin<50>::init();
  Pin<51>::init();
  Pin<52>::init();
  Pin<53>::init();

  Serial.begin(115200);

  // for (size_t i = 0; i < stepper::Ramp::STAIRS_COUNT; i++)
  // {
  //   Serial.println(stepper::Ramp::intervals[i]);
  // }

  Pin<DEBUG_SETUP_TIMING_PIN>::high();

  // auto x = NewtonRaphson::sqrt(2.0f * stepper::Ramp::STEP_ANGLE / stepper::Ramp::UTIL_ACCELERATION_RAD);
  // auto y = constexpr_sqrt(2.0f * stepper::Ramp::STEP_ANGLE / stepper::Ramp::UTIL_ACCELERATION_RAD);
  // Serial.println(x, 10);
  // Serial.println(y, 10);
  interrupt::init();

  Pin<DEBUG_SETUP_TIMING_PIN>::low();
}

void loop()
{
  // stepper::move(stepper::MovementSpec(DEG_TO_RAD * SLEWING_mRAD_PER_SEC / 4, 1000));
  // delay(420);
  // Pin<46>::init();
  // stepper::move(stepper::MovementSpec(DEG_TO_RAD * SLEWING_mRAD_PER_SEC, 3000));
  // do
  // {
  //   // nothing
  // } while (true);

  do
  {
    Pin<DEBUG_LOOP_TIMING_PIN>::high();

    if (started < (sizeof specs / sizeof specs[0]) && completed == started && millis() > next_start)
    {
      // Serial.print("Starting movement ");
      // Serial.println(started);
      // stepper::move(specs[started], onComplete);
      stepper::move(specs[started], StepperCallback::create<onComplete>());
      started++;
    }

    Pin<DEBUG_LOOP_TIMING_PIN>::low();
  } while (1);
}

#else

#include <chrono>
#include <iostream>

using namespace std::chrono;
unsigned long int millis()
{
  return duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
}

int main(int argc, char const *argv[])
{
  while (1)
  {
    if (started < (sizeof specs / sizeof specs[0]) && completed == started && millis() > next_start)
    {
      stepper::move(specs[started], StepperCallback::create<onComplete>());
      started++;
    }
  }

  return 0;
}

#endif
