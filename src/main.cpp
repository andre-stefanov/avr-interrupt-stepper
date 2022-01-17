#include "Pin.h"
#include "Driver.h"
#include "TimerInterrupt.h"
#include "Stepper.h"
#include "Angle.h"

#include "utils.h"

#include <limits.h>

constexpr auto SIDEREAL = 86164.0905f;
namespace axis
{
  namespace ra
  {
    constexpr float TRANSMISSION = 35.46611505122143f;

    constexpr uint32_t SPR = 400UL * 256UL;
    constexpr Angle TRACKING = Angle::from_deg(360.0f / SIDEREAL) * TRANSMISSION;
    constexpr Angle SLEWING_SPEED = Angle::from_deg(2.0f) * TRANSMISSION;

    using pinStep = Pin<46>;
    using pinDir = Pin<47>;
    using interrupt = TimerInterrupt<Timer::TIMER_3>;
    using driver = Driver<SPR, pinStep, pinDir>;
    using stepper = Stepper<interrupt, driver, SLEWING_SPEED.mrad_u32(), 2 * SLEWING_SPEED.mrad_u32()>;
  }
}

using namespace axis::ra;

void finish()
{
  Pin<53>::pulse();
}

stepper::MovementSpec specs[] = {
    stepper::MovementSpec(SLEWING_SPEED, Angle::from_deg(45.0f)), // full ramp
    stepper::MovementSpec(SLEWING_SPEED, Angle::from_deg(10.0f)),
    stepper::MovementSpec(SLEWING_SPEED, Angle::from_deg(-10.0f)),
    // stepper::MovementSpec(SLEWING_SPEED.rad() / 4, 1000),
    // stepper::MovementSpec(SLEWING_SPEED.rad() / 4, 100),
    // stepper::MovementSpec(TRACKING, Angle::from_deg(0.1)),
    // stepper::MovementSpec(TRACKING.rad() * 0.5, 5),
    // stepper::MovementSpec(TRACKING.rad(), 5),
    // stepper::MovementSpec(TRACKING.rad() * 1.5, 5),
    // stepper::MovementSpec(TRACKING.rad(), 5),
    // stepper::MovementSpec(TRACKING, Angle::from_deg(0)),
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

  Pin<DEBUG_SETUP_TIMING_PIN>::high();

  interrupt::init();

  Pin<DEBUG_SETUP_TIMING_PIN>::low();
}

void loop()
{
  stepper::move(SLEWING_SPEED, Angle::from_deg(90.0f));
  delay(500);
  stepper::move(SLEWING_SPEED, Angle::from_deg(1.0f));
  do
  {
    // nothing
  } while (true);

  // do
  // {
  //   Pin<DEBUG_LOOP_TIMING_PIN>::high();

  //   if (started < (sizeof specs / sizeof specs[0]) && completed == started && millis() > next_start)
  //   {
  //     // Serial.print("Starting movement ");
  //     // Serial.println(started);
  //     // stepper::move(specs[started], onComplete);
  //     stepper::move(specs[started], StepperCallback::create<onComplete>());
  //     started++;
  //   }

  //   Pin<DEBUG_LOOP_TIMING_PIN>::low();
  // } while (1);
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
