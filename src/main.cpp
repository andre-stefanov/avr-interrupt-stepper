#define USE_STEPPER_TIMER_1

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
    // constexpr uint32_t SPR = 36000UL;
    constexpr Angle TRACKING = Angle::deg(360.0f / SIDEREAL) * TRANSMISSION;
    constexpr Angle SLEWING_SPEED = Angle::deg(4.0f) * TRANSMISSION;

    using pinStep = Pin<46>;
    using pinDir = Pin<47>;
    using interrupt = TimerInterrupt<Timer::TIMER_1>;
    using driver = Driver<SPR, pinStep, pinDir>;
    using stepper = Stepper<interrupt, driver, SLEWING_SPEED.mrad_u32(), 4 * SLEWING_SPEED.mrad_u32()>;
  }
}

using namespace axis::ra;

stepper::MovementSpec specs[] = {
    stepper::MovementSpec(SLEWING_SPEED, Angle::deg(45.0f)),  // full ramp, full speed
    // stepper::MovementSpec(SLEWING_SPEED, Angle::deg(10.0f)),  // part. ramp, full speed
    // stepper::MovementSpec(SLEWING_SPEED, Angle::deg(-10.0f)), // part. ramp, full speed, inv. dir
    // stepper::MovementSpec(SLEWING_SPEED / 4, Angle::deg(10.0f)),
    // stepper::MovementSpec(SLEWING_SPEED.rad() / 4, 100),
    // stepper::MovementSpec(TRACKING, Angle::deg(0.1)),
    // stepper::MovementSpec(TRACKING.rad() * 0.5, 5),
    // stepper::MovementSpec(TRACKING.rad(), 5),
    // stepper::MovementSpec(TRACKING.rad() * 1.5, 5),
    // stepper::MovementSpec(TRACKING.rad(), 5),
    // stepper::MovementSpec(TRACKING, Angle::deg(0)),
    // stepper::MovementSpec(SLEWING_SPEED, Angle::deg(-360.0 / driver::SPR * 1001)),
};

unsigned int started = 0;
volatile unsigned int completed = 0;
volatile unsigned long next_start = 0;

void onComplete()
{
  completed++;
  // next_start = millis() + 100;
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
  // stepper::move(SLEWING_SPEED / 2, Angle::deg(90.0f));
  // delay(500);
  // stepper::move(SLEWING_SPEED, Angle::deg(45.0f));
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

#include <string>
#include <iostream>
#include <stdio.h>

float position = 0;
uint32_t time_ticks = 0;

FILE * file = nullptr;

void dump_snapshot()
{
  using namespace std;
  fprintf(file, "%u, %.4f\n", time_ticks, position);
  // cout << time_ticks << ",";
  // cout << position << ",";
  // cout << endl;
}

int main(int argc, char const *argv[])
{
  uint32_t interval = UINT32_MAX;
  TimerInterrupt_Delegate<Timer::TIMER_3>::setInterval = [&interval](uint32_t i)
  { interval = i; };
  TimerInterrupt_Delegate<Timer::TIMER_3>::stop = [&interval]()
  { interval = UINT32_MAX; };

  timer_callback callback = nullptr;
  TimerInterrupt_Delegate<Timer::TIMER_3>::setCallback = [&callback](timer_callback fn)
  { callback = fn; };

  while (true)
  {
    if (started < std::size(specs) && completed == started)
    {
      time_ticks = 0;

      auto filename = "data/movement_" + std::to_string(started) + ".csv";
      file = fopen(filename.c_str(), "w");
      std::cout << "Starting " << started << std::endl;
      stepper::move(specs[started], StepperCallback::create<onComplete>());
      started++;
    }
    else if (completed == std::size(specs))
    {
      break;
    }
    else if (interval != UINT32_MAX)
    {
      time_ticks += interval;
      if (callback == nullptr)
      {
        std::cerr << "called null callback" << std::endl;
        return 1;
      }
      callback();
      dump_snapshot();
    }
  }

  return 0;
}

template <uint8_t PIN>
void Pin<PIN>::init()
{
}

template <>
void inline __attribute__((always_inline)) Pin<46>::pulse()
{
  position += 360.0 / driver::SPR;
}

template <>
void inline __attribute__((always_inline)) Pin<47>::pulse()
{
}

template <uint8_t PIN>
void inline __attribute__((always_inline)) Pin<PIN>::high()
{
}

template <uint8_t PIN>
void inline __attribute__((always_inline)) Pin<PIN>::low()
{
}

#endif
