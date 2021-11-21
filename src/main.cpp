#define SPR (400LL * 64LL)
#define SIDEREAL (86164.0905f)
#define TRANSMISSION (35.46611505122143)

#include <Arduino.h>

#include "FastPin.h"
#include "TimerInterrupt.h"
#include "InterruptStepper.h"

#define RA_RAMP_LEVELS 64

#define PIN_STEP 53

void step()
{
	FastPin<PIN_STEP>::pulse();
	// digitalWrite(PIN_STEP, HIGH);
	// digitalWrite(PIN_STEP, LOW);
}

using InterruptRA = TimerInterrupt<TIMER_1>;

namespace axis {
	typedef InterruptStepper<InterruptRA, RA_RAMP_LEVELS> ra;
}

void setup()
{
	// FastPin<60>::output();
	FastPin<PIN_STEP>::output();
	
	// FastPin<38>::output();
	// FastPin<38>::low();

	InterruptRA::init();
	// InterruptRA::setCallback(step);
	// InterruptRA::setFrequency(TRANSMISSION * SPR / SIDEREAL);

	// tracking speed in deg
	float tracking_speed = TRANSMISSION * SPR / SIDEREAL;
	// slew at full stepper speed (600 RPM)
	float slewing_speed = DEG_TO_RAD * 4 * TRANSMISSION * 8;

	axis::ra::init(slewing_speed, slewing_speed * 2);
	// axis::ra::move()
	axis::ra::move(SPR * 10LL, DEG_TO_RAD * slewing_speed);

	// InterruptStepper<64>::moveAccelerated(static_cast<uint64_t>(SPR) * 1, slewing_speed);
}

void loop()
{
	while (1)
	{
		// nothing to do here, we want to test the interrupts
	}
}

// int main(void)
// {
// 	//InterruptStepper::init();
// 	FastPin<52>::output();

// 	isr1.init();
// 	isr1.setCallback(FInterruptStepper::interrupt_handler);

// 	//uint32_t speed = static_cast<long>(0.0174533 * 360 * 100 * 1 + 0.5);
// 	//float speed = 0.0174533 * 360 * 1000000; // 1 rotation per second
// 	//float earth_speed = TRANSMISSION * EARTH_SPEED; // tracking speed
// 	float slewing_speed = 2.476001922139725;
// 	FInterruptStepper::init(slewing_speed, slewing_speed * 2);
// 	//FInterruptStepper::move(400LL * 8 * 100, slewing_speed);
// 	FInterruptStepper::moveAccelerated(400LL * 8 * 10, slewing_speed);
// 	//InterruptStepper::move(400ULL * 8 * 100, speed * 2, speed);

// 	while (1)
// 	{
// 		//FastPin<52>::pulse();
// 	}
// }
