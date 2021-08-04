/*
* InterruptStepper.cpp
*
* Created: 10.07.2021 13:05:41
* Author : Andre
*/

#define SPR (400 * 8)
#define EARTH_SPEED (7.2921150 / 100000)
#define TRANSMISSION 35.46611505122143

#include <Arduino.h>

#include "FastPin.h"
#include "PreciseInterrupt.h"
#include "InterruptStepper.h"

void setup()
{
	FastPin<60>::output();
	FastPin<54>::output();
	FastPin<38>::output();
	FastPin<38>::low();

	isr1.init();

	// float tracking_speed = TRANSMISSION * EARTH_SPEED; // tracking speed
	float slewing_speed = DEG_TO_RAD * 4 * TRANSMISSION * 8;

	float speed = slewing_speed;
	InterruptStepper<64>::init(speed, speed * 2);
	// InterruptStepper::move(static_cast<uint64_t>(SPR) * 10, speed);
	InterruptStepper<64>::moveAccelerated(static_cast<uint64_t>(SPR) * 1, slewing_speed);
}

void loop()
{
	while (1)
	{
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
