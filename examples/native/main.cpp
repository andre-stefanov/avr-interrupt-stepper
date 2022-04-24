#include "Angle.h"
#include "Pin.h"
#include "Driver.h"
#include "Stepper.h"
#include "IntervalInterrupt.h"

#include <iostream>

constexpr auto SPR = 36000LU;

constexpr Angle MAX_SPEED = Angle::deg(2 * 35.0f);
constexpr Angle ACCELERATION = MAX_SPEED * 2;

using pin_step = Pin<0>;
using pin_dir = Pin<1>;

using interrupt = IntervalInterrupt<Timer::TIMER_1>;
using driver = Driver<SPR, pin_step, pin_dir, false>;
using ramp = AccelerationRamp<128, F_CPU, SPR, MAX_SPEED.mrad_u32(), ACCELERATION.mrad_u32()>;
using stepper = Stepper<interrupt, driver, ramp>;

bool finished = false;

void finish()
{
    finished = true;
}

void on_pinStateChanged(uint8_t pin, bool state)
{
    // using namespace std;
    // cout << "Pin<" << unsigned(pin) << "> = " << state << endl;
}

void on_init()
{

}

void on_setInterval(uint32_t interval)
{
    using namespace std;
    cout << "interval = " << interval << endl;
}

timer_callback timer_cb = nullptr;

void on_setCallback(timer_callback cb)
{
    timer_cb = cb;
}

void on_stop()
{

}

// template <>
// etl::delegate<void()> IntervalInterrupt_Delegate<Timer::TIMER_1>::init = etl::delegate<void()>::create<on_init>();

template <>
etl::delegate<void(uint32_t)> IntervalInterrupt_Delegate<Timer::TIMER_1>::setInterval = etl::delegate<void(uint32_t)>::create<on_setInterval>();

template <>
etl::delegate<void(timer_callback)> IntervalInterrupt_Delegate<Timer::TIMER_1>::setCallback = etl::delegate<void(timer_callback)>::create<on_setCallback>();

// template <>
// etl::delegate<void()> IntervalInterrupt_Delegate<Timer::TIMER_1>::stop = etl::delegate<void()>::create<on_stop>();

int main(int argc, char const *argv[])
{
    on_pinStateChanged(42, true);
    PinDelegate<0>::delegate(etl::delegate<void(uint8_t, bool)>::create<on_pinStateChanged>());
    PinDelegate<1>::delegate(etl::delegate<void(uint8_t, bool)>::create<on_pinStateChanged>());

    stepper::moveTime(MAX_SPEED, 500, etl::delegate<void()>::create<finish>());

    while (!finished)
    {
        timer_cb();
    }

    return 0;
}
