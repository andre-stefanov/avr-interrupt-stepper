#include <unity.h>

#define F_CPU 16000000

#include "Constants.h"
#include "mocks/PinMock.h"
#include "mocks/DriverMock.h"
#include "mocks/TimerInterruptMock.h"
#include "AccelerationRamp.h"
#include "Stepper.h"

using namespace axis::ra;

namespace mocks
{
    using pin_step = PinMock<1>;
    using pin_dir = PinMock<2>;

    using driver = DriverMock<SPR, pin_step::REAL_TYPE, pin_dir::REAL_TYPE>;

    using timer_interrupt = TimerInterruptMock<Timer::TIMER_TEST>;
}

using stepper = Stepper<mocks::timer_interrupt::REAL_TYPE, mocks::driver::REAL_TYPE, SLEWING_SPEED.mrad_u32(), SLEWING_SPEED.mrad_u32() * 4>;

void move_and_assert(float speed, uint32_t steps)
{
    int32_t start_position = mocks::driver::position;

    stepper::move(speed, steps);

    mocks::timer_interrupt::run_until_stopped();

    TEST_ASSERT_EQUAL_INT64(steps + start_position, mocks::driver::position);
}

void test_move_full_ramp()
{
    move_and_assert(SLEWING_SPEED.rad(), 10000);
}

void test_move_full_ramp_half_speed()
{
    move_and_assert(SLEWING_SPEED.rad() / 2, 10000);
}

void test_Stepper()
{
    UNITY_BEGIN();
    RUN_TEST(test_move_full_ramp);
    RUN_TEST(test_move_full_ramp_half_speed);
    UNITY_END();
}