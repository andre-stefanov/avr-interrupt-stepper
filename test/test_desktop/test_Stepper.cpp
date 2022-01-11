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

    using driver = DriverMock<400U*256U, pin_step::REAL_TYPE, pin_dir::REAL_TYPE>;

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
    mocks::timer_interrupt::reset();
    move_and_assert(SLEWING_SPEED.rad(), 10000);
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[0], mocks::timer_interrupt::max_interval);
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[stepper::Ramp::STAIRS_COUNT - 1], mocks::timer_interrupt::min_interval);
}

void test_move_full_ramp_half_speed()
{
    mocks::timer_interrupt::reset();
    move_and_assert(SLEWING_SPEED.rad() / 2, 10000);
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[0], mocks::timer_interrupt::max_interval);
    TEST_ASSERT_GREATER_THAN_UINT32(stepper::Ramp::intervals[stepper::Ramp::STAIRS_COUNT - 1], mocks::timer_interrupt::min_interval);
}

void test_move_zero_steps()
{
    mocks::timer_interrupt::reset();
    move_and_assert(SLEWING_SPEED.rad(), 0);
}

void test_MovementSpec_run_interval()
{
    constexpr auto spec = stepper::MovementSpec(SLEWING_SPEED.rad());
}

void test_Stepper()
{
    UNITY_BEGIN();
    RUN_TEST(test_move_full_ramp);
    RUN_TEST(test_move_full_ramp_half_speed);
    RUN_TEST(test_move_zero_steps);
    UNITY_END();
}