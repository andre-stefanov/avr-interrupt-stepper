#include <unity.h>

#include "mocks/PinMock.h"
#include "mocks/DriverMock.h"
#include "mocks/IntervalInterruptMock.h"
#include "AccelerationRamp.h"
#include "Stepper.h"

constexpr auto MAX_SPEED = Angle::deg(100.0f);

// With SPR of 36.000 one step of the stepper is exactly 0.01°
// Rotating at 100°/s (MAX_SPEED) will result in interval of F_CPU / 10000
constexpr auto SPR = 36000LU;

namespace mocks
{
    using pin_step = PinMock<1>;
    using pin_dir = PinMock<2>;

    using driver = DriverMock<SPR, pin_step::REAL_TYPE, pin_dir::REAL_TYPE>;

    using timer_interrupt = IntervalInterruptMock<Timer::TIMER_TEST>;
}

using stepper = Stepper<mocks::timer_interrupt::REAL_TYPE, mocks::driver::REAL_TYPE, MAX_SPEED.mrad_u32(), MAX_SPEED.mrad_u32() * 4>;

void move_and_assert_steps(Angle speed, Angle distance)
{
    mocks::timer_interrupt::reset();
    mocks::driver::reset();

    int32_t start_position = mocks::driver::position;
    int32_t steps = mocks::driver::REAL_TYPE::SPR / 360.0f * distance.deg();

    stepper::moveBy(speed, distance);

    mocks::timer_interrupt::run_until_stopped();

    // check for correct amount of steps
    TEST_ASSERT_EQUAL_INT64(steps + start_position, mocks::driver::position);

    // check for correct direction
    // bool expected_dir = (speed.rad() / abs(speed.rad())) == (distance.rad() / abs(distance.rad()));
    // TEST_ASSERT_EQUAL(expected_dir, expected_dir);
    // TEST_ASSERT_EQUAL(expected_dir, mocks::driver::cw);
}

void test_move_full_ramp()
{
    move_and_assert_steps(MAX_SPEED, Angle::deg(100.0f));
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[1], mocks::timer_interrupt::max_interval);
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::getIntervalForSpeed(MAX_SPEED.rad()), mocks::timer_interrupt::min_interval);
}

void test_move_full_ramp_half_speed()
{
    move_and_assert_steps(MAX_SPEED / 2, Angle::deg(45.0f));
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[1], mocks::timer_interrupt::max_interval);
    TEST_ASSERT_GREATER_THAN_UINT32(stepper::Ramp::intervals[stepper::Ramp::STAIRS_COUNT - 1], mocks::timer_interrupt::min_interval);
}

void test_move_full_ramp_odd()
{
    move_and_assert_steps(MAX_SPEED, Angle::deg(100.01f));
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[1], mocks::timer_interrupt::max_interval);
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::getIntervalForSpeed(MAX_SPEED.rad()), mocks::timer_interrupt::min_interval);
}

void test_move_full_ramp_half_speed_odd()
{
    move_and_assert_steps(MAX_SPEED / 2, Angle::deg(45.01f));
    TEST_ASSERT_EQUAL_UINT32(stepper::Ramp::intervals[1], mocks::timer_interrupt::max_interval);
    TEST_ASSERT_GREATER_THAN_UINT32(stepper::Ramp::intervals[stepper::Ramp::STAIRS_COUNT - 1], mocks::timer_interrupt::min_interval);
}

void test_move_part_ramp_even()
{
    move_and_assert_steps(MAX_SPEED, Angle::deg(1.0f));
    TEST_ASSERT_GREATER_THAN_UINT32(stepper::Ramp::getIntervalForSpeed(MAX_SPEED.rad()), mocks::timer_interrupt::min_interval);
}

void test_move_part_ramp_odd()
{
    move_and_assert_steps(MAX_SPEED, Angle::deg(1.01f));
}

void test_move_1_step()
{
    move_and_assert_steps(MAX_SPEED, Angle::deg(0.01f));
}

void test_move_1_step_ccw()
{
    move_and_assert_steps(MAX_SPEED, -Angle::deg(0.01f));
}

void test_MovementSpec_max_speed()
{
    auto spec = stepper::MovementSpec(MAX_SPEED, Angle::deg(100.0f));
    using ramp = stepper::Ramp;
    TEST_ASSERT_EQUAL_UINT32(1600, spec.run_interval);
    TEST_ASSERT_EQUAL_UINT32(ramp::STAIRS_COUNT - 1, spec.full_accel_stairs);
    TEST_ASSERT_GREATER_OR_EQUAL_UINT32(ramp::getIntervalForSpeed(MAX_SPEED.rad()), stepper::Ramp::intervals[spec.full_accel_stairs]);
}

void test_Stepper()
{
    UNITY_BEGIN();
    RUN_TEST(test_move_full_ramp);
    RUN_TEST(test_move_full_ramp_half_speed);
    RUN_TEST(test_move_full_ramp_odd);
    RUN_TEST(test_move_full_ramp_half_speed_odd);
    RUN_TEST(test_move_part_ramp_even);
    RUN_TEST(test_move_part_ramp_odd);
    RUN_TEST(test_move_1_step);
    RUN_TEST(test_move_1_step_ccw);
    RUN_TEST(test_MovementSpec_max_speed);
    UNITY_END();
}