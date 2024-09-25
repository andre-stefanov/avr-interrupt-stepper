#include <utility>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <cmath>

#include "oat/Configuration.h"

#include "gmocks/MockedIntervalInterrupt.h"
#include "gmocks/MockedDriver.h"
#include "gmocks/MockedAccelerationRamp.h"
#include "Stepper.h"

#define SIGN(x) ((x >= 0) ? 1 : -1)

#define RA_DEG_TO_STEPS(deg) ((deg / 360.f) * RA_STEPPER_SPR * RA_MICROSTEPPING * RA_TRANSMISSION)

constexpr auto ra_full_steps = RA_DEG_TO_STEPS(360.f);

constexpr auto TRACKING_SPEED = (RA_DEG_TO_STEPS(360.f) / SIDEREAL_SECONDS_PER_DAY);
constexpr auto SLEWING_SPEED = (RA_DEG_TO_STEPS(RA_SLEWING_SPEED));
constexpr auto ACCELERATION = (RA_DEG_TO_STEPS(RA_SLEWING_ACCELERATION));

using Interrupt = MockedIntervalInterrupt<0>;
using Driver = MockedDriver<RA_STEPPER_SPR * RA_MICROSTEPPING>;
using Ramp = MockedAccelerationRamp<
    256,
    F_CPU,
    static_cast<uint32_t>(SLEWING_SPEED),
    static_cast<uint32_t>(ACCELERATION)>;

using TestStepper = Stepper<Interrupt, Driver, Ramp>;

struct SimpleStepperTest : public testing::Test
{
  void SetUp() override
  {
    Interrupt::mock = new StrictMock<IntervalInterruptMock>();
    Driver::mock = new StrictMock<DriverMock>();
    Ramp::mock = new StrictMock<RampMock>();

    Ramp::delegateToReal();
    Ramp::expectLimits();

    EXPECT_CALL(*Interrupt::mock, setCallback(_)).Times(AnyNumber());
  }

  void TearDown() override
  {
    delete Interrupt::mock;
    delete Driver::mock;
    delete Ramp::mock;

    TestStepper::reset();
  }
};

TEST_F(SimpleStepperTest, moveTo)
{
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());

  EXPECT_CALL(*Driver::mock, dir(false)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  std::cout << "Moving with -5.33 sps" << std::endl;
  TestStepper::moveTo(-5.33, INT32_MAX);

  Interrupt::loopUntilStopped(5, false);

  std::cout << "stop" << std::endl;
  TestStepper::stop();

  EXPECT_EQ(Driver::position, -5);

  EXPECT_CALL(*Driver::mock, dir(false)).Times(1);

  std::cout << "Moving with -5.33 sps" << std::endl;
  TestStepper::moveTo(-5.33, INT32_MAX);

  Interrupt::loopUntilStopped(5, false);

  std::cout << "stop" << std::endl;
  TestStepper::stop();

  EXPECT_EQ(Driver::position, -10);
}
