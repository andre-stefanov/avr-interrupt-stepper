#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Stepper.h"
#include "gmocks/MockedIntervalInterrupt.h"
#include "gmocks/MockedDriver.h"
#include "gmocks/MockedAccelerationRamp.h"

using namespace ::testing;

template <typename T_Interrupt, typename T_Driver, typename T_Ramp>
struct FixtureParams
{
    using Interrupt = T_Interrupt;
    using Driver = T_Driver;
    using Ramp = T_Ramp;
};

template <typename T_Params>
struct StepperTest : public Test
{
protected:
    void SetUp() override
    {
        // Interrupt::mock = new IntervalInterruptMock();
        // Driver::mock = new DriverMock();
        // Ramp::mock = new RampMock();

        Interrupt::mock = new NiceMock<IntervalInterruptMock>();
        Driver::mock = new DriverMock();
        Ramp::mock = new NiceMock<RampMock>();
    }

    void TearDown() override
    {
        delete Interrupt::mock;
        delete Driver::mock;
        delete Ramp::mock;
    }

public:
    using Interrupt = typename T_Params::Interrupt;
    using Driver = typename T_Params::Driver;
    using Ramp = typename T_Params::Ramp;

    using stepper_t = Stepper<Interrupt, Driver, Ramp>;
};

constexpr auto SPR = 400UL * 256UL;
constexpr auto STEP_ANGLE = Angle::deg(360.0f) / SPR;
constexpr auto TRANSMISSION = 35.46611505122143f;
constexpr auto TRACKING_SPEED = Angle::deg(360.0f) / 86164.0905f;
constexpr auto SLEWING_SPEED = TRANSMISSION * Angle::deg(2.0f);
constexpr auto ACCELERATION = TRANSMISSION * Angle::deg(4.0f);

using TestInterrupt = MockedIntervalInterrupt<0>;
using TestDriver = MockedDriver<SPR>;
using TestRamp = MockedAccelerationRamp<64, F_CPU, SPR, SLEWING_SPEED.mrad_u32(), ACCELERATION.mrad_u32()>;
using TestTypes = ::testing::Types<FixtureParams<TestInterrupt, TestDriver, TestRamp>>;

TYPED_TEST_SUITE(StepperTest, TestTypes);

TYPED_TEST(StepperTest, ZeroStep)
{
    TestFixture::Ramp::delegateToReal();
    TestFixture::Ramp::expectLimits();

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
    // EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(Angle::deg(1.0f), 0);

    TestFixture::Interrupt::loopUntilStopped();
}

// TYPED_TEST(StepperTest, OneStepCW)
// {
//     TestFixture::Ramp::delegateToReal();
//     TestFixture::Ramp::expectLimits();

//     EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
//     EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

//     TestFixture::stepper_t::moveBy(Angle::deg(1.0f), 1);

//     TestFixture::Interrupt::loopUntilStopped();
// }

// TYPED_TEST(StepperTest, OneStepCCW)
// {
//     TestFixture::Ramp::delegateToReal();
//     TestFixture::Ramp::expectLimits();

//     EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
//     EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

//     TestFixture::stepper_t::moveBy(Angle::deg(-1.0f), 1);

//     TestFixture::Interrupt::loopUntilStopped();
// }

// TYPED_TEST(StepperTest, FullRamp)
// {
//     TestFixture::Ramp::delegateToReal();
//     TestFixture::Ramp::expectLimits();

//     constexpr auto DISTANCE = SLEWING_SPEED * 2;
//     constexpr auto EXPECTED_STEPS = static_cast<uint32_t>(DISTANCE / STEP_ANGLE + 0.5);

//     EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(EXPECTED_STEPS);
//     EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

//     EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(_)).Times(AnyNumber());
//     EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(0)).Times(0);

//     TestFixture::stepper_t::moveBy(SLEWING_SPEED + (SLEWING_SPEED * 0.01), DISTANCE);

//     TestFixture::Interrupt::loopUntilStopped();
// }

// TYPED_TEST(StepperTest, FullRamp_CCW_WhileSlowMovement)
// {
//     TestFixture::Ramp::delegateToReal();
//     TestFixture::Ramp::expectLimits();

//     constexpr auto TRACKING_STEPS = 100;

//     constexpr auto DISTANCE = SLEWING_SPEED * 2;
//     constexpr auto SLEWING_STEPS = static_cast<uint32_t>(DISTANCE / STEP_ANGLE + 0.5);

//     EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(_)).Times(AnyNumber());
//     EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(0)).Times(0);

//     EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(TRACKING_STEPS);
//     EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

//     TestFixture::stepper_t::moveBy(TRACKING_SPEED, UINT32_MAX);

//     TestFixture::Interrupt::loop(TRACKING_STEPS);

//     Mock::VerifyAndClearExpectations(TestFixture::Driver::mock);

//     EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(SLEWING_STEPS);
//     EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

//     TestFixture::stepper_t::moveBy(-SLEWING_SPEED + TRACKING_SPEED, DISTANCE);

//     TestFixture::Interrupt::loopUntilStopped();
// }