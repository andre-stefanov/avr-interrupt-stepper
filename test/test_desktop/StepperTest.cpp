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
        Interrupt::mock = new StrictMock<IntervalInterruptMock>();
        Driver::mock = new StrictMock<DriverMock>();
        Ramp::mock = new StrictMock<RampMock>();

        Ramp::delegateToReal();
        Ramp::expectLimits();
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

    // Speed not requiring any acceleration
    static constexpr Angle speed_slow = stepper_t::ANGLE_PER_STEP * (F_CPU / Ramp::REAL_TYPE::interval(1) / 10.0f);

    // Minimal speed of the ramp (first stair)
    static constexpr Angle speed_first_stair = stepper_t::ANGLE_PER_STEP * (F_CPU / Ramp::REAL_TYPE::interval(1));

    // Maximal speed of the ramp (last stair)
    static constexpr Angle speed_last_stair = stepper_t::ANGLE_PER_STEP * (F_CPU / Ramp::REAL_TYPE::interval(Ramp::REAL_TYPE::STAIRS_COUNT - 1));
};

constexpr auto SPR = 400UL * 256UL;
constexpr auto TRANSMISSION = 35.46611505122143f;
constexpr auto TRACKING_SPEED = Angle::deg(360.0f) / 86164.0905f;
constexpr auto SLEWING_SPEED = TRANSMISSION * Angle::deg(2.0f);
constexpr auto ACCELERATION = TRANSMISSION * Angle::deg(4.0f);

using OatRAInterrupt = MockedIntervalInterrupt<0>;
using OatRADriver = MockedDriver<SPR>;
using OatRARamp = MockedAccelerationRamp<128, F_CPU, SPR, SLEWING_SPEED.mrad_u32(), ACCELERATION.mrad_u32()>;
using OatRaFixture = FixtureParams<OatRAInterrupt, OatRADriver, OatRARamp>;

using TestTypes = ::testing::Types<OatRaFixture>;

class StepperTestNames
{
public:
    template <typename T>
    static std::string GetName(int i)
    {
        switch (i)
        {
        case 0:
            return "OAT_RA";
        default:
            return std::to_string(i);
        }
    }
};

TYPED_TEST_SUITE(StepperTest, TestTypes, StepperTestNames);

TYPED_TEST(StepperTest, moveBy_0steps_cw_slow)
{
    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(0);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(_)).Times(0);

    TestFixture::stepper_t::moveBy(TestFixture::speed_slow, 0);
}

TYPED_TEST(StepperTest, moveBy_0steps_ccw_slow)
{
    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(0);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(_)).Times(0);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_slow, 0);
}

TYPED_TEST(StepperTest, moveBy_1step_cw_slow)
{
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(TestFixture::speed_slow, 1);

    TestFixture::Interrupt::loopUntilStopped(1);
}

TYPED_TEST(StepperTest, moveBy_1step_ccw_slow)
{
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_slow, 1);

    TestFixture::Interrupt::loopUntilStopped(1);
}

TYPED_TEST(StepperTest, moveBy_360deg_cw_slow)
{
    constexpr auto expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(TestFixture::speed_slow, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_360deg_ccw_slow)
{
    constexpr auto expected_steps = TestFixture::Driver::SPR;
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_slow, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_360deg_cw_max)
{
    constexpr uint32_t expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_last_stair.rad());

    std::cout << expected_steps << std::endl;
    std::cout << TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT * TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR << std::endl;

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(TestFixture::speed_last_stair, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_360deg_ccw_max)
{
    constexpr uint32_t expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_last_stair.rad());

    std::cout << expected_steps << std::endl;
    std::cout << TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT * TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR << std::endl;

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_last_stair, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_360deg_cw_faster)
{
    constexpr uint32_t expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed((2.0f * TestFixture::speed_last_stair).rad());

    std::cout << expected_steps << std::endl;
    std::cout << TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT * TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR << std::endl;

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(2.0f * TestFixture::speed_last_stair, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_360deg_ccw_faster)
{
    constexpr uint32_t expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed((2.0f * TestFixture::speed_last_stair).rad());

    std::cout << expected_steps << std::endl;
    std::cout << TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT * TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR << std::endl;

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-2.0f * TestFixture::speed_last_stair, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_360deg_cw_max_short)
{
    constexpr uint32_t expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed((2.0f * TestFixture::speed_last_stair).rad());

    std::cout << expected_steps << std::endl;
    std::cout << TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT * TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR << std::endl;

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--)
        {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(2.0f * TestFixture::speed_last_stair, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

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