#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Stepper.h"
#include "gmocks/MockedIntervalInterrupt.h"
#include "gmocks/MockedDriver.h"
#include "gmocks/MockedAccelerationRamp.h"

using namespace ::testing;

template<typename T_Interrupt, typename T_Driver, typename T_Ramp>
struct FixtureParams {
    using Interrupt = T_Interrupt;
    using Driver = T_Driver;
    using Ramp = T_Ramp;
};

template<typename T_Params>
struct StepperTest : public Test {
protected:
    void SetUp() override {
        Interrupt::mock = new StrictMock<IntervalInterruptMock>();
        Driver::mock = new StrictMock<DriverMock>();
        Ramp::mock = new StrictMock<RampMock>();

        Ramp::delegateToReal();
        Ramp::expectLimits();

        EXPECT_CALL(*Interrupt::mock, setCallback(_)).Times(AnyNumber());
    }

    void TearDown() override {
        delete Interrupt::mock;
        delete Driver::mock;
        delete Ramp::mock;
    }

public:
    using Interrupt = typename T_Params::Interrupt;
    using Driver = typename T_Params::Driver;
    using Ramp = typename T_Params::Ramp;

    using stepper_t = Stepper<Interrupt, Driver, Ramp>;

    static constexpr Angle speedAtStair(uint16_t stair)
    {
        return stepper_t::ANGLE_PER_STEP * (static_cast<float>(F_CPU) / Ramp::REAL_TYPE::interval(stair));
    }

    // Speed not requiring any acceleration
    static constexpr Angle speed_slow = speedAtStair(1) / 10.0f;

    // Maximal speed of the ramp (last stair)
    static constexpr Angle speed_max = speedAtStair(Ramp::REAL_TYPE::STAIRS_COUNT - 1);

    static void inline prepareSpeed(Angle speed)
    {
        const uint16_t max_stair = Ramp::maxAccelStairs(abs(speed.rad()));
        const uint32_t steps = static_cast<uint32_t>(max_stair) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);

        EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, dir(speed.rad() >= 0.0f)).Times(1);

        if (max_stair > 0)
        {
            for (uint32_t stair = 1; stair <= max_stair; stair++) {
                uint32_t interval = Ramp::REAL_TYPE::interval(stair);
                EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
            }
        }

        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(speed.rad()))).RetiresOnSaturation();

        EXPECT_CALL(*Driver::mock, step()).Times(steps).RetiresOnSaturation();

        stepper_t::moveTo(speed, INT32_MAX);
        Interrupt::loopUntilStopped(steps, false);
    }
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

class StepperTestNames {
public:
    template<typename T>
    static std::string GetName(int i) {
        switch (i) {
            case 0:
                return "OAT_RA";
            default:
                return std::to_string(i);
        }
    }
};

TYPED_TEST_SUITE(StepperTest, TestTypes, StepperTestNames);

TYPED_TEST(StepperTest, moveBy_idle_0steps_slow_cw) {
    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(nullptr)).Times(1);
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(0);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(_)).Times(0);

    TestFixture::stepper_t::moveBy(TestFixture::speed_slow, 0U);
    TestFixture::Interrupt::loopUntilStopped(1);
}

TYPED_TEST(StepperTest, moveBy_idle_0steps_slow_ccw) {
    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(nullptr)).Times(1);
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(0);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(_)).Times(0);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_slow, 0);
    TestFixture::Interrupt::loopUntilStopped(1);
}

TYPED_TEST(StepperTest, moveBy_idle_1step_slow_cw) {
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(TestFixture::speed_slow, 1);

    TestFixture::Interrupt::loopUntilStopped(1);
}

TYPED_TEST(StepperTest, moveBy_idle_1step_slow_ccw) {
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_slow, 1);

    TestFixture::Interrupt::loopUntilStopped(1);
}

TYPED_TEST(StepperTest, moveBy_idle_360deg_slow_cw) {
    constexpr auto expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(TestFixture::speed_slow.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(2);
    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval)).Times(1);
    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(TestFixture::speed_slow, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);

    EXPECT_EQ(TestFixture::stepper_t::position().mrad_u32(), Angle::deg(360.0f).mrad_u32());
}

TYPED_TEST(StepperTest, moveBy_idle_360deg_slow_ccw) {
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

TYPED_TEST(StepperTest, moveBy_idle_360deg_max_cw) {
    constexpr auto expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) /
                                                              TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr auto expected_speed = TestFixture::speed_max;
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(expected_speed.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(TestFixture::speed_max, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps * 2);
}

TYPED_TEST(StepperTest, moveBy_idle_360deg_max_ccw) {
    constexpr auto expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) /
                                                              TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(
            TestFixture::speed_max.rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-TestFixture::speed_max, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_idle_360deg_2max_cw) {
    constexpr auto expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) / TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr auto expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(
            (2.0f * TestFixture::speed_max).rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i)));
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(2.0f * TestFixture::speed_max, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_idle_360deg_2max_ccw) {
    constexpr auto expected_steps = static_cast<uint32_t>(Angle::deg(360.0f) /
                                                              TestFixture::stepper_t::ANGLE_PER_STEP);
    constexpr uint32_t expected_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(
            (2.0f * TestFixture::speed_max).rad());

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
    EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(AtLeast(1));

    {
        InSequence s;

        for (size_t i = 1; i < TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT; i++) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i))).Times(1);
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(expected_interval));

        for (size_t i = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT - 1; i > 0; i--) {
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(TestFixture::Ramp::REAL_TYPE::interval(i))).Times(1);
        }
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(expected_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(false)).Times(1);

    TestFixture::stepper_t::moveBy(-2.0f * TestFixture::speed_max, Angle::deg(360.0f));

    TestFixture::Interrupt::loopUntilStopped(expected_steps);
}

TYPED_TEST(StepperTest, moveBy_idle_short_cw_max) {
    constexpr uint32_t stairs = TestFixture::Ramp::REAL_TYPE::STAIRS_COUNT / 2;
    constexpr uint32_t accel_steps = stairs * TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR;
    constexpr uint32_t run_steps = TestFixture::Ramp::REAL_TYPE::STEPS_PER_STAIR / 2;

    constexpr uint32_t test_steps = accel_steps * 2 + run_steps;
    constexpr auto test_speed = TestFixture::speed_max;

    {
        InSequence s;

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(1).RetiresOnSaturation();

        // acceleration
        for (size_t i = 1; i <= stairs; i++) {
            uint32_t interval = TestFixture::Ramp::REAL_TYPE::interval(i);

            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
        }

        // run
        uint32_t run_interval = TestFixture::Ramp::REAL_TYPE::interval(stairs);
        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(run_interval)).RetiresOnSaturation();

        // deceleration
        for (size_t i = stairs; i > 0; i--) {
            uint32_t interval = TestFixture::Ramp::REAL_TYPE::interval(i);
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).Times(1).RetiresOnSaturation();
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(test_steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true)).Times(1);

    TestFixture::stepper_t::moveBy(test_speed, test_steps);

    TestFixture::Interrupt::loopUntilStopped(test_steps);
}

TYPED_TEST(StepperTest, moveBy_runningAtMaxCw_slowCw) {
    const auto speed_before = TestFixture::speed_max;
    const auto stair_before = TestFixture::Ramp::REAL_TYPE::maxAccelStairs(speed_before.rad());

    TestFixture::prepareSpeed(speed_before);

    const auto speed_test = TestFixture::speed_slow;
    const uint32_t steps = (stair_before * TestFixture::Ramp::STEPS_PER_STAIR) + 100;

//    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());
//    EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(_)).Times(AnyNumber());

    {
        InSequence s;

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).RetiresOnSaturation();

        // pre-deceleration
        for (size_t stair = stair_before; stair > 0; stair--) {
            uint32_t interval = TestFixture::Ramp::REAL_TYPE::interval(stair);
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
        }

        // run
        uint32_t run_interval = TestFixture::Ramp::REAL_TYPE::getIntervalForSpeed(speed_test.rad());
        EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(run_interval)).RetiresOnSaturation();

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).RetiresOnSaturation();
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(steps);
    EXPECT_CALL(*TestFixture::Driver::mock, dir(true));

    TestFixture::stepper_t::moveBy(speed_test, steps);
    TestFixture::Interrupt::loopUntilStopped(steps);
}

TYPED_TEST(StepperTest, stop_runningSlowCw) {
    TestFixture::prepareSpeed(TestFixture::speed_slow);

    {
        InSequence s;

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).RetiresOnSaturation();
        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).RetiresOnSaturation();
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(0);

    TestFixture::stepper_t::stop();
}

TYPED_TEST(StepperTest, stop_runningMaxCw) {
    TestFixture::prepareSpeed(TestFixture::speed_max);

    const uint32_t steps = static_cast<uint32_t>(TestFixture::Ramp::STAIRS_COUNT - 1) * static_cast<uint32_t>(TestFixture::Ramp::STEPS_PER_STAIR);

    EXPECT_CALL(*TestFixture::Interrupt::mock, setCallback(_)).Times(AnyNumber());

    {
        InSequence s;

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).RetiresOnSaturation();

        for (size_t stair = TestFixture::Ramp::STAIRS_COUNT - 1; stair > 0; stair--) {
            const uint32_t interval = TestFixture::Ramp::REAL_TYPE::interval(stair);
            EXPECT_CALL(*TestFixture::Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
        }

        EXPECT_CALL(*TestFixture::Interrupt::mock, stop()).RetiresOnSaturation();
    }

    EXPECT_CALL(*TestFixture::Driver::mock, step()).Times(steps);

    TestFixture::stepper_t::stop();
    TestFixture::Interrupt::loopUntilStopped(steps, false);
}

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