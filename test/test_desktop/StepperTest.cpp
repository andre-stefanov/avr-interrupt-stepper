#include <utility>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include "Angle.h"
#include "gmocks/MockedIntervalInterrupt.h"
#include "gmocks/MockedDriver.h"
#include "gmocks/MockedAccelerationRamp.h"
#include "Stepper.h"

#define SIGN(x) ((x >= 0) ? 1 : -1)

constexpr auto SPR = 400UL * 256UL;
constexpr auto TRANSMISSION = 35.46611505122143f;
constexpr auto TRACKING_SPEED = Angle::deg(360.0f) / 86164.0905f;
constexpr auto SLEWING_SPEED = TRANSMISSION * Angle::deg(8.0f);
constexpr auto ACCELERATION = TRANSMISSION * Angle::deg(8.0f);

using Interrupt = MockedIntervalInterrupt<0>;
using Driver = MockedDriver<SPR>;
using Ramp = MockedAccelerationRamp<256, F_CPU, SPR, SLEWING_SPEED.mrad_u32(), ACCELERATION.mrad_u32()>;

using TestStepper = Stepper<Interrupt, Driver, Ramp>;

template<typename T>
struct NamedValue {
    NamedValue(std::string name, T value) : name(std::move(name)), value(value) {}

    std::string name;
    T value;
};

struct Movement {
    Movement(
            uint16_t preDecelStairs,
            uint16_t accelStairs,
            uint32_t runSteps,
            uint32_t run_interval,
            uint16_t decelStairs)
            : pre_decel_stairs(preDecelStairs),
              accel_stairs(accelStairs),
              run_steps(runSteps),
              run_interval(run_interval),
              decel_stairs(decelStairs) {}

    const uint16_t pre_decel_stairs;
    const uint16_t accel_stairs;
    const uint32_t run_steps;
    const uint32_t run_interval;
    const uint16_t decel_stairs;
};

struct StepperStopTestParams {
    StepperStopTestParams(
            std::string name,
            Angle initial_speed,
            uint16_t expect_decel_stairs)
            : name(std::move(name)),
              initial_speed(initial_speed),
              expect_decel_stairs(expect_decel_stairs) {}

    std::string name;
    Angle initial_speed;
    uint16_t expect_decel_stairs;

    [[nodiscard]] uint32_t expect_steps() const {
        const uint32_t decel_stairs = expect_decel_stairs;
        const uint32_t steps_per_stair = Ramp::STEPS_PER_STAIR;
        return decel_stairs * steps_per_stair;
    }
};

template<typename T>
struct StepperTest : public testing::TestWithParam<T> {
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

        TestStepper::reset();
    }

    static void prepareSpeed(Angle speed) {
        if (speed.mrad_u32() == 0) {
            return;
        }

        const uint16_t max_stair = Ramp::maxAccelStairs(abs(speed.rad()));
        const auto steps = static_cast<int>(max_stair) * static_cast<int>(Ramp::STEPS_PER_STAIR) + 1;

        EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, dir(speed.rad() >= 0.0f)).Times(1);

        if (max_stair > 0) {
            for (uint32_t stair = 1; stair <= max_stair; stair++) {
                uint32_t interval = Ramp::REAL_TYPE::interval(stair);
                EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
            }
        }

        EXPECT_CALL(
                *Interrupt::mock,
                setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(speed.rad()))
        ).RetiresOnSaturation();

        EXPECT_CALL(*Driver::mock, step()).Times(steps).RetiresOnSaturation();

        TestStepper::moveTo(speed, INT32_MAX);
        Interrupt::loopUntilStopped(steps, false);
    }

    static constexpr Angle speedAtStair(uint16_t stair) {
        return TestStepper::ANGLE_PER_STEP *
               static_cast<float>(F_CPU) /
               static_cast<float>(Ramp::REAL_TYPE::interval(stair));
    }
};

using StepperStopTest = StepperTest<StepperStopTestParams>;

TEST_P(StepperStopTest, stop) {
    const auto p = GetParam();
    prepareSpeed(p.initial_speed);

    {
        InSequence s;

        // interrupt will be stopped immediately after/on calling Stepper::stop()
        EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();

        if (p.expect_decel_stairs > 0) {
            for (uint16_t stair = GetParam().expect_decel_stairs; stair > 0; stair--) {
                const uint32_t interval = Ramp::REAL_TYPE::interval(stair);
                EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
                EXPECT_CALL(*Driver::mock, step()).Times(Ramp::STEPS_PER_STAIR).RetiresOnSaturation();
            }
        }

        EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    }

    // TODO: assert position

    TestStepper::stop();
    Interrupt::loopUntilStopped(GetParam().expect_steps());
}

INSTANTIATE_TEST_SUITE_P(
        StepperTest,
        StepperStopTest,
        testing::Values(
                StepperStopTestParams(
                        "idle",
                        Angle::deg(0),
                        0
                ),
                StepperStopTestParams(
                        "tracking_cw",
                        TRACKING_SPEED,
                        0
                ),
                StepperStopTestParams(
                        "tracking_ccw",
                        -TRACKING_SPEED,
                        0
                ),
                StepperStopTestParams(
                        "slewing_cw",
                        SLEWING_SPEED,
                        Ramp::STAIRS_COUNT - 1
                ),
                StepperStopTestParams(
                        "slewing_ccw",
                        -SLEWING_SPEED,
                        Ramp::STAIRS_COUNT - 1
                )
        ),
        [=](const TestParamInfo<StepperStopTestParams> &i) {
            return i.param.name;
        }
);

struct StepperMoveTestParams {

    using GtestTuple = std::tuple<NamedValue<Angle>, NamedValue<Angle>, uint32_t>;

//    StepperMoveTestParams(
////            std::string name,
//            Angle initial_speed,
//            Angle run_speed,
//            uint32_t steps,
//            Movement expect) :
////            name(std::move(name)),
//            initial_speed(initial_speed),
//            run_speed(run_speed),
//            steps(steps),
//            expect(expect) {}

    explicit StepperMoveTestParams(const GtestTuple &tuple)
            : initial_speed(std::get<0>(tuple).value),
              run_speed(std::get<1>(tuple).value),
              steps(std::get<2>(tuple))
//              expect(Movement(0, 0, 0, 0, 0))
    {}

//    std::string name;
    Angle initial_speed;
    Angle run_speed;
    uint32_t steps;
//    Movement expect;

    [[nodiscard]] Movement expect() const {
        using RealRamp = Ramp::REAL_TYPE;
        uint16_t initial_ramp_stair = RealRamp::maxAccelStairs(initial_speed.rad());
        uint16_t requested_ramp_stair = RealRamp::maxAccelStairs(run_speed.rad());

//        int32_t pos_after_instant_stop =
        int32_t total_steps = static_cast<int32_t>(steps) - (initial_ramp_stair * RealRamp::STEPS_PER_STAIR);

        // pre-deceleration is needed if ...
        // 1. requested speed is at a lower ramp stair
        // 2. requested run direction differs from current one AND current stair is greater than 0
        // 3. compensation movement is needed
        uint16_t pre_decel_stairs = 0;
//        if (signbit(initial_speed.rad()) != signbit(run_speed.rad()) &&) {
//
//        }

        return {
                0,
                0,
                0,
                0,
                0
        };
    }
};

using StepperMoveByTest = StepperTest<StepperMoveTestParams::GtestTuple>;

TEST_P(StepperMoveByTest, moveBy) {
    const StepperMoveTestParams &p = StepperMoveTestParams(GetParam());

    prepareSpeed(p.initial_speed);
    auto current_stair = Ramp::maxAccelStairs(p.initial_speed.rad());

    EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
    EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
    EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

    {
        InSequence seq;

//        // interrupt will be stopped immediately after/on calling Stepper::stop()
        EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
//
//        if (p.expect.pre_decel_stairs > 0) {
//            for (uint16_t s = 0; s < p.expect.pre_decel_stairs; s--) {
//                const uint32_t interval = Ramp::REAL_TYPE::interval(--current_stair);
//                EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
//                EXPECT_CALL(*Driver::mock, step()).Times(Ramp::STEPS_PER_STAIR).RetiresOnSaturation();
//            }
//        }

        if (p.steps > 0 && SIGN(p.initial_speed.rad()) != SIGN(p.run_speed.rad())) {
            EXPECT_CALL(*Driver::mock, dir(_)).RetiresOnSaturation();
        }
//
//        // TODO: handle direction change
//        if (p.expect.accel_stairs > 0) {
//            for (uint16_t s = 0; s < p.expect.accel_stairs; s++) {
//                const uint32_t interval = Ramp::REAL_TYPE::interval(++current_stair);
//                EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
//                EXPECT_CALL(*Driver::mock, step()).Times(Ramp::STEPS_PER_STAIR).RetiresOnSaturation();
//            }
//        }
//
//        if (p.expect.run_steps > 0) {
//            EXPECT_CALL(*Interrupt::mock, setInterval(p.expect.run_interval)).RetiresOnSaturation();
//            EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(p.expect.run_steps)).RetiresOnSaturation();
//        }
//
//        if (p.expect.decel_stairs > 0) {
//            for (uint16_t s = 0; s < p.expect.decel_stairs; s++) {
//                const uint32_t interval = Ramp::REAL_TYPE::interval(current_stair--);
//                EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
//                EXPECT_CALL(*Driver::mock, step()).Times(Ramp::STEPS_PER_STAIR).RetiresOnSaturation();
//            }
//        }

        EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    }

    auto sign = [](float x) {
        return (x < 0) ? -1 : 1;
    };
    const int32_t initial_pos = TestStepper::getPosition();
    const auto expected_pos = initial_pos + (sign(p.run_speed.rad()) * static_cast<int32_t>(p.steps));

    TestStepper::moveBy(p.run_speed, p.steps);
    constexpr uint32_t reverse_steps =
            2 * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);
    Interrupt::loopUntilStopped(p.steps + reverse_steps);

    ASSERT_EQ(expected_pos, TestStepper::getPosition());
}

//static std::vector<StepperMoveTestParams> moveByParams() {
//    std::vector<StepperMoveTestParams> params;
//
//    params.emplace_back(
//            StepperMoveTestParams(
//                    "atMaxSpeedCW_by0Steps_whileIdle",
//                    Angle::deg(0),
//                    SLEWING_SPEED,
//                    0,
//                    Movement(0, 0, 0, 0, 0)
//            )
//    );
//    return params;
//}

std::vector<NamedValue<Angle>> initialSpeeds = {
        NamedValue("Idle", Angle::deg(0)),
        NamedValue("TrackingCW", TRACKING_SPEED),
        NamedValue("TrackingCCW", -TRACKING_SPEED),
        NamedValue("SlewingCW", SLEWING_SPEED),
        NamedValue("SlewingCCW", -SLEWING_SPEED),
};

std::vector<NamedValue<Angle>> runSpeeds = {
        NamedValue("TrackingCW", TRACKING_SPEED),
        NamedValue("TrackingCCW", -TRACKING_SPEED),
        NamedValue("SlewingCW", SLEWING_SPEED),
        NamedValue("SlewingCCW", -SLEWING_SPEED),
};

std::vector<uint32_t> steps = {
        0,
        1,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) / 2,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1),
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) + 1,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) * 2,
};

INSTANTIATE_TEST_SUITE_P(
        StepperTest,
        StepperMoveByTest,
        testing::Combine(
                testing::ValuesIn(initialSpeeds),
                testing::ValuesIn(runSpeeds),
                testing::ValuesIn(steps)
        ),
//        testing::ValuesIn(moveByParams()),
//        testing::Values(
//                StepperMoveTestParams(
//                        "atMaxSpeedCW_by0Steps_whileIdle",
//                        Angle::deg(0),
//                        SLEWING_SPEED,
//                        0,
//                        Movement(0, 0, 0, 0, 0)
//                ),
//                StepperMoveTestParams(
//                        "atMaxSpeedCW_by1step_whileIdle",
//                        Angle::deg(0),
//                        SLEWING_SPEED,
//                        1,
//                        Movement(0, 0, 1, Ramp::REAL_TYPE::interval(1), 0)
//                ),
//                StepperMoveTestParams(
//                        "atMaxSpeedCW_by65steps_whileIdle",
//                        Angle::deg(0),
//                        SLEWING_SPEED,
//                        Ramp::REAL_TYPE::STEPS_PER_STAIR * 2 + 1,
//                        Movement(
//                                0,
//                                1,
//                                1,
//                                Ramp::REAL_TYPE::interval(1),
//                                1
//                        )
//                )
//        ),
        [=](const TestParamInfo<StepperMoveTestParams::GtestTuple> &i) {
//            const StepperMoveTestParams &p = StepperMoveTestParams(i.param);
            return "while" + std::get<0>(i.param).name + "_move"
                   + "_at" + std::get<1>(i.param).name
                   + "_by" + std::to_string(std::get<2>(i.param)) + "Steps";
        }
);

//
//#include <vector>
//#include <tuple>
//
//#include "Angle.h"
//#include "gmocks/MockedIntervalInterrupt.h"
//#include "gmocks/MockedDriver.h"
//#include "gmocks/MockedAccelerationRamp.h"
//
//template<typename T_Interrupt, typename T_Driver, typename T_Ramp>
//struct FixtureParams {
//    using Interrupt = T_Interrupt;
//    using Driver = T_Driver;
//    using Ramp = T_Ramp;
//
//    std::vector<Angle> speed_before;
//};
//
//constexpr auto SPR = 400UL * 256UL;
//constexpr auto TRANSMISSION = 35.46611505122143f;
//constexpr auto TRACKING_SPEED = Angle::deg(360.0f) / 86164.0905f;
//constexpr auto SLEWING_SPEED = TRANSMISSION * Angle::deg(2.0f);
//constexpr auto ACCELERATION = TRANSMISSION * Angle::deg(4.0f);
//
//using OatRAInterrupt = MockedIntervalInterrupt<0>;
//using OatRADriver = MockedDriver<SPR>;
//using OatRARamp = MockedAccelerationRamp<256, F_CPU, SPR, SLEWING_SPEED.mrad_u32(), ACCELERATION.mrad_u32()>;
//using OatRaFixture = FixtureParams<OatRAInterrupt, OatRADriver, OatRARamp>;
//
//using TestTypes = ::testing::Types<OatRaFixture>;
//
//static std::tuple<OatRaFixture> allParams{
//        {
//                .speed_before = {
//                        Angle::rad(0.0f),
//                        TRACKING_SPEED,
//                        SLEWING_SPEED
//                }
//        },
//};
//
//template<typename T>
//struct StopTest : public testing::Test {
//    StopTest() : params{std::get<T>(allParams)} {}
//
//    T params;
//};
//
//TYPED_TEST_SUITE_P(StopTest);
//
//TYPED_TEST_P(StopTest, stop) {
//    for (auto const &speed: this->params.speed_before) {
//
//    }
//}
//
//REGISTER_TYPED_TEST_SUITE_P(StopTest, stop);
//
//INSTANTIATE_TYPED_TEST_SUITE_P(TestPrefix, StopTest, TestTypes);
