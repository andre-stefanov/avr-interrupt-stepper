#include <utility>

#include "gtest/gtest.h"
#include "gmock/gmock.h"

#include <cmath>

#include "oat/Configuration.h"

#include "gmocks/MockedIntervalInterrupt.h"
#include "gmocks/MockedDriver.h"
#include "gmocks/MockedAccelerationRamp.h"
#include "Stepper.h"

#define RA_DEG_TO_STEPS(deg) ((deg / 360.f) * RA_STEPPER_SPR * RA_MICROSTEPPING * RA_TRANSMISSION)

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

namespace
{
  int on_complete_calls = 0;

  void onComplete()
  {
    ++on_complete_calls;
  }
}

template <typename T>
struct NamedValue
{
  NamedValue(std::string name, T value) : name(std::move(name)), value(value) {}

  std::string name;
  T value;
};

struct StepperStopTestParams
{
  StepperStopTestParams(
      std::string name,
      float initial_speed,
      uint16_t expect_decel_stairs)
      : name(std::move(name)),
        initial_speed(initial_speed),
        expect_decel_stairs(expect_decel_stairs) {}

  std::string name;
  float initial_speed;
  uint16_t expect_decel_stairs;

  [[nodiscard]] uint32_t expect_steps() const
  {
    const uint32_t decel_stairs = expect_decel_stairs;
    const uint32_t steps_per_stair = Ramp::STEPS_PER_STAIR;
    return decel_stairs * steps_per_stair;
  }
};

struct StepperTestHarness : public testing::Test
{
  void SetUp() override
  {
    Interrupt::mock = new StrictMock<IntervalInterruptMock>();
    Driver::mock = new StrictMock<DriverMock>();
    Ramp::mock = new StrictMock<RampMock>();
    Driver::position = 0;
    Driver::direction = false;
    on_complete_calls = 0;

    Ramp::delegateToReal();
    Ramp::expectLimits();

    EXPECT_CALL(*Interrupt::mock, setCallback(_)).Times(AnyNumber());
  }

  void TearDown() override
  {
    TestStepper::reset();

    delete Interrupt::mock;
    delete Driver::mock;
    delete Ramp::mock;
  }
};

struct StepperBehaviorTestBase : public StepperTestHarness
{
  static void prepareRunningAtSpeed(const float speed)
  {
    if (speed == 0.0f)
    {
      return;
    }

    const uint16_t max_stair = Ramp::maxAccelStairs(speed);
    const auto steps = static_cast<int>(max_stair) * static_cast<int>(Ramp::STEPS_PER_STAIR) + 1;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(speed >= 0.0f)).Times(1);

    if (max_stair > 0)
    {
      for (uint32_t stair = 1; stair <= max_stair; stair++)
      {
        const uint32_t interval = Ramp::REAL_TYPE::interval(stair);
        EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
      }
    }

    EXPECT_CALL(
        *Interrupt::mock,
        setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(speed)))
        .RetiresOnSaturation();

    EXPECT_CALL(*Driver::mock, step()).Times(steps).RetiresOnSaturation();

    TestStepper::move(speed);
    Interrupt::loopUntilStopped(steps, false);
  }
};

struct StepperPlannerCharacterizationTestBase : public StepperTestHarness
{
};

template <typename T>
struct StepperBehaviorTest : public StepperBehaviorTestBase, public testing::WithParamInterface<T>
{
};

struct StepperStateTest : public StepperBehaviorTestBase
{
};

using StepperStopTest = StepperBehaviorTest<StepperStopTestParams>;

TEST_P(StepperStopTest, stop)
{
  const auto p = GetParam();
  prepareRunningAtSpeed(p.initial_speed);
  const int32_t initial_driver_pos = Driver::position;
  const int32_t initial_stepper_pos = TestStepper::getPosition();

  ASSERT_EQ(initial_driver_pos, initial_stepper_pos);

  {
    InSequence s;

    // interrupt will be stopped immediately after/on calling Stepper::stop()
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();

    if (p.expect_decel_stairs > 0)
    {
      for (uint16_t stair = GetParam().expect_decel_stairs; stair > 0; stair--)
      {
        const uint32_t interval = Ramp::REAL_TYPE::interval(stair);
        EXPECT_CALL(*Interrupt::mock, setInterval(interval)).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, step()).Times(Ramp::STEPS_PER_STAIR).RetiresOnSaturation();
      }
    }

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::stop();
  Interrupt::loopUntilStopped(GetParam().expect_steps());

  const int32_t dir = (p.initial_speed < 0.0f) ? -1 : 1;
  const auto expected_final_pos = initial_driver_pos + (dir * static_cast<int32_t>(p.expect_steps()));

  EXPECT_EQ(expected_final_pos, Driver::position);
  EXPECT_EQ(expected_final_pos, TestStepper::getPosition());
  EXPECT_EQ(Driver::position, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

INSTANTIATE_TEST_SUITE_P(
    StepperTest,
    StepperStopTest,
    testing::Values(
        StepperStopTestParams(
            "idle",
            0.0f,
            0),
        StepperStopTestParams(
            "tracking_cw",
            TRACKING_SPEED,
            0),
        StepperStopTestParams(
            "tracking_ccw",
            -TRACKING_SPEED,
            0),
        StepperStopTestParams(
            "slewing_cw",
            SLEWING_SPEED,
            Ramp::STAIRS_COUNT - 1),
        StepperStopTestParams(
            "slewing_ccw",
            -SLEWING_SPEED,
            Ramp::STAIRS_COUNT - 1)),
    [=](const TestParamInfo<StepperStopTestParams> &i)
    {
      return i.param.name;
    });

TEST_F(StepperStateTest, slowMoveStateQueriesTrackProgress)
{
  constexpr int32_t total_steps = 5;

  EXPECT_EQ(0, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());

  {
    InSequence s;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(TRACKING_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(total_steps).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveBy(TRACKING_SPEED, total_steps);

  EXPECT_EQ(0, TestStepper::getPosition());
  EXPECT_EQ(static_cast<uint32_t>(total_steps), TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(2, false);

  EXPECT_EQ(2, TestStepper::getPosition());
  EXPECT_EQ(static_cast<uint32_t>(total_steps - 2), TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(total_steps - 2);

  EXPECT_EQ(total_steps, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, multistepMoveStateQueriesTrackProgress)
{
  constexpr int32_t total_steps = 227;

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveBy(SLEWING_SPEED, total_steps);

  EXPECT_EQ(0, TestStepper::getPosition());
  EXPECT_EQ(static_cast<uint32_t>(total_steps), TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(1, false);

  EXPECT_EQ(1, TestStepper::getPosition());
  EXPECT_EQ(static_cast<uint32_t>(total_steps - 1), TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(Ramp::STEPS_PER_STAIR - 1, false);

  EXPECT_EQ(static_cast<int32_t>(Ramp::STEPS_PER_STAIR), TestStepper::getPosition());
  EXPECT_EQ(static_cast<uint32_t>(total_steps - Ramp::STEPS_PER_STAIR), TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(total_steps - Ramp::STEPS_PER_STAIR);

  EXPECT_EQ(total_steps, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, stopInvokesCompletionCallbackOnceAfterDeceleration)
{
  prepareRunningAtSpeed(SLEWING_SPEED);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::stop(StepperCallback::create<onComplete>());
  Interrupt::loopUntilStopped(static_cast<uint32_t>(Ramp::STAIRS_COUNT - 1) * Ramp::STEPS_PER_STAIR);

  EXPECT_EQ(1, on_complete_calls);
  EXPECT_FALSE(TestStepper::isRunning());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
}

TEST_F(StepperStateTest, moveToUsesAbsoluteTargetFromCurrentPosition)
{
  constexpr int32_t initial_pos = 100;
  constexpr int32_t target_pos = 105;
  constexpr int32_t steps_to_target = target_pos - initial_pos;

  Driver::position = initial_pos;
  TestStepper::setPosition(initial_pos);

  {
    InSequence s;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(TRACKING_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(steps_to_target).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveTo(TRACKING_SPEED, target_pos);

  EXPECT_EQ(static_cast<uint32_t>(steps_to_target), TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(steps_to_target);

  EXPECT_EQ(target_pos, Driver::position);
  EXPECT_EQ(target_pos, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, moveToReachesAbsoluteTargetAfterReversing)
{
  prepareRunningAtSpeed(SLEWING_SPEED);

  const int32_t current_pos = TestStepper::getPosition();
  const int32_t target_pos = current_pos - static_cast<int32_t>(Ramp::STEPS_PER_STAIR * 3U);
  const uint32_t stop_steps = static_cast<uint32_t>(Ramp::STAIRS_COUNT - 1) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);
  const uint32_t expected_remaining_steps = (stop_steps * 2U) + static_cast<uint32_t>(current_pos - target_pos);

  ASSERT_GT(current_pos, target_pos);
  ASSERT_EQ(current_pos, Driver::position);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  TestStepper::moveTo(SLEWING_SPEED, target_pos);

  EXPECT_EQ(expected_remaining_steps, TestStepper::distanceToGo());

  const uint32_t limit = static_cast<uint32_t>(current_pos - target_pos) + 2U * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR) + 10U;

  Interrupt::loopUntilStopped(limit);

  EXPECT_EQ(target_pos, Driver::position);
  EXPECT_EQ(target_pos, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, moveByReachesRequestedOffsetAfterReversingThreeStairs)
{
  prepareRunningAtSpeed(SLEWING_SPEED);

  const int32_t current_pos = TestStepper::getPosition();
  const int32_t reverse_steps = static_cast<int32_t>(Ramp::STEPS_PER_STAIR * 3U);
  const int32_t target_pos = current_pos - reverse_steps;
  const uint32_t stop_steps = static_cast<uint32_t>(Ramp::STAIRS_COUNT - 1) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);
  const uint32_t expected_remaining_steps = (stop_steps * 2U) + static_cast<uint32_t>(reverse_steps);

  ASSERT_EQ(current_pos, Driver::position);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  TestStepper::moveBy(-SLEWING_SPEED, reverse_steps);

  EXPECT_EQ(expected_remaining_steps, TestStepper::distanceToGo());

  const uint32_t limit = static_cast<uint32_t>(reverse_steps) + 2U * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR) + 10U;

  Interrupt::loopUntilStopped(limit);

  EXPECT_EQ(target_pos, Driver::position);
  EXPECT_EQ(target_pos, TestStepper::getPosition());
  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, moveToClampsOverflowingNegativeDistance)
{
  constexpr int32_t initial_pos = 42;
  constexpr uint32_t clamped_distance = static_cast<uint32_t>(INT32_MAX) + 1U;

  Driver::position = initial_pos;
  TestStepper::setPosition(initial_pos);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(false)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveTo(-TRACKING_SPEED, INT32_MIN);

  EXPECT_EQ(clamped_distance, TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(1, false);

  EXPECT_EQ(initial_pos - 1, Driver::position);
  EXPECT_EQ(initial_pos - 1, TestStepper::getPosition());
  EXPECT_EQ(clamped_distance - 1U, TestStepper::distanceToGo());

  TestStepper::stop();

  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, moveToClampsOverflowingPositiveDistance)
{
  constexpr int32_t initial_pos = -42;
  constexpr uint32_t clamped_distance = static_cast<uint32_t>(INT32_MAX);

  Driver::position = initial_pos;
  TestStepper::setPosition(initial_pos);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveTo(TRACKING_SPEED, INT32_MAX);

  EXPECT_EQ(clamped_distance, TestStepper::distanceToGo());
  EXPECT_TRUE(TestStepper::isRunning());

  Interrupt::loopUntilStopped(1, false);

  EXPECT_EQ(initial_pos + 1, Driver::position);
  EXPECT_EQ(initial_pos + 1, TestStepper::getPosition());
  EXPECT_EQ(clamped_distance - 1U, TestStepper::distanceToGo());

  TestStepper::stop();

  EXPECT_EQ(0U, TestStepper::distanceToGo());
  EXPECT_FALSE(TestStepper::isRunning());
}

struct StepperCartesianTestParams
{

  using GtestTuple = std::tuple<NamedValue<float>, NamedValue<float>, int32_t>;

  explicit StepperCartesianTestParams(const GtestTuple &tuple)
      : initial_speed(std::get<0>(tuple).value),
        run_speed(std::get<1>(tuple).value),
        steps(std::get<2>(tuple)) {}

  float initial_speed;
  float run_speed;
  int32_t steps;

  static auto cartesianProduct()
  {
    std::vector<NamedValue<float>> initialSpeeds = {
        NamedValue("Idle", 0.0f),
        NamedValue("TrackingCW", TRACKING_SPEED),
        NamedValue("TrackingCCW", -TRACKING_SPEED),
        NamedValue("SlewingCW", SLEWING_SPEED),
        NamedValue("SlewingCCW", -SLEWING_SPEED),
    };

    std::vector<NamedValue<float>> runSpeeds = {
        NamedValue("TrackingCW", TRACKING_SPEED),
        NamedValue("TrackingCCW", -TRACKING_SPEED),
        NamedValue("SlewingCW", SLEWING_SPEED),
        NamedValue("SlewingCCW", -SLEWING_SPEED),
    };

    std::vector<int32_t> steps = {
        0,
        1,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) / 2,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1),
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) + 1,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) * 2,
        Ramp::STEPS_PER_STAIR * (Ramp::STAIRS_COUNT - 1) * 3,
    };

    return testing::Combine(
        testing::ValuesIn(initialSpeeds),
        testing::ValuesIn(runSpeeds),
        testing::ValuesIn(steps));
  }
};

using CartesianStepperTest = StepperBehaviorTest<StepperCartesianTestParams::GtestTuple>;

TEST_P(CartesianStepperTest, moveBy)
{
  const StepperCartesianTestParams &p = StepperCartesianTestParams(GetParam());

  prepareRunningAtSpeed(p.initial_speed);

  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  {
    InSequence seq;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  auto sign = [](float x)
  {
    return (x < 0) ? -1 : 1;
  };
  const int32_t initial_pos = TestStepper::getPosition();
  const auto expected_pos = initial_pos + (sign(p.run_speed) * static_cast<int32_t>(p.steps));

  TestStepper::moveBy(p.run_speed, p.steps);
  constexpr uint32_t reverse_steps =
      2 * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);
  Interrupt::loopUntilStopped(p.steps + reverse_steps);

  const auto final_pos = TestStepper::getPosition();

  ASSERT_EQ(expected_pos, final_pos);
}

INSTANTIATE_TEST_SUITE_P(
    StepperTest,
    CartesianStepperTest,
    StepperCartesianTestParams::cartesianProduct(),
    [=](const TestParamInfo<StepperCartesianTestParams::GtestTuple> &i)
    {
      return "while" + std::get<0>(i.param).name + "_move" + "_at" + std::get<1>(i.param).name + "_by" + std::to_string(std::get<2>(i.param)) + "Steps";
    });

struct RampStairs
{
  constexpr RampStairs(uint16_t from, uint16_t to) : from(from), to(to) {}

  explicit operator bool() const { return from > 0 && to > 0; }

  [[nodiscard]] uint16_t total() const
  {
    return to - from;
  }

  const uint16_t from;
  const uint16_t to;

  static const RampStairs NONE;
};

constexpr RampStairs RampStairs::NONE = {0, 0};

struct RampMovement
{

  [[maybe_unused]] constexpr RampMovement(
      const RampStairs &preDecelStairs,
      const int8_t accelDir,
      const RampStairs &accelStairs,
      const int8_t runDir,
      const uint32_t runSteps,
      const uint32_t runInterval,
      const RampStairs &decelStairs)
      : preDecelStairs(preDecelStairs),
        accelDir(accelDir),
        accelStairs(accelStairs),
        runDir(runDir),
        runSteps(runSteps),
        runInterval(runInterval),
        decelStairs(decelStairs) {}

  const RampStairs preDecelStairs;
  const int8_t accelDir;
  const RampStairs accelStairs;
  const int8_t runDir;
  const uint32_t runSteps;
  const uint32_t runInterval;
  const RampStairs decelStairs;
};

struct RampStepperTestParams
{

  std::string name;
  float initial_speed;
  float testSpeed;
  int32_t testSteps;
  RampMovement expected;

  RampStepperTestParams(
      std::string name,
      const float &initialSpeed,
      const float &testSpeed,
      int32_t testSteps,
      RampMovement expected)
      : name(std::move(name)),
        initial_speed(initialSpeed),
        testSpeed(testSpeed),
        testSteps(testSteps),
        expected(std::move(expected)) {}

  static auto values()
  {
    return testing::Values<RampStepperTestParams>(
        RampStepperTestParams(
            "whileIdle_atSlewingCW_byFullRamp",
            0.0f,
            SLEWING_SPEED,
            Ramp::REAL_TYPE::STEPS_TOTAL * 3,
            {RampStairs::NONE,
             1,
             {1, 255},
             0,
             Ramp::REAL_TYPE::STEPS_TOTAL,
             Ramp::REAL_TYPE::getIntervalForSpeed(SLEWING_SPEED),
             {255, 1}}),
        RampStepperTestParams(
            "whileIdle_atHalfSlewingCW_by10000Steps",
            0.0f,
            SLEWING_SPEED / 2,
            10000,
            {RampStairs::NONE,
             1,
             {1, 64},
             0,
             1808,
             Ramp::REAL_TYPE::getIntervalForSpeed(SLEWING_SPEED / 2),
             {64, 1}}),
        RampStepperTestParams(
            "whileIdle_atSlewingCW_by227Steps",
            0.0f,
            SLEWING_SPEED,
            227,
            {RampStairs::NONE,
             1,
             {1, 1},
             0,
             99,
             Ramp::REAL_TYPE::interval(1),
             {1, 1}}));
  }
};

struct PlannerSequenceCharacterizationFixture : public StepperPlannerCharacterizationTestBase,
                                                public testing::WithParamInterface<RampStepperTestParams>
{
};

TEST_P(PlannerSequenceCharacterizationFixture, moveBy)
{
  const auto p = GetParam();

  {
    InSequence s;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();

    // pre deceleration
    if (p.expected.preDecelStairs)
    {
      for (uint16_t stair = p.expected.preDecelStairs.from; stair >= p.expected.preDecelStairs.to; stair--)
      {
        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair)))
            .RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, step())
            .Times(static_cast<int>(Ramp::REAL_TYPE::STEPS_PER_STAIR))
            .RetiresOnSaturation();
      }
    }

    if (p.expected.accelDir == 1)
    {
      EXPECT_CALL(*Driver::mock, dir(true)).RetiresOnSaturation();
    }

    if (p.expected.accelDir == -1)
    {
      EXPECT_CALL(*Driver::mock, dir(false)).RetiresOnSaturation();
    }

    // acceleration
    if (p.expected.accelStairs)
    {
      for (uint16_t stair = p.expected.accelStairs.from; stair <= p.expected.accelStairs.to; stair++)
      {
        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair))).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(Ramp::REAL_TYPE::STEPS_PER_STAIR)).RetiresOnSaturation();
      }
    }

    if (p.expected.runDir == 1)
    {
      EXPECT_CALL(*Driver::mock, dir(true)).RetiresOnSaturation();
    }

    if (p.expected.runDir == -1)
    {
      EXPECT_CALL(*Driver::mock, dir(false)).RetiresOnSaturation();
    }

    // run
    EXPECT_CALL(*Interrupt::mock, setInterval(p.expected.runInterval)).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(p.expected.runSteps)).RetiresOnSaturation();

    // deceleration
    if (p.expected.decelStairs)
    {
      for (uint16_t stair = p.expected.decelStairs.from; stair >= p.expected.decelStairs.to; stair--)
      {
        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair))).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(Ramp::REAL_TYPE::STEPS_PER_STAIR)).RetiresOnSaturation();
      }
    }

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveBy(p.testSpeed, p.testSteps);
  Interrupt::loopUntilStopped(p.testSteps + Ramp::REAL_TYPE::STEPS_TOTAL);
}

INSTANTIATE_TEST_SUITE_P(
    StepperPlannerCharacterization,
    PlannerSequenceCharacterizationFixture,
    RampStepperTestParams::values(),
    [=](const TestParamInfo<RampStepperTestParams> &i)
    {
      return i.param.name;
    });