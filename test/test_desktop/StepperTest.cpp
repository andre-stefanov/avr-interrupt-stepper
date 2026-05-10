#include <cstdlib>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "StepperTestSupport.h"

using ::testing::_;
using ::testing::AnyNumber;
using ::testing::InSequence;

namespace
{
int on_complete_calls = 0;

void onComplete()
{
  ++on_complete_calls;
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
      const float initialSpeed,
      const uint16_t expectDecelStairs)
      : name(std::move(name)),
        initial_speed(initialSpeed),
        expect_decel_stairs(expectDecelStairs) {}

  [[nodiscard]] uint32_t expectSteps() const
  {
    return static_cast<uint32_t>(expect_decel_stairs) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);
  }

  [[nodiscard]] int32_t direction() const
  {
    return (initial_speed < 0.0f) ? -1 : 1;
  }

  std::string name;
  float initial_speed;
  uint16_t expect_decel_stairs;
};

struct TimedMoveTestParams
{
  TimedMoveTestParams(
      std::string name,
      const float speed,
      const uint32_t timeMs,
      const int32_t expectedSteps)
      : name(std::move(name)),
        speed(speed),
        time_ms(timeMs),
        expected_steps(expectedSteps) {}

  std::string name;
  float speed;
  uint32_t time_ms;
  int32_t expected_steps;
};

struct TimedMoveCurrentStateParams
{
  using GTestTuple = std::tuple<NamedValue<float>, TimedMoveTestParams>;

  explicit TimedMoveCurrentStateParams(const GTestTuple &tuple)
      : initial_speed(std::get<0>(tuple).value),
        timed_move(std::get<1>(tuple)) {}

  static auto values()
  {
    const std::vector<NamedValue<float>> initialSpeeds = {
        NamedValue("SlowCW", SLOW_SPEED),
        NamedValue("SlowCCW", -SLOW_SPEED),
        NamedValue("FastCW", FAST_SPEED),
        NamedValue("FastCCW", -FAST_SPEED),
    };

    return testing::Combine(
        testing::ValuesIn(initialSpeeds),
        testing::Values(
            TimedMoveTestParams("forward_for_one_second", 5.0f, 1000U, 5),
            TimedMoveTestParams("backward_for_one_second", -5.0f, 1000U, -5),
            TimedMoveTestParams("forward_truncates_fractional_step", 2.5f, 1000U, 2),
            TimedMoveTestParams("backward_truncates_fractional_step", -2.5f, 1000U, -2),
            TimedMoveTestParams("below_one_step", 2.5f, 399U, 0)));
  }

  float initial_speed;
  TimedMoveTestParams timed_move;
};

struct RelativeMovementMatrixParams
{
  using GTestTuple = std::tuple<NamedValue<float>, NamedValue<float>, int32_t>;

  explicit RelativeMovementMatrixParams(const GTestTuple &tuple)
      : initial_speed(std::get<0>(tuple).value),
        run_speed(std::get<1>(tuple).value),
        steps(std::get<2>(tuple)) {}

  static auto values()
  {
    const std::vector<NamedValue<float>> initialSpeeds = {
        NamedValue("Idle", 0.0f),
      NamedValue("SlowCW", SLOW_SPEED),
      NamedValue("SlowCCW", -SLOW_SPEED),
      NamedValue("FastCW", FAST_SPEED),
      NamedValue("FastCCW", -FAST_SPEED),
    };

    const std::vector<NamedValue<float>> runSpeeds = {
      NamedValue("SlowCW", SLOW_SPEED),
      NamedValue("SlowCCW", -SLOW_SPEED),
      NamedValue("FastCW", FAST_SPEED),
      NamedValue("FastCCW", -FAST_SPEED),
    };

    const std::vector<int32_t> steps = {
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

  float initial_speed;
  float run_speed;
  int32_t steps;
};
} // namespace

// These tests verify public Stepper behavior only.
// Exact interval and stair sequencing lives in StepperPlannerCharacterizationTest.cpp.

using StepperStopBehaviorTest = StepperBehaviorTest<StepperStopTestParams>;
using StepperInversionBehaviorTest = StepperBehaviorTest<bool>;
using TimedMoveBehaviorTest = StepperBehaviorTest<TimedMoveTestParams>;
using TimedMoveCurrentStateTest = StepperBehaviorTest<TimedMoveCurrentStateParams::GTestTuple>;
using RelativeMovementMatrixTest = StepperBehaviorTest<RelativeMovementMatrixParams::GTestTuple>;

struct StepperStateTest : public StepperBehaviorTestBase
{
};

TEST_P(StepperStopBehaviorTest, StopCompletesAtExpectedPosition)
{
  const auto p = GetParam();
  prepareRunningAtSpeed(p.initial_speed);

  const int32_t initialPosition = TestStepper::getPosition();
  ASSERT_EQ(initialPosition, Driver::position);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(0);
  EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(p.expectSteps()));

  TestStepper::stop();
  runInterruptSteps(p.expectSteps());

  expectIdleState(initialPosition + (p.direction() * static_cast<int32_t>(p.expectSteps())));
}

TEST_P(StepperInversionBehaviorTest, MoveByKeepsLogicalDirectionAndFinalPosition)
{
  const bool inverted = GetParam();
  constexpr int32_t steps = 7;

  TestStepper::setInverted(inverted);

  EXPECT_TRUE(Driver::inverted == inverted);

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(steps).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveBy(SLOW_SPEED, steps);
  runInterruptSteps(steps);

  expectIdleState(steps);
}

TEST_P(StepperInversionBehaviorTest, MoveToKeepsLogicalDirectionAndReachesTarget)
{
  const bool inverted = GetParam();
  constexpr int32_t initialPosition = 10;
  constexpr int32_t targetPosition = 5;
  constexpr int32_t stepsToTarget = initialPosition - targetPosition;

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);
  TestStepper::setInverted(inverted);

  EXPECT_TRUE(Driver::inverted == inverted);

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(false)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(stepsToTarget).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveTo(SLOW_SPEED, targetPosition);
  runInterruptSteps(stepsToTarget);

  expectIdleState(targetPosition);
}

TEST_P(StepperInversionBehaviorTest, MoveTimeKeepsLogicalDirectionAndDistance)
{
  const bool inverted = GetParam();
  constexpr float speed = 5.0f;
  constexpr uint32_t timeMs = 1000U;
  constexpr int32_t expectedSteps = 5;

  TestStepper::setInverted(inverted);

  EXPECT_TRUE(Driver::inverted == inverted);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(speed))).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(expectedSteps);

  TestStepper::moveTime(speed, timeMs);
  runInterruptSteps(expectedSteps);

  expectIdleState(expectedSteps);
}

INSTANTIATE_TEST_SUITE_P(
    StepperBehavior,
    StepperInversionBehaviorTest,
    testing::Values(false, true),
    [](const TestParamInfo<bool> &i)
    {
      return i.param ? "driverInverted" : "driverNormal";
    });

INSTANTIATE_TEST_SUITE_P(
    StepperBehavior,
    StepperStopBehaviorTest,
    testing::Values(
        StepperStopTestParams("idle", 0.0f, 0),
      StepperStopTestParams("slow_cw", SLOW_SPEED, 0),
      StepperStopTestParams("slow_ccw", -SLOW_SPEED, 0),
      StepperStopTestParams("fast_cw", FAST_SPEED, Ramp::STAIRS_COUNT - 1),
      StepperStopTestParams("fast_ccw", -FAST_SPEED, Ramp::STAIRS_COUNT - 1)),
    [](const TestParamInfo<StepperStopTestParams> &i)
    {
      return i.param.name;
    });

TEST_F(StepperStateTest, SlowMoveReportsProgressViaQueries)
{
  constexpr int32_t totalSteps = 5;

  expectIdleState(0);

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(totalSteps).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveBy(SLOW_SPEED, totalSteps);
  expectStepperState(0, static_cast<uint32_t>(totalSteps), true);

  runInterruptSteps(2, false);
  expectStepperState(2, static_cast<uint32_t>(totalSteps - 2), true);

  runInterruptSteps(totalSteps - 2);
  expectIdleState(totalSteps);
}

TEST_F(StepperStateTest, ContinuousMoveReportsProgressUntilStopped)
{
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED))).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(3);

  TestStepper::move(SLOW_SPEED);

  const uint32_t initialRemaining = TestStepper::distanceToGo();
  EXPECT_TRUE(TestStepper::isRunning());
  EXPECT_GT(initialRemaining, static_cast<uint32_t>(INT32_MAX / 2));

  runInterruptSteps(3, false);

  expectPosition(3);
  EXPECT_TRUE(TestStepper::isRunning());
  EXPECT_LT(TestStepper::distanceToGo(), initialRemaining);

  TestStepper::stop();

  expectIdleState(3);
}

TEST_F(StepperStateTest, MultistepMoveReportsProgressViaQueries)
{
  constexpr int32_t totalSteps = 227;

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveBy(FAST_SPEED, totalSteps);
  expectStepperState(0, static_cast<uint32_t>(totalSteps), true);

  runInterruptSteps(1, false);
  expectStepperState(1, static_cast<uint32_t>(totalSteps - 1), true);

  runInterruptSteps(Ramp::STEPS_PER_STAIR - 1, false);
  expectStepperState(
      static_cast<int32_t>(Ramp::STEPS_PER_STAIR),
      static_cast<uint32_t>(totalSteps - Ramp::STEPS_PER_STAIR),
      true);

  runInterruptSteps(totalSteps - Ramp::STEPS_PER_STAIR);
  expectIdleState(totalSteps);
}

TEST_F(StepperStateTest, MoveByZeroStepsTerminatesImmediately)
{
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(0);
  EXPECT_CALL(*Driver::mock, dir(_)).Times(0);
  EXPECT_CALL(*Driver::mock, step()).Times(0);

  TestStepper::moveBy(SLOW_SPEED, 0);

  expectIdleState(0);
}

TEST_F(StepperStateTest, StopInvokesCompletionCallbackOnceAfterDeceleration)
{
  on_complete_calls = 0;
  prepareRunningAtSpeed(FAST_SPEED);

  const int32_t initialPosition = TestStepper::getPosition();
  const uint32_t stopSteps = static_cast<uint32_t>(Ramp::STAIRS_COUNT - 1) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(stopSteps));

  TestStepper::stop(StepperCallback::create<onComplete>());
  runInterruptSteps(stopSteps);

  EXPECT_EQ(1, on_complete_calls);
  expectIdleState(initialPosition + static_cast<int32_t>(stopSteps));
}

TEST_F(StepperStateTest, StopInvokesCompletionCallbackImmediatelyWhenIdle)
{
  on_complete_calls = 0;

  EXPECT_CALL(*Interrupt::mock, stop()).Times(2);
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(0);
  EXPECT_CALL(*Driver::mock, dir(_)).Times(0);
  EXPECT_CALL(*Driver::mock, step()).Times(0);

  TestStepper::stop(StepperCallback::create<onComplete>());

  EXPECT_EQ(1, on_complete_calls);
  expectIdleState(0);
}

TEST_F(StepperStateTest, MoveByInvokesCompletionCallbackOnceOnNaturalCompletion)
{
  on_complete_calls = 0;
  constexpr int32_t totalSteps = 4;

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(totalSteps).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveBy(SLOW_SPEED, totalSteps, StepperCallback::create<onComplete>());
  EXPECT_EQ(0, on_complete_calls);

  runInterruptSteps(totalSteps);

  EXPECT_EQ(1, on_complete_calls);
  expectIdleState(totalSteps);
}

TEST_F(StepperStateTest, MoveToInvokesCompletionCallbackOnceOnNaturalCompletion)
{
  on_complete_calls = 0;
  constexpr int32_t initialPosition = 100;
  constexpr int32_t targetPosition = 103;
  constexpr int32_t stepsToTarget = targetPosition - initialPosition;

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(stepsToTarget).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveTo(SLOW_SPEED, targetPosition, StepperCallback::create<onComplete>());
  EXPECT_EQ(0, on_complete_calls);

  runInterruptSteps(stepsToTarget);

  EXPECT_EQ(1, on_complete_calls);
  expectIdleState(targetPosition);
}

TEST_F(StepperStateTest, MoveTimeInvokesCompletionCallbackOnceOnNaturalCompletion)
{
  on_complete_calls = 0;
  constexpr float speed = 5.0f;
  constexpr uint32_t timeMs = 1000U;
  constexpr int32_t expectedSteps = 5;

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(speed))).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(expectedSteps);

  TestStepper::moveTime(speed, timeMs, StepperCallback::create<onComplete>());
  EXPECT_EQ(0, on_complete_calls);

  runInterruptSteps(expectedSteps);

  EXPECT_EQ(1, on_complete_calls);
  expectIdleState(expectedSteps);
}

TEST_F(StepperStateTest, MoveToUsesAbsoluteTargetFromCurrentPosition)
{
  constexpr int32_t initialPosition = 100;
  constexpr int32_t targetPosition = 105;
  constexpr int32_t stepsToTarget = targetPosition - initialPosition;

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, dir(true)).Times(1).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(SLOW_SPEED)))
        .RetiresOnSaturation();
    EXPECT_CALL(*Driver::mock, step()).Times(stepsToTarget).RetiresOnSaturation();
    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveTo(SLOW_SPEED, targetPosition);
  expectStepperState(initialPosition, static_cast<uint32_t>(stepsToTarget), true);

  runInterruptSteps(stepsToTarget);
  expectIdleState(targetPosition);
}

TEST_F(StepperStateTest, MoveToAlreadyAtTargetTerminatesImmediately)
{
  constexpr int32_t initialPosition = 100;

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(0);
  EXPECT_CALL(*Driver::mock, dir(_)).Times(0);
  EXPECT_CALL(*Driver::mock, step()).Times(0);

  TestStepper::moveTo(SLOW_SPEED, initialPosition);

  expectIdleState(initialPosition);
}

TEST_F(StepperStateTest, MoveToReachesAbsoluteTargetAfterReversing)
{
  prepareRunningAtSpeed(FAST_SPEED);

  const int32_t currentPosition = TestStepper::getPosition();
  const int32_t targetPosition = currentPosition - static_cast<int32_t>(Ramp::STEPS_PER_STAIR * 3U);
  const uint32_t stepBudget = static_cast<uint32_t>(currentPosition - targetPosition)
      + (2U * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR))
      + 10U;

  ASSERT_GT(currentPosition, targetPosition);
  ASSERT_EQ(currentPosition, Driver::position);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  TestStepper::moveTo(FAST_SPEED, targetPosition);
  EXPECT_TRUE(TestStepper::isRunning());
  EXPECT_GT(TestStepper::distanceToGo(), 0U);

  runInterruptSteps(stepBudget);
  expectIdleState(targetPosition);
}

TEST_F(StepperStateTest, MoveByReachesRequestedOffsetAfterReversingThreeStairs)
{
  prepareRunningAtSpeed(FAST_SPEED);

  const int32_t currentPosition = TestStepper::getPosition();
  const int32_t reverseSteps = static_cast<int32_t>(Ramp::STEPS_PER_STAIR * 3U);
  const int32_t targetPosition = currentPosition - reverseSteps;
  const uint32_t stepBudget = static_cast<uint32_t>(reverseSteps)
      + (2U * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR))
      + 10U;

  ASSERT_EQ(currentPosition, Driver::position);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  TestStepper::moveBy(-FAST_SPEED, reverseSteps);
  EXPECT_TRUE(TestStepper::isRunning());
  EXPECT_GT(TestStepper::distanceToGo(), 0U);

  runInterruptSteps(stepBudget);
  expectIdleState(targetPosition);
}

// This characterizes the guarded negative-overflow path without pinning an exact
// remaining-distance value, because the underlying INT32_MIN handling still needs
// a production-side contract cleanup.
TEST_F(StepperStateTest, MoveToClampsOverflowingNegativeDistance)
{
  constexpr int32_t initialPosition = 42;

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(false)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveTo(-SLOW_SPEED, INT32_MIN);

  const uint32_t initialRemaining = TestStepper::distanceToGo();
  EXPECT_TRUE(TestStepper::isRunning());
  EXPECT_GT(initialRemaining, static_cast<uint32_t>(INT32_MAX / 2));

  runInterruptSteps(1, false);
  expectStepperState(initialPosition - 1, TestStepper::distanceToGo(), true);
  EXPECT_LT(TestStepper::distanceToGo(), initialRemaining);

  TestStepper::stop();
  runInterruptSteps(0);
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, MoveToClampsOverflowingNegativeDistanceToInt32Max)
{
  constexpr int32_t initialPosition = 42;
  constexpr uint32_t clampedDistance = static_cast<uint32_t>(INT32_MAX);

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(false)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveTo(-SLOW_SPEED, INT32_MIN);
  expectStepperState(initialPosition, clampedDistance, true);

  runInterruptSteps(1, false);
  expectStepperState(initialPosition - 1, clampedDistance - 1U, true);

  TestStepper::stop();
  runInterruptSteps(0);
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_F(StepperStateTest, MoveToClampsOverflowingPositiveDistance)
{
  constexpr int32_t initialPosition = -42;
  constexpr uint32_t clampedDistance = static_cast<uint32_t>(INT32_MAX);

  Driver::position = initialPosition;
  TestStepper::setPosition(initialPosition);

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(true)).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());

  TestStepper::moveTo(SLOW_SPEED, INT32_MAX);
  expectStepperState(initialPosition, clampedDistance, true);

  runInterruptSteps(1, false);
  expectStepperState(initialPosition + 1, clampedDistance - 1U, true);

  TestStepper::stop();
  EXPECT_FALSE(TestStepper::isRunning());
}

TEST_P(TimedMoveBehaviorTest, MoveTimeHonorsSignedSpeedAndDuration)
{
  const auto p = GetParam();
  const auto expectedSteps = static_cast<int>(std::abs(p.expected_steps));

  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(p.speed >= 0.0f)).Times(1);
  EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::getIntervalForSpeed(p.speed))).Times(1);
  EXPECT_CALL(*Driver::mock, step()).Times(expectedSteps);

  TestStepper::moveTime(p.speed, p.time_ms);
  expectStepperState(0, static_cast<uint32_t>(expectedSteps), true);

  runInterruptSteps(static_cast<uint32_t>(expectedSteps));
  expectIdleState(p.expected_steps);
}

INSTANTIATE_TEST_SUITE_P(
    StepperBehavior,
    TimedMoveBehaviorTest,
    testing::Values(
        TimedMoveTestParams("forward_for_one_second", 5.0f, 1000U, 5),
        TimedMoveTestParams("backward_for_one_second", -5.0f, 1000U, -5),
        TimedMoveTestParams("forward_for_two_hundred_ms", 5.0f, 200U, 1),
        TimedMoveTestParams("backward_for_two_hundred_ms", -5.0f, 200U, -1),
        TimedMoveTestParams("forward_truncates_fractional_step", 2.5f, 1000U, 2),
        TimedMoveTestParams("backward_truncates_fractional_step", -2.5f, 1000U, -2)),
    [](const TestParamInfo<TimedMoveTestParams> &i)
    {
      return i.param.name;
    });

TEST_F(StepperStateTest, MoveTimeBelowOneStepTerminatesImmediately)
{
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(0);
  EXPECT_CALL(*Driver::mock, dir(_)).Times(0);
  EXPECT_CALL(*Driver::mock, step()).Times(0);

  TestStepper::moveTime(2.5f, 399U);

  expectIdleState(0);
}

TEST_F(StepperStateTest, MoveTimeWithZeroDurationTerminatesImmediately)
{
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(0);
  EXPECT_CALL(*Driver::mock, dir(_)).Times(0);
  EXPECT_CALL(*Driver::mock, step()).Times(0);

  TestStepper::moveTime(5.0f, 0U);

  expectIdleState(0);
}

TEST_P(TimedMoveCurrentStateTest, MoveTimeReachesExpectedFinalPositionAcrossCurrentStates)
{
  const TimedMoveCurrentStateParams p = TimedMoveCurrentStateParams(GetParam());

  prepareRunningAtSpeed(p.initial_speed);

  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  const int32_t initialPosition = TestStepper::getPosition();
  constexpr uint32_t reverseStepBudget =
      2U * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);

  TestStepper::moveTime(p.timed_move.speed, p.timed_move.time_ms);

  if (p.timed_move.expected_steps != 0)
  {
    EXPECT_TRUE(TestStepper::isRunning());
    EXPECT_GT(TestStepper::distanceToGo(), 0U);
  }

  runInterruptSteps(static_cast<uint32_t>(std::abs(p.timed_move.expected_steps)) + reverseStepBudget);

  expectIdleState(initialPosition + p.timed_move.expected_steps);
}

INSTANTIATE_TEST_SUITE_P(
    StepperBehavior,
    TimedMoveCurrentStateTest,
    TimedMoveCurrentStateParams::values(),
    [](const TestParamInfo<TimedMoveCurrentStateParams::GTestTuple> &i)
    {
      return "while" + std::get<0>(i.param).name
          + "_moveTime_" + std::get<1>(i.param).name;
    });

TEST_P(RelativeMovementMatrixTest, MoveByReachesExpectedFinalPositionAcrossCurrentStates)
{
  const RelativeMovementMatrixParams p = RelativeMovementMatrixParams(GetParam());

  prepareRunningAtSpeed(p.initial_speed);

  EXPECT_CALL(*Interrupt::mock, setInterval(_)).Times(AnyNumber());
  EXPECT_CALL(*Interrupt::mock, stop()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, step()).Times(AnyNumber());
  EXPECT_CALL(*Driver::mock, dir(_)).Times(AnyNumber());

  const int32_t initialPosition = TestStepper::getPosition();
  const int32_t expectedPosition = initialPosition + ((p.run_speed < 0.0f ? -1 : 1) * static_cast<int32_t>(p.steps));
  constexpr uint32_t reverseStepBudget =
      2U * static_cast<uint32_t>(Ramp::STAIRS_COUNT) * static_cast<uint32_t>(Ramp::STEPS_PER_STAIR);

  TestStepper::moveBy(p.run_speed, p.steps);
  runInterruptSteps(static_cast<uint32_t>(p.steps) + reverseStepBudget);

  expectIdleState(expectedPosition);
}

INSTANTIATE_TEST_SUITE_P(
    StepperBehavior,
    RelativeMovementMatrixTest,
    RelativeMovementMatrixParams::values(),
    [](const TestParamInfo<RelativeMovementMatrixParams::GTestTuple> &i)
    {
      return "while" + std::get<0>(i.param).name
          + "_move_at" + std::get<1>(i.param).name
          + "_by" + std::to_string(std::get<2>(i.param)) + "Steps";
    });