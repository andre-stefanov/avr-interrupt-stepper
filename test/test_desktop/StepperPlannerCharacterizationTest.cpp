#include <string>
#include <utility>

#include "StepperTestSupport.h"

using ::testing::InSequence;

// These tests intentionally pin the current planner shape.
// They should fail when internal stair orchestration changes, even if the public
// Stepper behavior remains the same.

namespace
{
struct RampStairs
{
  constexpr RampStairs(const uint16_t from, const uint16_t to) : from(from), to(to) {}

  explicit operator bool() const
  {
    return from > 0 && to > 0;
  }

  [[nodiscard]] uint16_t count() const
  {
    if (!static_cast<bool>(*this))
    {
      return 0;
    }

    return (from <= to)
        ? static_cast<uint16_t>((to - from) + 1U)
        : static_cast<uint16_t>((from - to) + 1U);
  }

  const uint16_t from;
  const uint16_t to;

  static const RampStairs NONE;
};

constexpr RampStairs RampStairs::NONE = {0, 0};

struct RampMovement
{
  constexpr RampMovement(
      const RampStairs &preDecelStairs,
      const int8_t accelDir,
      const RampStairs &accelStairs,
      const int8_t runDir,
      const uint32_t runSteps,
      const uint32_t runInterval,
      const RampStairs &decelStairs,
      const bool reuseRunIntervalForFirstDecelStair = false)
      : pre_decel_stairs(preDecelStairs),
        accel_dir(accelDir),
        accel_stairs(accelStairs),
        run_dir(runDir),
        run_steps(runSteps),
        run_interval(runInterval),
        decel_stairs(decelStairs),
        reuse_run_interval_for_first_decel_stair(reuseRunIntervalForFirstDecelStair) {}

  const RampStairs pre_decel_stairs;
  const int8_t accel_dir;
  const RampStairs accel_stairs;
  const int8_t run_dir;
  const uint32_t run_steps;
  const uint32_t run_interval;
  const RampStairs decel_stairs;
  const bool reuse_run_interval_for_first_decel_stair;

  [[nodiscard]] uint32_t totalSteps() const
  {
    return (static_cast<uint32_t>(pre_decel_stairs.count()) * static_cast<uint32_t>(Ramp::REAL_TYPE::STEPS_PER_STAIR))
        + (static_cast<uint32_t>(accel_stairs.count()) * static_cast<uint32_t>(Ramp::REAL_TYPE::STEPS_PER_STAIR))
        + run_steps
        + (static_cast<uint32_t>(decel_stairs.count()) * static_cast<uint32_t>(Ramp::REAL_TYPE::STEPS_PER_STAIR));
  }
};

struct PlannerSequenceParams
{
  PlannerSequenceParams(
      std::string name,
      const float initialSpeed,
      const float testSpeed,
      const int32_t testSteps,
      RampMovement expected)
      : name(std::move(name)),
        initial_speed(initialSpeed),
        test_speed(testSpeed),
        test_steps(testSteps),
        expected(std::move(expected)) {}

  [[nodiscard]] int32_t signedTargetOffset() const
  {
    return (test_speed < 0.0f) ? -test_steps : test_steps;
  }

  static auto values()
  {
    constexpr uint32_t stepsPerStair = Ramp::REAL_TYPE::STEPS_PER_STAIR;
    const uint16_t halfFastStair = Ramp::REAL_TYPE::maxAccelStairs(FAST_SPEED / 2.0f);
    const uint32_t halfFastStopSteps = static_cast<uint32_t>(halfFastStair) * stepsPerStair;
    const uint32_t reverseThreeStairs = stepsPerStair * 3U;
    const uint16_t reverseHalfFastAccelStairs = static_cast<uint16_t>((halfFastStopSteps + reverseThreeStairs) / (stepsPerStair * 2U));
    const uint32_t reverseHalfFastRunRestSteps =
        (halfFastStopSteps + reverseThreeStairs) - (static_cast<uint32_t>(reverseHalfFastAccelStairs) * stepsPerStair * 2U);

    return testing::Values(
        PlannerSequenceParams(
            "idle_to_full_fast_ramp_then_run_and_decel",
            0.0f,
            FAST_SPEED,
            Ramp::REAL_TYPE::STEPS_TOTAL * 3,
            {RampStairs::NONE,
             1,
             {1, 255},
             0,
             Ramp::REAL_TYPE::STEPS_TOTAL,
         Ramp::REAL_TYPE::getIntervalForSpeed(FAST_SPEED),
             {255, 1}}),
        PlannerSequenceParams(
        "idle_to_half_fast_short_run",
        0.0f,
        FAST_SPEED / 2,
            10000,
            {RampStairs::NONE,
             1,
             {1, 64},
             0,
             1808,
         Ramp::REAL_TYPE::getIntervalForSpeed(FAST_SPEED / 2),
             {64, 1}}),
        PlannerSequenceParams(
        "idle_to_short_partial_fast_ramp",
        0.0f,
        FAST_SPEED,
            227,
            {RampStairs::NONE,
             1,
             {1, 1},
             0,
             99,
             Ramp::REAL_TYPE::interval(1),
         {1, 1}}),
      // Reverse a single slow step from a steady half-fast run: fully pre-decelerate,
      // switch direction, then finish with a slow reverse run and no acceleration.
      PlannerSequenceParams(
        "half_fast_forward_to_slow_reverse_single_step",
        FAST_SPEED / 2,
        -SLOW_SPEED,
        1,
        {RampStairs{halfFastStair, 1},
         0,
         RampStairs::NONE,
         -1,
         halfFastStopSteps + 1U,
         Ramp::REAL_TYPE::getIntervalForSpeed(-SLOW_SPEED),
         RampStairs::NONE}),
      // Reverse three stairs from a steady half-fast run: fully pre-decelerate,
      // accelerate in reverse up to the highest possible stair, run one rest block,
      // then decelerate back to idle.
      PlannerSequenceParams(
        "half_fast_forward_to_half_fast_reverse_three_stairs",
        FAST_SPEED / 2,
        -(FAST_SPEED / 2),
        static_cast<int32_t>(reverseThreeStairs),
        {RampStairs{halfFastStair, 1},
         -1,
         RampStairs{1, reverseHalfFastAccelStairs},
         0,
         reverseHalfFastRunRestSteps,
         Ramp::REAL_TYPE::interval(reverseHalfFastAccelStairs),
         RampStairs{reverseHalfFastAccelStairs, 1}}),
      // Hold the same half-fast speed and request exactly the stop distance.
      // This should skip the run phase entirely and switch straight into deceleration.
      PlannerSequenceParams(
        "half_fast_forward_exact_stop_distance_decelerates_immediately",
        FAST_SPEED / 2,
        FAST_SPEED / 2,
        static_cast<int32_t>(halfFastStopSteps),
        {RampStairs::NONE,
         0,
         RampStairs::NONE,
         0,
         0,
         Ramp::REAL_TYPE::getIntervalForSpeed(FAST_SPEED / 2),
         RampStairs{halfFastStair, 1},
         true}));
  }

  std::string name;
    float initial_speed;
  float test_speed;
  int32_t test_steps;
  RampMovement expected;
};
} // namespace

struct PlannerSequenceCharacterizationFixture : public StepperPlannerCharacterizationTestBase,
                                                public testing::WithParamInterface<PlannerSequenceParams>
{
};

TEST_P(PlannerSequenceCharacterizationFixture, MoveByFollowsExpectedPlannerShape)
{
  const auto p = GetParam();
  if (p.initial_speed != 0.0f)
  {
    prepareRunningAtSpeed(p.initial_speed);
  }

  const int32_t initialPosition = TestStepper::getPosition();
  ASSERT_EQ(initialPosition, Driver::position);

  {
    InSequence sequence;

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();

    if (p.expected.pre_decel_stairs)
    {
      for (uint16_t stair = p.expected.pre_decel_stairs.from; stair >= p.expected.pre_decel_stairs.to; stair--)
      {
        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair))).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, step())
            .Times(static_cast<int>(Ramp::REAL_TYPE::STEPS_PER_STAIR))
            .RetiresOnSaturation();
      }
    }

    if (p.expected.accel_dir == 1)
    {
      EXPECT_CALL(*Driver::mock, dir(true)).RetiresOnSaturation();
    }
    else if (p.expected.accel_dir == -1)
    {
      EXPECT_CALL(*Driver::mock, dir(false)).RetiresOnSaturation();
    }

    if (p.expected.accel_stairs)
    {
      for (uint16_t stair = p.expected.accel_stairs.from; stair <= p.expected.accel_stairs.to; stair++)
      {
        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair))).RetiresOnSaturation();
        EXPECT_CALL(*Driver::mock, step())
            .Times(static_cast<int>(Ramp::REAL_TYPE::STEPS_PER_STAIR))
            .RetiresOnSaturation();
      }
    }

    if (p.expected.run_dir == 1)
    {
      EXPECT_CALL(*Driver::mock, dir(true)).RetiresOnSaturation();
    }
    else if (p.expected.run_dir == -1)
    {
      EXPECT_CALL(*Driver::mock, dir(false)).RetiresOnSaturation();
    }

    EXPECT_CALL(*Interrupt::mock, setInterval(p.expected.run_interval)).RetiresOnSaturation();
    if (p.expected.run_steps > 0)
    {
      EXPECT_CALL(*Driver::mock, step()).Times(static_cast<int>(p.expected.run_steps)).RetiresOnSaturation();
    }

    if (p.expected.decel_stairs)
    {
      for (uint16_t stair = p.expected.decel_stairs.from; stair >= p.expected.decel_stairs.to; stair--)
      {
        if (!(p.expected.reuse_run_interval_for_first_decel_stair && stair == p.expected.decel_stairs.from))
        {
          EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair))).RetiresOnSaturation();
        }
        EXPECT_CALL(*Driver::mock, step())
            .Times(static_cast<int>(Ramp::REAL_TYPE::STEPS_PER_STAIR))
            .RetiresOnSaturation();
      }
    }

    EXPECT_CALL(*Interrupt::mock, stop()).RetiresOnSaturation();
  }

  TestStepper::moveBy(p.test_speed, p.test_steps);
  runInterruptSteps(p.expected.totalSteps());

  expectIdleState(initialPosition + p.signedTargetOffset());
}

INSTANTIATE_TEST_SUITE_P(
    StepperPlannerCharacterization,
    PlannerSequenceCharacterizationFixture,
    PlannerSequenceParams::values(),
    [](const testing::TestParamInfo<PlannerSequenceParams> &i)
    {
      return i.param.name;
    });