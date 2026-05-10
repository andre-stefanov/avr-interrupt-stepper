#pragma once

#include <cstdint>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "gmocks/MockedIntervalInterrupt.h"
#include "gmocks/MockedDriver.h"
#include "gmocks/MockedAccelerationRamp.h"
#include "Stepper.h"

constexpr uint16_t TEST_MOTOR_STEPS_PER_REVOLUTION = 400;
constexpr uint16_t TEST_MICROSTEPS = 256;
constexpr uint16_t TEST_RAMP_STAIRS = 256;
/// Slow baseline speed in steps per second for the shared desktop test profile.
constexpr float SLOW_SPEED = 42.14899919642364f;
/// Fast speed in steps per second for the shared desktop test profile.
constexpr float FAST_SPEED = 40352.55756938972f;
/// Ramp acceleration in steps per second squared for the shared desktop test profile.
constexpr float FAST_ACCELERATION = 40352.55756938972f;

/// Mocked interrupt backend used by the desktop Stepper tests.
using Interrupt = MockedIntervalInterrupt<0>;
/// Mocked stepper driver using the shared desktop-test motor resolution.
using Driver = MockedDriver<TEST_MOTOR_STEPS_PER_REVOLUTION * TEST_MICROSTEPS>;
/// Mocked acceleration ramp configured to match the shared desktop test limits.
using Ramp = MockedAccelerationRamp<
    TEST_RAMP_STAIRS,
    F_CPU,
    static_cast<uint32_t>(FAST_SPEED),
    static_cast<uint32_t>(FAST_ACCELERATION)>;

/// Stepper instance under test for the native desktop suite.
using TestStepper = Stepper<Interrupt, Driver, Ramp>;

/**
 * @brief Base fixture that owns and resets the shared Stepper test doubles.
 *
 * All Stepper desktop tests use static mocks, so this fixture centralizes mock
 * allocation, default expectations, and Stepper reset behavior.
 */
struct StepperTestHarness : public testing::Test
{
protected:
  /**
   * @brief Create strict mocks and install default expectations for each test.
   */
  void SetUp() override
  {
    Interrupt::mock = new ::testing::StrictMock<IntervalInterruptMock>();
    Driver::mock = new ::testing::StrictMock<DriverMock>();
    Ramp::mock = new ::testing::StrictMock<RampMock>();
    Driver::position = 0;
    Driver::inverted = false;
    Driver::direction = false;

    Ramp::delegateToReal();
    Ramp::expectLimits();

    EXPECT_CALL(*Interrupt::mock, setCallback(::testing::_)).Times(::testing::AnyNumber());
  }

  /**
   * @brief Reset the Stepper singleton and destroy the test doubles.
   */
  void TearDown() override
  {
    TestStepper::reset();

    delete Interrupt::mock;
    delete Driver::mock;
    delete Ramp::mock;
  }
};

/**
 * @brief Shared behavior-test helpers for arranging and asserting Stepper state.
 *
 * These helpers intentionally focus on observable behavior so individual tests
 * can stay short and describe only the scenario-specific expectations.
 */
struct StepperBehaviorTestBase : public StepperTestHarness
{
protected:
  /**
   * @brief Drive the Stepper into a steady running state at the requested speed.
   *
   * The helper performs the acceleration sequence needed to reach the target
   * speed, then leaves the Stepper running so the caller can test interruption,
   * reversal, or query behavior from that starting point.
   *
   * @param speed Target speed in steps per second. A value of `0.0f` leaves the
   * Stepper idle.
   */
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
        EXPECT_CALL(*Interrupt::mock, setInterval(Ramp::REAL_TYPE::interval(stair))).RetiresOnSaturation();
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

  /**
   * @brief Execute a bounded number of simulated interrupt callbacks.
   *
   * @param steps Number of interrupt-driven steps to process.
   * @param expectStopped Whether the Stepper is expected to be stopped after the
   * processed steps.
   */
  static void runInterruptSteps(const uint32_t steps, const bool expectStopped = true)
  {
    Interrupt::loopUntilStopped(steps, expectStopped);
  }

  /**
   * @brief Assert that driver and Stepper position queries report the same value.
   *
   * @param expectedPosition Expected absolute position in steps.
   */
  static void expectPosition(const int32_t expectedPosition)
  {
    EXPECT_EQ(expectedPosition, Driver::position);
    EXPECT_EQ(expectedPosition, TestStepper::getPosition());
  }

  /**
   * @brief Assert the observable Stepper state in one place.
   *
   * @param expectedPosition Expected absolute position in steps.
   * @param expectedDistance Expected remaining distance in steps.
   * @param expectedRunning Expected running flag state.
   */
  static void expectStepperState(
      const int32_t expectedPosition,
      const uint32_t expectedDistance,
      const bool expectedRunning)
  {
    expectPosition(expectedPosition);
    EXPECT_EQ(expectedDistance, TestStepper::distanceToGo());
    EXPECT_EQ(expectedRunning, TestStepper::isRunning());
  }

  /**
   * @brief Assert that the Stepper is fully idle at the requested position.
   *
   * @param expectedPosition Expected absolute position in steps.
   */
  static void expectIdleState(const int32_t expectedPosition)
  {
    expectStepperState(expectedPosition, 0U, false);
  }
};

/**
 * @brief Base fixture for tests that intentionally lock down planner internals.
 *
 * This exists mainly for naming clarity so characterization tests stand out from
 * behavior-driven tests in the suite structure.
 */
struct StepperPlannerCharacterizationTestBase : public StepperBehaviorTestBase
{
};

/**
 * @brief Parameterized behavior-test fixture for Stepper scenarios.
 *
 * @tparam ParamType Parameter type supplied through GoogleTest.
 */
template <typename ParamType>
struct StepperBehaviorTest : public StepperBehaviorTestBase, public testing::WithParamInterface<ParamType>
{
};