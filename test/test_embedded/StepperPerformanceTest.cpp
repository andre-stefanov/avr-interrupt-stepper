#include <Arduino.h>
#include "unity.h"

#ifdef max
#undef max
#endif

#ifdef min
#undef min
#endif

#include "AccelerationRamp.h"
#include "Driver.h"
#include "IntervalInterrupt.h"
#include "Pin.h"
#include "Stepper.h"

#ifndef STEPPER_PERF_STEP_PIN
#define STEPPER_PERF_STEP_PIN 46
#endif

#ifndef STEPPER_PERF_DIR_PIN
#define STEPPER_PERF_DIR_PIN 47
#endif

#ifndef STEPPER_PERF_TARGET_SPS
#define STEPPER_PERF_TARGET_SPS 40352UL
#endif

#ifndef STEPPER_PERF_ACCELERATION
#define STEPPER_PERF_ACCELERATION STEPPER_PERF_TARGET_SPS
#endif

#ifndef STEPPER_PERF_RAMP_STAIRS
#define STEPPER_PERF_RAMP_STAIRS 256U
#endif

#ifndef STEPPER_PERF_MEASURE_WINDOW_US
#define STEPPER_PERF_MEASURE_WINDOW_US 250000UL
#endif

#ifndef STEPPER_PERF_START_TIMEOUT_MS
#define STEPPER_PERF_START_TIMEOUT_MS 5000UL
#endif

#ifndef STEPPER_PERF_STEADY_MARGIN_STEPS
#define STEPPER_PERF_STEADY_MARGIN_STEPS 1024UL
#endif

#ifndef STEPPER_PERF_MIN_STEADY_STEPS_PER_SEC
#define STEPPER_PERF_MIN_STEADY_STEPS_PER_SEC 0UL
#endif

namespace
{
using PerformanceInterrupt = IntervalInterrupt<Timer::TIMER_3>;
using PerformanceDriver = Driver<Pin<STEPPER_PERF_STEP_PIN>, Pin<STEPPER_PERF_DIR_PIN>>;
using PerformanceRamp = AccelerationRamp<
    STEPPER_PERF_RAMP_STAIRS,
    PerformanceInterrupt::FREQ,
    static_cast<uint32_t>(STEPPER_PERF_TARGET_SPS),
    static_cast<uint32_t>(STEPPER_PERF_ACCELERATION)>;
using PerformanceStepper = Stepper<PerformanceInterrupt, PerformanceDriver, PerformanceRamp>;

struct PerformanceMeasurement
{
    uint32_t elapsed_us;
    uint32_t steps;
    uint32_t achieved_steps_per_second;
};

static_assert(STEPPER_PERF_TARGET_SPS > 0, "STEPPER_PERF_TARGET_SPS must be greater than zero");
static_assert(STEPPER_PERF_ACCELERATION > 0, "STEPPER_PERF_ACCELERATION must be greater than zero");
static_assert(STEPPER_PERF_MEASURE_WINDOW_US > 0, "STEPPER_PERF_MEASURE_WINDOW_US must be greater than zero");

PerformanceMeasurement measureSteadyStateStepRate()
{
    PerformanceStepper::terminate(false);
    PerformanceStepper::reset();
    PerformanceStepper::init();

    const int32_t steady_state_start =
        static_cast<int32_t>(PerformanceRamp::STEPS_TOTAL) + static_cast<int32_t>(STEPPER_PERF_STEADY_MARGIN_STEPS);

    PerformanceStepper::move(static_cast<float>(STEPPER_PERF_TARGET_SPS));

    const uint32_t wait_started_ms = millis();
    while (PerformanceStepper::getPosition() < steady_state_start)
    {
        TEST_ASSERT_LESS_OR_EQUAL_UINT32_MESSAGE(
            STEPPER_PERF_START_TIMEOUT_MS,
            millis() - wait_started_ms,
            "Stepper did not reach steady-state speed before timeout");
    }

    const int32_t start_position = PerformanceStepper::getPosition();
    const uint32_t start_us = micros();
    while ((micros() - start_us) < STEPPER_PERF_MEASURE_WINDOW_US)
    {
    }
    const uint32_t elapsed_us = micros() - start_us;
    const int32_t end_position = PerformanceStepper::getPosition();

    PerformanceStepper::stop();
    while (PerformanceStepper::isRunning())
    {
    }

    const uint32_t steps = static_cast<uint32_t>(end_position - start_position);
    const uint32_t achieved_steps_per_second =
        static_cast<uint32_t>((static_cast<uint64_t>(steps) * 1000000ULL) / elapsed_us);

    return {elapsed_us, steps, achieved_steps_per_second};
}

void printMeasurement(const PerformanceMeasurement &measurement)
{
    Serial.print(F("[ PERF ] steady-state target: "));
    Serial.print(static_cast<uint32_t>(STEPPER_PERF_TARGET_SPS));
    Serial.print(F(" steps/s, achieved: "));
    Serial.print(measurement.achieved_steps_per_second);
    Serial.print(F(" steps/s, window: "));
    Serial.print(measurement.elapsed_us);
    Serial.print(F(" us, steps: "));
    Serial.println(measurement.steps);
}
} // namespace

STEPPER_USE_TIMER(3);

void test_stepper_reports_steady_state_step_rate_on_avr()
{
    const PerformanceMeasurement measurement = measureSteadyStateStepRate();

    printMeasurement(measurement);

    TEST_ASSERT_GREATER_THAN_UINT32(0U, measurement.elapsed_us);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, measurement.steps);
    TEST_ASSERT_GREATER_THAN_UINT32(0U, measurement.achieved_steps_per_second);

    if (static_cast<uint32_t>(STEPPER_PERF_MIN_STEADY_STEPS_PER_SEC) > 0U)
    {
        TEST_ASSERT_GREATER_OR_EQUAL_UINT32(
            static_cast<uint32_t>(STEPPER_PERF_MIN_STEADY_STEPS_PER_SEC),
            measurement.achieved_steps_per_second);
    }
}

void runStepperPerformanceTest()
{
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_stepper_reports_steady_state_step_rate_on_avr);
}