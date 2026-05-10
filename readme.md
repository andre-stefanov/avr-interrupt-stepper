# InterruptStepper

InterruptStepper is a header-only C++17 library for interrupt-driven stepper motor control in Arduino-style projects. It is organized around a few small building blocks that can be combined into a board-specific stepper configuration:

- `Pin` provides the pin abstraction used by the driver layer.
- `Driver` wraps a pulse/dir style stepper driver.
- `IntervalInterrupt` provides the timer backend.
- `AccelerationRamp` precomputes the interval table used for acceleration and deceleration.
- `Stepper` ties the timer, driver, and ramp together into the movement API.

The package metadata targets Arduino/AVR in PlatformIO. This repository also contains STM32 timer support and a native delegate backend used by the examples and desktop tests.

## What is special about this library

- Optimized for AVR targets. The default pin and timer backends include AVR-specific implementations, and the package metadata is set up for Arduino/AVR projects.
- No floating-point math in the active movement loop. Once a move is running, the ISR-driven stepping path works with integer intervals and counters rather than runtime float calculations.
- Direct timer interrupt usage. The AVR backend configures timer registers and compare/overflow interrupts directly instead of routing timing through a higher-level scheduler.
- Precalculated interval lookup table. `AccelerationRamp` builds the acceleration/deceleration intervals ahead of time so the runtime path can index a lookup table instead of recomputing ramp values on each step.

## Repository layout

- `include/` contains the public headers.
- `examples/avr/` contains an AVR example configuration.
- `examples/native/` contains a native host example.
- `test/test_desktop/` contains the GoogleTest-based native test suite.
- `test/test_embedded/` contains the Unity-based embedded tests.

## Add it to a PlatformIO project

The simplest way to use the library from another PlatformIO project is to add this repository to `lib_deps`:

```ini
[env:my_board]
platform = atmelavr
board = megaatmega2560
framework = arduino

lib_deps =
    https://github.com/andre-stefanov/avr-interrupt-stepper.git
```

Notes:

- The library is header-only, so there is no separate source module to link manually.
- `library.json` already declares the Embedded Template Library dependency and enables C++17.
- If your project overrides compiler flags, keep C++17 enabled.
- `F_CPU` must match your board configuration. For standard Arduino PlatformIO environments this is usually provided automatically.

For local development, you can also vendor this repository under your project's `lib/` directory instead of using `lib_deps`.

## Integration pattern

Typical usage follows this pattern:

1. Define the step and direction pins with `Pin`.
2. Select a timer backend with `IntervalInterrupt<Timer::...>`.
3. Build an `AccelerationRamp` with your timer frequency, maximum speed, and acceleration limits.
4. Combine those types into a `Stepper` alias.
5. Initialize the stepper in `setup()` and issue movement commands from your application code.

Minimal type composition looks like this:

```cpp
#include "Pin.h"
#include "Driver.h"
#include "IntervalInterrupt.h"
#include "AccelerationRamp.h"
#include "Stepper.h"

using step_pin = Pin<54>;
using dir_pin = Pin<55>;
using interrupt = IntervalInterrupt<Timer::TIMER_3>;
using driver = Driver<step_pin, dir_pin>;
using ramp = AccelerationRamp<128, interrupt::FREQ, 20000, 40000>;
using stepper = Stepper<interrupt, driver, ramp>;
```

In this repository, full board-specific setups are shown in `examples/avr/Configuration.h` and `examples/oat/Configuration.h`. A small host-side example is available in `examples/native/main.cpp`.

If you target AVR, your application also needs the timer ISR hookup for the timer you select. See `examples/avr/main.cpp` for the pattern used in this project.

## Running tests

### Native tests

The native test suite lives in `test/test_desktop/` and runs through the `example_native` PlatformIO environment.

```sh
pio test -e example_native
```

To list the PlatformIO test targets for that environment:

```sh
pio test -e example_native --list-tests
```

### Embedded tests

The embedded tests live in `test/test_embedded/` and use PlatformIO with Unity. The repository currently wires those tests to the `avr` environment in `platformio.ini`.

To build, upload, and run the embedded tests on connected hardware:

```sh
pio test -e avr
```

To compile the embedded test environment without uploading to a board:

```sh
pio test -e avr --without-uploading
```

### AVR performance tests

The embedded suite contains the performance regression test in `test/test_embedded/StepperPerformanceTest.cpp`.
It runs on the AVR target itself and measures the steady-state step rate using the real timer ISR path, the real AVR pin backend, and the real `Stepper` state machine.

The test prints the measured rate over serial during `pio test -e avr`. By default it only reports the achieved rate. To turn it into a regression gate on a dedicated board, add a minimum threshold through build flags:

```ini
build_flags =
    ${env.build_flags}
    -D STEPPER_PERF_MIN_STEADY_STEPS_PER_SEC=36000UL
```

You can also tune the test setup without editing the source:

- `STEPPER_PERF_TARGET_SPS` sets the requested speed to characterize.
- `STEPPER_PERF_ACCELERATION` sets the acceleration profile used to reach that speed.
- `STEPPER_PERF_STEP_PIN` and `STEPPER_PERF_DIR_PIN` select the pins toggled by the test driver.
- `STEPPER_PERF_MEASURE_WINDOW_US` controls the steady-state measurement window.

Current reference measurement on a 16 MHz ATmega2560:

- Requested rates up to `133000` steps/s were reached in steady state.
- Once the requested rate was increased further, the achieved rate plateaued at about `132700` steps/s.

Treat that number as an absolute maximum for the current single-axis test setup, not as a recommended application setting.
This test is intentionally close to a best-case scenario: it drives one stepper, uses the real timer ISR path and AVR pin backend, and spends almost all of the CPU budget on generating steps.
Real applications usually need extra headroom for other interrupts, main-loop work, communication, multi-axis coordination, safety handling, and any driver-specific pulse timing constraints.
In practice the usable application limit will therefore be lower, and production thresholds should be set comfortably below the measured ceiling.

Capture a baseline on the exact board and clock configuration you care about, then set the threshold slightly below that measured rate so future changes catch real regressions instead of normal run-to-run noise.

## AVR performance measurement

The library also exposes probe hooks for deeper on-target analysis, and they can be enabled from build flags without editing headers:

- `PROFILE_STEPPER=1` with `PROFILE_STEPPER_PIN=<pin>` toggles a probe around the `Stepper::move()` planning code.
- `DEBUG_INTERRUPT_TIMING_PIN=<pin>` toggles a probe around the compare-match ISR work, including the callback.
- `DEBUG_INTERRUPT_SET_INTERVAL_PIN=<pin>` toggles a probe around timer reprogramming in `setInterval()`.

Example `platformio.ini` additions for the `avr` environment:

```ini
build_flags =
    ${env.build_flags}
    -D PROFILE_STEPPER=1
    -D PROFILE_STEPPER_PIN=45
    -D DEBUG_INTERRUPT_TIMING_PIN=44
    -D DEBUG_INTERRUPT_SET_INTERVAL_PIN=43
```

Drive a long move at the target speed and measure the step pin plus any enabled probe pins with a logic analyzer or oscilloscope. The step pin frequency gives the true hardware step rate, while the probe pins show where the CPU budget is spent inside planning, ISR execution, and timer updates.

## Examples

Useful starting points in this repository:

- `examples/avr/` for AVR hardware integration.
- `examples/native/` for host-side experimentation.
- `examples/oat/` for a more complete multi-axis configuration.
