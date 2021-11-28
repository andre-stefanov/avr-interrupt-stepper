#pragma once

#if defined(UNIT_TEST)
#include "FastPin_Test.h"
#elif defined(ARDUINO_ARCH_AVR)
#include "FastPin_AVR.h"
#else
#include "FastPin_Arduino.h"
#endif