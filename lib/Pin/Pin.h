#pragma once

#if defined(ARDUINO_ARCH_AVR)
#include "Pin_AVR.h"
#elif defined(ARDUINO)
#include "Pin_Arduino.h"
#endif