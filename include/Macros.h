//
// Created by andre on 26.11.22.
//

#ifndef AVR_INTERRUPT_STEPPER_INCLUDE_MACROS_H
#define AVR_INTERRUPT_STEPPER_INCLUDE_MACROS_H

#include "Arduino.h"
#include "IntervalInterrupt_AVR.h"

#define STEPPER_USE_TIMER(x) \
  ISR(TIMER##x##_OVF_vect) { IntervalInterrupt_AVR<Timer::TIMER_##x##>::handle_overflow(); } \
  ISR(TIMER##x##_COMPA_vect) { IntervalInterrupt_AVR<Timer::TIMER_##x##>::handle_compare_match(); }

STEPPER_USE_TIMER(3);

#endif //AVR_INTERRUPT_STEPPER_INCLUDE_MACROS_H
