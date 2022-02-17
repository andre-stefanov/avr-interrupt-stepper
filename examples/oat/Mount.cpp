#include "Mount.h"
#include "etl/delegate.h"

#ifdef ARDUINO_ARCH_AVR
ISR(TIMER3_OVF_vect) { IntervalInterrupt_AVR<Timer::TIMER_3>::handle_overflow(); }
ISR(TIMER3_COMPA_vect) { IntervalInterrupt_AVR<Timer::TIMER_3>::handle_compare_match(); }
ISR(TIMER4_OVF_vect) { IntervalInterrupt_AVR<Timer::TIMER_4>::handle_overflow(); }
ISR(TIMER4_COMPA_vect) { IntervalInterrupt_AVR<Timer::TIMER_4>::handle_compare_match(); }
#endif

void Mount::setup()
{
    Ra::setup();
    Dec::setup();
}

void Mount::track(bool enabled)
{
    if (enabled)
    {
        if (recent_tracking_start_time == 0UL)
        {
            recent_tracking_start_time = millis();
        }
    }
    else
    {
        if (recent_tracking_start_time > 0UL)
        {
            total_tracking_time += millis() - recent_tracking_start_time;
            recent_tracking_start_time = 0UL;
        }
    }
    Ra::track(enabled);
}

template <>
Angle Mount::position<Mount::Ra>()
{
    auto tracking_time = total_tracking_time + ((recent_tracking_start_time) ? millis() - recent_tracking_start_time : 0);
    return Ra::position() - (Ra::TRACKING_SPEED * tracking_time);
}

template <>
void Mount::position<Mount::Ra>(Angle value)
{
    Mount::Ra::position(value);
    total_tracking_time = 0;
    recent_tracking_start_time = millis();
}