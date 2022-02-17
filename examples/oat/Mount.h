#pragma once

#include "Configuration.h"
#include "Axis.h"

class Mount
{
public:
    using Ra = Axis<config::Ra>;
    using Dec = Axis<config::Dec>;

    void setup();

    void track(bool enable);

    template <typename AXIS>
    void slewTo(Angle target);

    template <typename AXIS>
    void startSlewing(bool direction);

    template <typename AXIS>
    void stopSlewing();

    template <typename AXIS>
    void guide(bool direction, unsigned long time_ms);

    template <typename AXIS>
    Angle position();

    template <typename AXIS>
    void position(Angle value);

private:
    unsigned long total_tracking_time = 0;
    unsigned long recent_tracking_start_time = 0;
};

template <typename AXIS>
void Mount::slewTo(Angle target)
{
    AXIS::slewTo(target);
}

template <typename AXIS>
void Mount::guide(bool direction, unsigned long time_ms)
{
    AXIS::guide(direction, time_ms);
}

template <typename AXIS>
void Mount::startSlewing(bool direction)
{
    AXIS::slew(direction);
}

template <typename AXIS>
void Mount::stopSlewing()
{
    AXIS::stopSlewing();
}

template <typename AXIS>
Angle Mount::position()
{
    return AXIS::position();
}

template <typename AXIS>
void Mount::position(Angle value)
{
    AXIS::position(value);
}