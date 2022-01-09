#pragma once

#include "utils.h"

constexpr float SIDEREAL = 86164.0905f;

namespace axis
{
    namespace ra
    {
        constexpr auto TRANSMISSION = 35.46611505122143f;
        constexpr auto SPR = 400U * 64U;

        constexpr auto TRACKING_SPEED = Speed(360.0f / SIDEREAL * TRANSMISSION);
        constexpr auto SLEWING_SPEED = Speed(2.0f * TRANSMISSION);
    }
}