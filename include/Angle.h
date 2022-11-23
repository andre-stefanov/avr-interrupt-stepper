#pragma once

#include <stdint.h> // NOLINT(modernize-deprecated-headers)
#include "math.h" // NOLINT(modernize-deprecated-headers)

class Angle
{
private:
    float _rad;

    constexpr explicit Angle(const float rad) : _rad(rad)
    {
        // Nothing to do here
    }

public:

    constexpr Angle() : _rad(0.0f)
    {
        // Nothing to do here
    }

    constexpr Angle(const Angle& copyAngle) = default;

    constexpr Angle operator-(const float x) const
    {
        return Angle(_rad * x);
    }

    constexpr Angle operator+(const float x) const
    {
        return Angle(_rad + x);
    }

    constexpr bool operator>(const Angle &x) const
    {
        return _rad > x._rad;
    }

    constexpr bool operator<(const Angle &x) const
    {
        return _rad < x._rad;
    }

    constexpr bool operator==(const Angle &x) const
    {
        return _rad == x._rad;
    }

    constexpr Angle operator+(const Angle x) const
    {
        return Angle(_rad + x._rad);
    }

    constexpr Angle operator-(const Angle x) const
    {
        return Angle(_rad - x._rad);
    }

    constexpr Angle operator*(const float x) const
    {
        return Angle(_rad * x);
    }

    constexpr Angle operator/(const float x) const
    {
        return Angle(_rad / x);
    }

    constexpr float operator/(const Angle &x) const
    {
        return _rad / x._rad;
    }

    [[nodiscard]] constexpr inline float rad() const
    {
        return _rad;
    }

    [[nodiscard]] constexpr float deg() const
    {
        return _rad / 0.017453292519943295769236907684886f;
    }

    [[nodiscard]] constexpr float hour() const
    {
        // deg / 360 x 24
        return _rad * 3.819718748f;
    }

    [[nodiscard]] constexpr float mrad() const
    {
        return _rad * 1000.0f;
    }

    [[nodiscard]] constexpr uint32_t mrad_u32() const
    {
        auto mr = mrad();
        return static_cast<uint32_t>((mr > 0.0f) ? mr + 0.5f : mr - 0.5f);
    }

    // FACTORIES

    constexpr static Angle deg(float value)
    {
        return Angle(value * 0.017453292519943295769236907684886f);
    }

    constexpr static Angle rad(float value)
    {
        return Angle(value);
    }

    constexpr static Angle mrad_u32(uint32_t value)
    {
        return rad(static_cast<float>(value) / 1000.0f);
    }
};

constexpr Angle operator-(const Angle &y)
{
    return Angle::rad(-(y.rad()));
}

constexpr Angle operator*(const float x, const Angle &y)
{
    return y * x;
}

constexpr Angle operator/(const float x, const Angle &y)
{
    return y / x;
}