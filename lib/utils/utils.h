#pragma once

#include "etl/type_traits.h"

template <typename T>
constexpr inline bool is_pow2(const T value)
{
  return (value & (value - 1)) == 0;
}

constexpr inline float deg_to_rad(const float value)
{
  return value * 0.017453292519943295769236907684886f;
}

constexpr inline float rad_to_deg(const float value)
{
  return value / 0.017453292519943295769236907684886f;
}

constexpr inline uint32_t deg_to_rad_u32(float value)
{
  if (value < 0)
  {
    value *= -1;
  }
  return (uint32_t)(value * 0.017453292519943295769236907684886f);
}

class Angle
{
private:
  const float _rad;

  constexpr Angle(float rad) : _rad(rad)
  {
    // Nothing to do here
  }

public:
  constexpr Angle operator*(const Angle &y) const
  {
    return y * _rad;
  }

  constexpr Angle operator*(const float x) const
  {
    return Angle(_rad * x);
  }

  constexpr Angle operator/(const float x) const
  {
    return Angle(_rad / x);
  }

  constexpr float rad() const
  {
    return _rad;
  }

  constexpr float mrad() const
  {
    return _rad * 1000.0f;
  }

  constexpr uint32_t mrad_u32() const
  {
    return (uint32_t)mrad();
  }

  // FACTORIES

  constexpr static Angle from_deg(float value)
  {
    return Angle(deg_to_rad(value));
  }

  constexpr static Angle from_rad(float value)
  {
    return Angle(value);
  }

  constexpr static Angle from_mrad_u32(uint32_t value)
  {
    return from_rad(value / 1000.0f);
  }
};

constexpr Angle operator*(const float x, const Angle &y)
{
  return y * x;
}

constexpr Angle operator/(const float x, const Angle &y)
{
  return y / x;
}

constexpr double constexpr_sqrt(double x)
{
  if (0 <= x)
  {
    double curr = x;
    double prev = 0;
    while (curr != prev)
    {
      prev = curr;
      curr = 0.5 * (curr + x / curr);
    }
    return curr;
  }
  else
  {
    static_assert(true);
    return -1;
  }
}