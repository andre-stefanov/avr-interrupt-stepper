#pragma once

constexpr inline float deg_to_rad(float value)
{
  return value * 0.017453292519943295769236907684886f;
}

constexpr inline uint32_t deg_to_rad_u32(float value)
{
  if (value < 0)
  {
    value *= -1;
  }
  return (uint32_t)(value * 0.017453292519943295769236907684886f);
}

struct Speed
{
  const float deg_per_sec;

  constexpr Speed(float deg_per_sec) : deg_per_sec(deg_per_sec)
  {
    // Nothing to do here
  }

  constexpr float rad() const
  {
    return deg_to_rad(deg_per_sec);
  }

  constexpr float mrad() const
  {
    return rad() * 1000;
  }

  constexpr uint32_t mrad_u32() const
  {
    return (uint32_t)mrad();
  }
};