#pragma once

template <typename T>
constexpr inline bool is_pow2(const T value)
{
  return (value & (value - 1)) == 0;
}