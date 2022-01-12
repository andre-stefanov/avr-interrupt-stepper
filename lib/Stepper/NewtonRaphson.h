#pragma once

class NewtonRaphson
{
private:
    NewtonRaphson(/* args */);

    static float constexpr sqrt(float x, float curr, float prev)
    {
        return curr == prev
                   ? curr
                   : sqrt(x, 0.5 * (curr + x / curr), curr);
    }

public:
    static float constexpr sqrt(float x)
    {
        return sqrt(x, x, 0);
    }
};