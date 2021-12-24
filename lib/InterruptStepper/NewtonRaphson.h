#pragma once

class NewtonRaphson
{
private:
    NewtonRaphson(/* args */);

    static double constexpr sqrt(double x, double curr, double prev)
    {
        return curr == prev
                   ? curr
                   : sqrt(x, 0.5 * (curr + x / curr), curr);
    }

public:
    static double constexpr sqrt(double x)
    {
        return sqrt(x, x, 0);
    }
};