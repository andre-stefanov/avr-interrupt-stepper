#include <unity.h>

#include <stdio.h>

void test_AccelerationRamp();
void test_Stepper();

int main(int argc, char const *argv[])
{
    test_AccelerationRamp();
    test_Stepper();
    return 0;
}
