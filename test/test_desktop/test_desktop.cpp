#include "gtest/gtest.h"

// void test_AccelerationRamp();
// void test_Stepper();

int main(int argc, char *argv[])
{
    // test_AccelerationRamp();
    // test_Stepper();
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
