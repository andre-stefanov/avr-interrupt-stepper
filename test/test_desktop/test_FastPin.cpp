#include <unity.h>

#include "FastPin.h"
#include "gtest/gtest.h"

using ::testing::Mock;

typedef FastPin<0> TestPin;

TEST(FastPinTests, high)
{
    EXPECT_CALL(TestPin::mock, high())
        .Times(3);
    // TEST_ASSERT_EQUAL(false, FastPin<0>::value);
    TestPin::high();
    EXPECT_CALL(TestPin::mock, high())
        .Times(1);
}

// void test_pin_high(void)
// {
//     EXPECT_CALL(TestPin::mock, high())
//         .Times(3);
//     // TEST_ASSERT_EQUAL(false, FastPin<0>::value);
//     FastPin<0>::high();
//     EXPECT_CALL(TestPin::mock, high())
//         .Times(1);
//     // TEST_ASSERT_EQUAL(true, FastPin<0>::value);

//     Mock::VerifyAndClearExpectations(&TestPin::mock);
// }

// void test_pin_pulse(void)
// {
//     // TEST_ASSERT_EQUAL(0LL, FastPin<0>::pulseCnt);
//     FastPin<0>::pulse();
//     // TEST_ASSERT_EQUAL(1LL, FastPin<0>::pulseCnt);
// }

int main(int argc, char **argv)
{
    // UNITY_BEGIN();
    // RUN_TEST(test_pin_high);
    // RUN_TEST(test_pin_pulse);
    // UNITY_END();

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();

    return 0;
}