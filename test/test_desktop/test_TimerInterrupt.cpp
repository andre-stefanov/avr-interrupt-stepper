#include <TimerInterrupt.h>
#include <unity.h>

void test_function_calculator_addition(void) {
    TEST_ASSERT_EQUAL(32, 32);
}

int main(int argc, char **argv) {
    UNITY_BEGIN();
    RUN_TEST(test_function_calculator_addition);
    UNITY_END();

    return 0;
}