#include "unity.h"
#include "mocks/PinMock.h"
#include "Driver.h"

using PIN_STEP = Pin<0>;
using PIN_DIR = Pin<0>;

using TestDriver = Driver<400, PIN_STEP, PIN_DIR>;

void test_init()
{
    TestDriver::init();
}

void runDriverTest()
{
    UnitySetTestFile(__FILE__);
    RUN_TEST(test_init);
}