#include "gtest/gtest.h"
#include "gmock/gmock.h"
#include "gmocks/MockedPin.h"

#include "Driver.h"

using namespace ::testing;

using PIN_STEP = MockedPin<0>;
using PIN_DIR = MockedPin<1>;

class DriverTest : public Test
{
protected:
    void SetUp() override
    {
        PIN_STEP::mock = new StrictMock<PinMock>();
        PIN_DIR::mock = new StrictMock<PinMock>();
    }

    void TearDown() override
    {
        delete PIN_STEP::mock;
        delete PIN_DIR::mock;
    }
};

TEST_F(DriverTest, init)
{
    EXPECT_CALL(*PIN_STEP::mock, init()).Times(1);
    EXPECT_CALL(*PIN_DIR::mock, init()).Times(1);
    Driver<PIN_STEP, PIN_DIR>::init();
}

TEST_F(DriverTest, dir_cw)
{
    EXPECT_CALL(*PIN_DIR::mock, high()).Times(1);
    Driver<PIN_STEP, PIN_DIR, false>::dir(true);
}

TEST_F(DriverTest, dir_ccw)
{
    EXPECT_CALL(*PIN_DIR::mock, low()).Times(1);
    Driver<PIN_STEP, PIN_DIR, false>::dir(false);
}

TEST_F(DriverTest, dir_cw_inv)
{
    EXPECT_CALL(*PIN_DIR::mock, low()).Times(1);
    Driver<PIN_STEP, PIN_DIR, true>::dir(true);
}

TEST_F(DriverTest, dir_ccw_inv)
{
    EXPECT_CALL(*PIN_DIR::mock, high()).Times(1);
    Driver<PIN_STEP, PIN_DIR, true>::dir(false);
}

TEST_F(DriverTest, step)
{
    EXPECT_CALL(*PIN_STEP::mock, pulse()).Times(1);
    Driver<PIN_STEP, PIN_DIR, true>::step();
}