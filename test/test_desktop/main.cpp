#include "gtest/gtest.h"
#include "gmock/gmock.h"

int main(int argc, char **argv)
{
    ::testing::InitGoogleMock(&argc, argv);
    if (RUN_ALL_TESTS())
        ;
    return 0;
}
