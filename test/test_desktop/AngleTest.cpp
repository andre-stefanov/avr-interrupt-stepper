#include "gtest/gtest.h"

#include "Angle.h"

namespace {

constexpr auto kSubtractedScalar = Angle::rad(3.5f) - 1.25f;
static_assert(kSubtractedScalar.rad() == 2.25f);

} // namespace

TEST(AngleTest, subtractsScalarFromAngle) {
    constexpr auto angle = Angle::rad(3.5f) - 1.25f;

    EXPECT_FLOAT_EQ(angle.rad(), 2.25f);
}

TEST(AngleTest, addsAndSubtractsAngles) {
    constexpr auto lhs = Angle::rad(2.5f);
    constexpr auto rhs = Angle::rad(1.0f);

    EXPECT_FLOAT_EQ((lhs + rhs).rad(), 3.5f);
    EXPECT_FLOAT_EQ((lhs - rhs).rad(), 1.5f);
}

TEST(AngleTest, scalesAngles) {
    constexpr auto angle = Angle::rad(2.0f);

    EXPECT_FLOAT_EQ((angle * 1.5f).rad(), 3.0f);
    EXPECT_FLOAT_EQ((2.0f * angle).rad(), 4.0f);
    EXPECT_FLOAT_EQ((angle / 2.0f).rad(), 1.0f);
    EXPECT_FLOAT_EQ((angle / Angle::rad(0.5f)), 4.0f);
}

TEST(AngleTest, convertsBetweenUnits) {
    EXPECT_NEAR(Angle::deg(180.0f).rad(), 3.14159265f, 1e-6f);
    EXPECT_NEAR(Angle::rad(3.14159265f).deg(), 180.0f, 1e-4f);
    EXPECT_EQ(Angle::mrad_u32(1250).mrad_u32(), 1250U);
}

TEST(AngleTest, negatesAngle) {
    constexpr auto angle = -Angle::rad(1.75f);

    EXPECT_FLOAT_EQ(angle.rad(), -1.75f);
}