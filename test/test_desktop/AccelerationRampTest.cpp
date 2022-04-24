#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-err58-cpp"

#include "gtest/gtest.h"

#include "Angle.h"
#include "AccelerationRamp.h"

template <uint8_t T_RAMP_STAIRS, uint32_t T_SPR, uint32_t T_MAX_SPEED, uint32_t T_ACCELERATION>
struct FixtureParams
{
    static constexpr uint8_t RAMP_STAIRS = T_RAMP_STAIRS;
    static constexpr uint32_t SPR = T_SPR;
    static constexpr Angle MAX_SPEED = Angle::mrad_u32(T_MAX_SPEED);
    static constexpr Angle ACCELERATION = Angle::mrad_u32(T_ACCELERATION);
};

template <typename PARAMS>
class AccelerationRampTest : public ::testing::Test
{
protected:
    void SetUp() override {}

public:
    using Ramp = AccelerationRamp<PARAMS::RAMP_STAIRS, F_CPU, PARAMS::SPR, PARAMS::MAX_SPEED.mrad_u32(), PARAMS::ACCELERATION.mrad_u32()>;

    const uint8_t RAMP_STAIRS = PARAMS::RAMP_STAIRS;
    const uint32_t SPR = PARAMS::SPR;
    const Angle MAX_SPEED = PARAMS::MAX_SPEED;
    const Angle ACCELERATION = PARAMS::ACCELERATION;
};

using TestTypes = ::testing::Types<
    FixtureParams<64, 400UL * 32UL, Angle::deg(35.46611505122143f * 2.0f).mrad_u32(), Angle::deg(35.46611505122143f * 4.0f).mrad_u32()>,
    FixtureParams<64, 400UL * 64UL, Angle::deg(35.46611505122143f * 2.0f).mrad_u32(), Angle::deg(35.46611505122143f * 4.0f).mrad_u32()>,
    FixtureParams<64, 400UL * 256UL, Angle::deg(35.46611505122143f * 2.0f).mrad_u32(), Angle::deg(35.46611505122143f * 4.0f).mrad_u32()>,
    FixtureParams<128, 400UL * 32UL, Angle::deg(35.46611505122143f * 2.0f).mrad_u32(), Angle::deg(35.46611505122143f * 4.0f).mrad_u32()>,
    FixtureParams<128, 400UL * 64UL, Angle::deg(35.46611505122143f * 2.0f).mrad_u32(), Angle::deg(35.46611505122143f * 4.0f).mrad_u32()>,
    FixtureParams<128, 400UL * 256UL, Angle::deg(35.46611505122143f * 2.0f).mrad_u32(), Angle::deg(35.46611505122143f * 4.0f).mrad_u32()>>;

TYPED_TEST_SUITE(AccelerationRampTest, TestTypes);

TYPED_TEST(AccelerationRampTest, StairZero)
{
    ASSERT_EQ(TestFixture::Ramp::interval(0), UINT32_MAX);
}

TYPED_TEST(AccelerationRampTest, IntervalsDecrementing)
{
    for (size_t i = 0; i < TestFixture::Ramp::STAIRS_COUNT - 1; i++)
    {
        auto i1 = TestFixture::Ramp::interval(i);
        auto i2 = TestFixture::Ramp::interval(i + 1);

        ASSERT_LT(i2, i1);
    }
}

TYPED_TEST(AccelerationRampTest, StairsCount)
{
    ASSERT_EQ(TestFixture::Ramp::STAIRS_COUNT, this->RAMP_STAIRS);
}

TYPED_TEST(AccelerationRampTest, maxAccelSteps_speed_0)
{
    ASSERT_EQ(TestFixture::Ramp::maxAccelStairs(0.0f), 0);
}

TYPED_TEST(AccelerationRampTest, maxAccelSteps_speed_max)
{
    ASSERT_EQ(TestFixture::Ramp::maxAccelStairs(this->MAX_SPEED.rad()), this->RAMP_STAIRS - 1);
}

TYPED_TEST(AccelerationRampTest, maxAccelSteps_speed_half)
{
    ASSERT_EQ(TestFixture::Ramp::maxAccelStairs(this->MAX_SPEED.rad() / 2), this->RAMP_STAIRS / 4);
}

#pragma clang diagnostic pop