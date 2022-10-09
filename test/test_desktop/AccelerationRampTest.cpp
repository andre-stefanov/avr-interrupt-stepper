#include "gtest/gtest.h"

#include "Angle.h"
#include "AccelerationRamp.h"

#define OAT_RA_TRANSMISSION 35.46611505122143f
#define OAT_RA_DEG(d) Angle::deg(OAT_RA_TRANSMISSION *static_cast<float>(d)).mrad_u32()

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
    FixtureParams<32, 400UL * 32UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<32, 400UL * 64UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<32, 400UL * 128UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<64, 400UL * 32UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<64, 400UL * 64UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<64, 400UL * 128UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<64, 400UL * 256UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<128, 400UL * 32UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<128, 400UL * 64UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<128, 400UL * 128UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>,
    FixtureParams<128, 400UL * 256UL, OAT_RA_DEG(2), OAT_RA_DEG(4)>>;

/*class AccelerationRampTestNames
{
public:
    template <typename T>
    static std::string GetName(int i)
    {
        std::stringstream ss;
        ss << "<";
        ss << "STAIRS=" + std::to_string(T::RAMP_STAIRS);
        ss << ",SPR=" + std::to_string(T::SPR);
        ss << ",MAX_SPEED=" + std::to_string(T::MAX_SPEED.mrad_u32());
        ss << ",ACCEL=" + std::to_string(T::ACCELERATION.mrad_u32());
        ss << ">";
        return ss.str();
    }
};*/

TYPED_TEST_SUITE(AccelerationRampTest, TestTypes/*, AccelerationRampTestNames*/);

TYPED_TEST(AccelerationRampTest, interval_0)
{
    ASSERT_EQ(TestFixture::Ramp::interval(0), UINT32_MAX);
}

TYPED_TEST(AccelerationRampTest, interval_decrementing)
{
    for (size_t i = 0; i < TestFixture::Ramp::STAIRS_COUNT - 1; i++)
    {
        auto i1 = TestFixture::Ramp::interval(i);
        auto i2 = TestFixture::Ramp::interval(i + 1);

        ASSERT_LT(i2, i1);
    }
}

TYPED_TEST(AccelerationRampTest, STAIRS_COUNT)
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