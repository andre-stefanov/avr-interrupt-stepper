#include "gtest/gtest.h"

#include "AccelerationRamp.h"

#define TRANSMISSION 35.46611505122143f
#define SPR (400 * 256)
#define DEG_TO_STEPS(d) static_cast<uint32_t>((d / 360.f) * SPR * TRANSMISSION)

template<
        uint16_t T_RAMP_STAIRS,
        uint32_t T_MAX_SPEED,
        uint32_t T_ACCELERATION
>
struct FixtureParams {
    static constexpr uint16_t RAMP_STAIRS = T_RAMP_STAIRS;
    static constexpr uint32_t MAX_SPEED = T_MAX_SPEED;
    static constexpr uint32_t ACCELERATION = T_ACCELERATION;
};

template<typename PARAMS>
class AccelerationRampTest : public ::testing::Test {
protected:
    void SetUp() override {}

public:
    using Ramp = AccelerationRamp<PARAMS::RAMP_STAIRS, F_CPU, PARAMS::MAX_SPEED, PARAMS::ACCELERATION>;

    const uint16_t RAMP_STAIRS = PARAMS::RAMP_STAIRS;
    const uint32_t MAX_SPEED = PARAMS::MAX_SPEED;
    const uint32_t ACCELERATION = PARAMS::ACCELERATION;
};

using TestTypes = ::testing::Types<
        FixtureParams<32, DEG_TO_STEPS(2), DEG_TO_STEPS(4)>,
        FixtureParams<64, DEG_TO_STEPS(2), DEG_TO_STEPS(4)>,
        FixtureParams<128, DEG_TO_STEPS(2), DEG_TO_STEPS(4)>,
        FixtureParams<256, DEG_TO_STEPS(4), DEG_TO_STEPS(4)>,
        FixtureParams<512, DEG_TO_STEPS(8), DEG_TO_STEPS(8)>
>;

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

TYPED_TEST(AccelerationRampTest, interval_0) {
    ASSERT_EQ(TestFixture::Ramp::interval(0), UINT32_MAX);
}

TYPED_TEST(AccelerationRampTest, interval_decrementing) {
    for (size_t i = 0; i < TestFixture::Ramp::STAIRS_COUNT - 1; i++) {
        auto i1 = TestFixture::Ramp::interval(i);
        auto i2 = TestFixture::Ramp::interval(i + 1);

        ASSERT_LE(i2, i1);
    }
}

TYPED_TEST(AccelerationRampTest, lessEqual_maxSpeed) {
    for (size_t i = 0; i < TestFixture::Ramp::STAIRS_COUNT - 1; i++) {
        auto stairInterval = TestFixture::Ramp::interval(i);
        auto maxSpeedInterval = F_CPU / TestFixture::MAX_SPEED;
        ASSERT_GE(stairInterval, maxSpeedInterval);
    }
}

TYPED_TEST(AccelerationRampTest, STAIRS_COUNT) {
    ASSERT_EQ(TestFixture::Ramp::STAIRS_COUNT, this->RAMP_STAIRS);
}

TYPED_TEST(AccelerationRampTest, maxAccelSteps_speed_0) {
    ASSERT_EQ(TestFixture::Ramp::maxAccelStairs(0.0f), 0);
}

TYPED_TEST(AccelerationRampTest, maxAccelSteps_speed_max) {
    ASSERT_EQ(TestFixture::Ramp::maxAccelStairs(this->MAX_SPEED), this->RAMP_STAIRS - 1);
}

TYPED_TEST(AccelerationRampTest, maxAccelSteps_speed_half) {
    ASSERT_EQ(TestFixture::Ramp::maxAccelStairs(this->MAX_SPEED / 2.0f), this->RAMP_STAIRS / 4.0f);
}