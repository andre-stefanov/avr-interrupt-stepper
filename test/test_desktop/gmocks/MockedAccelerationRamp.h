#pragma once

#include "gmock/gmock.h"
#include "AccelerationRamp.h"

using namespace ::testing;

class RampMock
{
public:
    MOCK_METHOD(uint32_t, interval, (unsigned int));
    MOCK_METHOD(uint32_t, setCallback, (timer_callback));
    MOCK_METHOD(uint32_t, getIntervalForSpeed, (float));
    MOCK_METHOD(uint8_t, maxAccelStairs, (float));

    virtual ~RampMock() = default;

    template <typename T_REAL>
    void delegateToReal()
    {
        ON_CALL(*this, interval).WillByDefault([](uint16_t stair)
                                               { return T_REAL::interval(stair); });

        ON_CALL(*this, getIntervalForSpeed).WillByDefault([](float radPerSec)
                                                          { return T_REAL::getIntervalForSpeed(radPerSec); });

        ON_CALL(*this, maxAccelStairs).WillByDefault([](float radPerSec)
                                                     { return T_REAL::maxAccelStairs(radPerSec); });
    }

    template <typename T_REAL>
    void expectLimits()
    {
        EXPECT_CALL(*this, getIntervalForSpeed(_)).Times(AnyNumber());
        EXPECT_CALL(*this, maxAccelStairs(_)).Times(AnyNumber());
        EXPECT_CALL(*this, interval(_)).Times(AnyNumber());
        EXPECT_CALL(*this, interval(0)).Times(0);
        EXPECT_CALL(*this, interval(Ge(T_REAL::STAIRS_COUNT))).Times(0);
    }
};

template <uint16_t T_STAIRS, uint32_t T_FREQ, uint32_t T_SPR, uint32_t T_MAX_SPEED_mRAD, uint32_t T_ACCELERATION_mRAD>
struct MockedAccelerationRamp
{
    using REAL_TYPE = AccelerationRamp<T_STAIRS, T_FREQ, T_SPR, T_MAX_SPEED_mRAD, T_ACCELERATION_mRAD>;

    static RampMock *mock;

    constexpr static uint16_t STAIRS_COUNT = REAL_TYPE::STAIRS_COUNT;
    constexpr static uint8_t STEPS_PER_STAIR = REAL_TYPE::STEPS_PER_STAIR;
    constexpr static uint8_t FIRST_STEP = REAL_TYPE::FIRST_STEP;
    constexpr static uint8_t LAST_STEP = REAL_TYPE::LAST_STEP;

    static void delegateToReal()
    {
        mock->delegateToReal<REAL_TYPE>();
    }

    static void expectLimits()
    {
        mock->expectLimits<REAL_TYPE>();
    }

    static uint32_t interval(uint16_t stair)
    {
        return mock->interval(stair);
    }

    static uint32_t getIntervalForSpeed(float radPerSec)
    {
        return mock->getIntervalForSpeed(radPerSec);
    }

    static uint16_t maxAccelStairs(float radPerSec)
    {
        return mock->maxAccelStairs(radPerSec);
    }
};

template <uint16_t T_STAIRS, uint32_t T_FREQ, uint32_t T_SPR, uint32_t T_MAX_SPEED_mRAD, uint32_t T_ACCELERATION_mRAD>
RampMock *MockedAccelerationRamp<T_STAIRS, T_FREQ, T_SPR, T_MAX_SPEED_mRAD, T_ACCELERATION_mRAD>::mock = nullptr;