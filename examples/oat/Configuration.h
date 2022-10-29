

#include "Angle.h"
#include "Pin.h"
#include "IntervalInterrupt.h"
#include "Driver.h"
#include "Stepper.h"

// CONFIGURATION
#define RA_STEPPER_SPR (400UL * 256UL)
#define RA_TRANSMISSION 35.46611505122143f
#define RA_SLEWING_SPEED 8.0f        // deg/s
#define RA_SLEWING_ACCELERATION 8.0f // deg/s/s
#define RA_GUIDING_SPEED 0.5f        // fraction of sidereal speed to add/substract to/from tracking speed
#define RA_DRIVER_INVERT_STEP false

#define DEC_STEPPER_SPR (400UL * 256UL)
#define DEC_TRANSMISSION 17.70597411692611f
#define DEC_SLEWING_SPEED 2.0f        // deg/s
#define DEC_SLEWING_ACCELERATION 4.0f // deg/s/s
#define DEC_GUIDING_SPEED 0.5f        // fraction of sidereal speed to add/substract to/from tracking speed
#define DEC_DRIVER_INVERT_STEP false

#define SIDEREAL_SECONDS 86164.0905f

namespace config
{
    struct Ra
    {
        constexpr static auto TRANSMISSION = RA_TRANSMISSION;

        constexpr static auto SPR = RA_STEPPER_SPR;

        constexpr static Angle SPEED_SIDEREAL = Angle::deg(360.0f) / SIDEREAL_SECONDS;

        constexpr static Angle SPEED_TRACKING = SPEED_SIDEREAL;
        constexpr static Angle SPEED_SLEWING = Angle::deg(RA_SLEWING_SPEED);
        constexpr static Angle ACCELERATION = Angle::deg(RA_SLEWING_ACCELERATION);

        constexpr static Angle SPEED_GUIDE_POS = SPEED_TRACKING + (SPEED_SIDEREAL * RA_GUIDING_SPEED);
        constexpr static Angle SPEED_GUIDE_NEG = SPEED_TRACKING - (SPEED_SIDEREAL * RA_GUIDING_SPEED);

        using pin_step = Pin<PIN_RA_STEP>;
        using pin_dir = Pin<PIN_RA_DIR>;

        using interrupt = IntervalInterrupt<TIMER_RA>;
        using driver = Driver<SPR, pin_step, pin_dir, RA_DRIVER_INVERT_STEP>;
        using ramp = AccelerationRamp<256, interrupt::FREQ, driver::SPR, (SPEED_SLEWING * TRANSMISSION).mrad_u32(), (ACCELERATION * TRANSMISSION).mrad_u32()>;
        using stepper = Stepper<interrupt, driver, ramp>;
    };

    struct Dec
    {
        constexpr static auto TRANSMISSION = DEC_TRANSMISSION;

        constexpr static auto SPR = DEC_STEPPER_SPR;

        constexpr static Angle SPEED_SIDEREAL = Angle::deg(360.0f) / SIDEREAL_SECONDS;

        constexpr static Angle SPEED_TRACKING = Angle::deg(0.0f);
        constexpr static Angle SPEED_SLEWING = Angle::deg(DEC_SLEWING_SPEED);
        constexpr static Angle ACCELERATION = Angle::deg(DEC_SLEWING_ACCELERATION);

        constexpr static Angle SPEED_GUIDE_POS = SPEED_TRACKING + (SPEED_SIDEREAL * DEC_GUIDING_SPEED);
        constexpr static Angle SPEED_GUIDE_NEG = SPEED_TRACKING - (SPEED_SIDEREAL * DEC_GUIDING_SPEED);

        using pin_step = Pin<PIN_DEC_STEP>;
        using pin_dir = Pin<PIN_DEC_DIR>;

        using interrupt = IntervalInterrupt<TIMER_DEC>;
        using driver = Driver<SPR, pin_step, pin_dir, DEC_DRIVER_INVERT_STEP>;
        using ramp = AccelerationRamp<64, interrupt::FREQ, driver::SPR, (SPEED_SLEWING * TRANSMISSION).mrad_u32(), (ACCELERATION * TRANSMISSION).mrad_u32()>;
        using stepper = Stepper<interrupt, driver, ramp>;
    };
}