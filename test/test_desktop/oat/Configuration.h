#ifndef OAT_TEST_CONFIGURATION_H
#define OAT_TEST_CONFIGURATION_H

// Seconds per astronomical day (23h 56m 4.0905s)
#define SIDEREAL_SECONDS_PER_DAY 86164.0905f

#define RA_TRANSMISSION 35.46611505122143f
#define RA_SLEWING_SPEED 4.0f        // deg/s
#define RA_SLEWING_ACCELERATION 4.0f // deg/s/s
#define RA_GUIDING_SPEED 0.5f        // fraction of sidereal speed to add/substract to/from tracking speed
#define RA_DRIVER_INVERT_DIR false
#define RA_STEPPER_SPR 400
#define RA_MICROSTEPPING 256
#define RA_STEP_PIN 54
#define RA_DIR_PIN 55

#define DEC_TRANSMISSION 17.70597411692611f
#define DEC_SLEWING_SPEED 4.0f        // deg/s
#define DEC_SLEWING_ACCELERATION 4.0f // deg/s/s
#define DEC_GUIDING_SPEED 0.5f        // fraction of sidereal speed to add/substract to/from tracking speed
#define DEC_DRIVER_INVERT_DIR false
#define DEC_STEPPER_SPR 400
#define DEC_MICROSTEPPING 256
#define DEC_STEP_PIN 60
#define DEC_DIR_PIN 61

#endif //OAT_TEST_CONFIGURATION_H
