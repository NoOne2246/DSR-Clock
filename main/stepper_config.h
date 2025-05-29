#ifndef STEPPER_CONFIG_H
#define STEPPER_CONFIG_H

// Sanyo Denki SH2141-5541 Motor Settings
#define SANYO_IRUN_SLOW     70  // Run Current (Slow Speed)
#define SANYO_IHOLD_SLOW    50  // Hold Current (Slow Speed)
#define SANYO_IHOLD_DELAY_SLOW  70  // Hold Delay (Slow Speed)

#define SANYO_IRUN_FAST     90  // Run Current (High Speed)
#define SANYO_IHOLD_FAST    80  // Hold Current (High Speed)
#define SANYO_IHOLD_DELAY_FAST  30  // Hold Delay (High Speed)

#define SANYO_IRUN_ULTRA    100  // Run Current (Ultra Speed)
#define SANYO_IHOLD_ULTRA   80  // Hold Current (Ultra Speed)
#define SANYO_IHOLD_DELAY_ULTRA  10  // Hold Delay (Ultra Speed)

#define SANYO_RMS_CURRENT   300   // RMS Current (mA)
#define SANYO_RSENSE        0.11  // rSense Value
#define SANYO_HOLD_MULTIPLIER  0.6  // Hold Current Multiplier

#define SANYO_STEP_MAX       1   // No microstepper
#define SANYO_STEP_ADJUST    8   // Microstepping for Daylight Savings
#define SANYO_STEP_REGULAR   32  // Microstepping for Regular Use

#define SANYO_MAX_SPEED      1000

// Lin Engineering WO-208-13-01-RO Motor Settings
#define LIN_IRUN_SLOW       50  // Run Current (Slow Speed)
#define LIN_IHOLD_SLOW      40  // Hold Current (Slow Speed)
#define LIN_IHOLD_DELAY_SLOW  80  // Hold Delay (Slow Speed)

#define LIN_IRUN_FAST       90  // Run Current (High Speed)
#define LIN_IHOLD_FAST      80  // Hold Current (High Speed)
#define LIN_IHOLD_DELAY_FAST  20  // Hold Delay (High Speed)

#define LIN_IRUN_ULTRA      100  // Run Current (Ultra Speed)
#define LIN_IHOLD_ULTRA     100  // Hold Current (Ultra Speed)
#define LIN_IHOLD_DELAY_ULTRA  10  // Hold Delay (Ultra Speed)

#define LIN_RMS_CURRENT     600   // RMS Current (mA)
#define LIN_RSENSE          0.11  // rSense Value
#define LIN_HOLD_MULTIPLIER  0.6  // Hold Current Multiplier

#define LIN_STEP_MAX      1   // No microstepper
#define LIN_STEP_ADJUST   16  // Microstepping for Daylight Savings
#define LIN_STEP_REGULAR    64  // Microstepping for Regular Use

#define LIN_MAX_SPEED      1000

#endif // STEPPER_CONTROL_H
