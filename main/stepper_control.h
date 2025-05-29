#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <stdint.h>

typedef enum {
    REGULAR_CLOCK=0,
    ADJUST_CLOCK=1,
    REWIND_CLOCK=2,
    FORWARD_CLOCK=3,
    HOMING_CLOCK=4,
}clock_Mode_t;

// Function declarations
extern void stepper_init(void);
extern void homeSteppers(void);
extern uint8_t rewindTime(void);
extern uint8_t forwardTime(void);
extern void stepperTest(void * pvParameter);

#endif // STEPPER_CONTROL_H
