#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include <AccelStepper.h>

#ifdef __cplusplus
extern "C" {
#endif


// Function declarations
void stepper_init();
void homeStepper();

#ifdef __cplusplus
}
#endif


#endif // STEPPER_CONTROL_H
