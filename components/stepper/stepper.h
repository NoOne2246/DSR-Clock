#ifndef STEPPER_H
#define STEPPER_H

#include "AccelStepper.h"
#include "TMC2209.h"

#define MULTISTEPPER_MAX_STEPPERS 4

typedef struct
{
    /// Array of pointers to the steppers we are controlling.
    /// Fills from 0 onwards
    accelstepper_t* _steppers[MULTISTEPPER_MAX_STEPPERS];

    /// Array of pointers to the steppers we are controlling.
    /// Fills from 0 onwards
    TMC2209_t* _drivers[MULTISTEPPER_MAX_STEPPERS];

    /// Number of steppers we are controlling and the number
    /// of steppers in _steppers[]
    uint8_t       _num_steppers;
} stepper_controller_t;


extern void steppers_initialize(stepper_controller_t* controller);


#endif