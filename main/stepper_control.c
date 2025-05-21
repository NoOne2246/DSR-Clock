#include "stepper_control.h"
#include "gpio_definitions.h"
#include "AccelStepper.h"


// Define stepper motor pins and AccelStepper object


// AccelStepper Stepper_Hour(AccelStepper::DRIVER, MOTOR_HOUR_STEP, MOTOR_HOUR_DIR);
// AccelStepper Stepper_Min(AccelStepper::DRIVER, MOTOR_MIN_STEP, MOTOR_MIN_DIR);

// void stepper_init() {

//     Stepper_Hour.setMaxSpeed(1000);  // Set max speed
//     Stepper_Hour.setAcceleration(500); // Set acceleration
//     Stepper_Min.setMaxSpeed(1000);  // Set max speed
//     Stepper_Min.setAcceleration(500); // Set acceleration
// }

// Function declarations
void stepper_init(){}
void homeStepper(){}