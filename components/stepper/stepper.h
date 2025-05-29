#ifndef STEPPER_H
#define STEPPER_H

#include "AccelStepper.h"
#include "TMC2209.h"

#define STEPPER_LIMIT 4

typedef struct
{
    /// Number of steppers we are controlling and the number
    /// of steppers in _steppers[]
    uint8_t _num_steppers;

    /// Array of pointers to the steppers we are controlling.
    /// Fills from 0 onwards
    accelstepper_t *_steppers[STEPPER_LIMIT];

    /// Array of pointers to the steppers we are controlling.
    /// Fills from 0 onwards
    TMC2209_t *_drivers[STEPPER_LIMIT];

    long _last_full_step[STEPPER_LIMIT];     // last position recorded in full steps
    uint8_t _last_micro_step[STEPPER_LIMIT]; // last position recorded in microsteps steps (0-255)
    long _looping_limit[STEPPER_LIMIT];      // Point at which the position should loop.

    // simultaneously move motor
    double _fraction[STEPPER_LIMIT]; // percentage of main motor speed to use.
    int8_t _controlling;             // controlling motor index.
    float _last_speed;
} stepper_controller_t;

/**
 * @brief Initializes the stepper controller.
 *
 * Prepares the controller for managing stepper motors.
 * @param controller Pointer to the stepper controller structure.
 */
extern void steppers_initialize(stepper_controller_t *controller);

/**
 * @brief Adds a stepper motor to the controller with specified pin and UART configurations.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param stepPin GPIO pin for step control.
 * @param dirPin GPIO pin for direction control.
 * @param uartPort UART port for communication.
 * @param rxPin GPIO pin for RX.
 * @param txPin GPIO pin for TX.
 * @param Address Address for TMC2209 UART communication.
 * @return Index of the added stepper motor.
 */
extern uint8_t steppers_addStepper(stepper_controller_t *controller, gpio_num_t stepPin, gpio_num_t dirPin, uart_port_t uartPort, gpio_num_t rxPin, gpio_num_t txPin, tmc_SerialAddress Address);

/**
 * @brief Configures the number of steps per revolution.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param looping_limit Maximum revolution limit.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_setupRevolution(stepper_controller_t *controller, uint8_t stepperIndex, long looping_limit);

extern uint8_t steppers_setCurrentPercentages(stepper_controller_t *controller, uint8_t stepperIndex, uint8_t runCurrentPercent, uint8_t holdCurrentPercent, uint8_t holdDelayPercent);

extern uint8_t steppers_setRMSCurrent(stepper_controller_t *controller, uint8_t stepperIndex, float rmsCurrent, float rSense, float holdMultiplier);

extern uint8_t steppers_setMaxSpeed(stepper_controller_t *controller, uint8_t stepperIndex, float speed);

// === Stepper Enable/Disable Functions ===

/**
 * @brief Enables a specific stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepper Index of the stepper to enable.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_enableStepper(stepper_controller_t *controller, uint8_t stepper);
/**
 * @brief Disables a specific stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepper Index of the stepper to disable.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_disableStepper(stepper_controller_t *controller, uint8_t stepper);

/**
 * @brief Enables all stepper motors.
 * @param controller Pointer to the stepper controller structure.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_enableAll(stepper_controller_t *controller);

/**
 * @brief Disables all stepper motors.
 * @param controller Pointer to the stepper controller structure.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_disableAll(stepper_controller_t *controller);

// === Stealth Configuration ===

/**
 * @brief Enables stealth mode for reduced noise and smooth operation.
 *
 * This function configures the TMC2209 driver to use StealthChop mode.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_enableStealth(stepper_controller_t *controller, uint8_t stepperIndex);

/**
 * @brief Disables stealth mode and switches to SpreadCycle mode.
 *
 * This function configures the TMC2209 driver to use SpreadCycle mode instead of StealthChop.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_disableStealth(stepper_controller_t *controller, uint8_t stepperIndex);

// === Microstepping Configuration ===

/**
 * @brief Sets the microstepping resolution for all motors.
 * @param controller Pointer to the stepper controller structure.
 * @param Microsteps Array of microstep values for each stepper.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_setMicrostepAll(stepper_controller_t *controller, uint16_t microsteps[]);

/**
 * @brief Sets the microstepping resolution for a specific motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param Microsteps Microstep value.
 * @return number of microsteps set.
 */
extern uint16_t steppers_setMicrostep(stepper_controller_t *controller, uint8_t stepperIndex, uint16_t microsteps);

// === Position Queries ===

/**
 * @brief Retrieves the current absolute position of a stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Absolute position.
 */
extern double steppers_currentPosition(stepper_controller_t *controller, uint8_t stepperIndex);

/**
 * @brief Retrieves the current absolute position of a stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Percentage of a full revolution.
 */
extern double steppers_currentPositionPercent(stepper_controller_t *controller, uint8_t stepperIndex);

/**
 * @brief Retrieves the target absolute position of a stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Target position.
 */
extern double steppers_targetPosition(stepper_controller_t *controller, uint8_t stepperIndex);
/**
 * @brief Retrieves the target absolute position of a stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Percentage of a full revolution.
 */
extern double steppers_targetPositionPercent(stepper_controller_t *controller, uint8_t stepperIndex);

// === Movement Functions ===

/**
 * @brief Moves all steppers to an absolute position.
 * @param controller Pointer to the stepper controller structure.
 * @param absolute Array of target absolute positions.
 */
extern void steppers_moveTo(stepper_controller_t *controller, double absolute[]);

/**
 * @brief Moves a specific stepper motor to an absolute position.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param absolute Target position.
 * @return Steps to be taken.
 */
extern long steppers_moveToStepper(stepper_controller_t *controller, uint8_t stepperIndex, double absolute);

/**
 * @brief Moves all steppers by a relative displacement.
 * @param controller Pointer to the stepper controller structure.
 * @param relative Array of steps to move.
 */
extern void steppers_move(stepper_controller_t *controller, double relative[]);

/**
 * @brief Moves a specific stepper motor by a relative displacement.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param relative Displacement value.
 * @return Steps to be taken.
 */
extern long steppers_moveStepper(stepper_controller_t *controller, uint8_t stepperIndex, double relative);

/**
 * @brief Moves all steppers to an absolute position expressed as a percent of the loop.
 * @param controller Pointer to the stepper controller structure.
 * @param percent Array of target percent of full loop positions.
 */
extern void steppers_moveToPercent(stepper_controller_t *controller, double percent[]);

/**
 * @brief Moves a specific stepper motor to an absolute position expressed as a percent of the loop.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param percent Percent of full loop position.
 * @return Steps to be taken.
 */
extern long steppers_moveToStepperPercent(stepper_controller_t *controller, uint8_t stepperIndex, double percent);

/**
 * @brief Moves all steppers to the specified absolute positions simultaneously.
 *
 * This function sends movement commands to all stepper motors at the same time,
 * ensuring coordinated motion across multiple axes.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param absolute Array containing absolute target positions for each stepper.
 */
extern void steppers_moveToSimultaneous(stepper_controller_t *controller, double absolute[]);

/**
 * @brief Moves all steppers by the specified relative amounts simultaneously.
 *
 * This function applies a relative movement to all stepper motors at the same time.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param relative Array containing relative movement values for each stepper.
 */
extern void steppers_moveSimultaneous(stepper_controller_t *controller, double relative[]);

// === Speed Functions ===

/**
 * @brief Sets the speed of a specific stepper motor in Steps per Second.
 *
 * This function updates the movement speed of the specified stepper motor. The speed value
 * is typically measured in steps per second or another relevant unit depending on the motion control system.
 * The function ensures that the speed setting remains within the acceptable range for the motor driver.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor to update.
 * @param speed Desired speed value (steps per second).
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_setSpeedSPS(stepper_controller_t *controller, uint8_t stepperIndex, double speed);

/**
 * @brief Sets the speed of a stepper motor in revolutions per minute (RPM).
 *
 * Converts the given RPM value into steps per second, based on the configured **steps per revolution**.
 * The function ensures proper scaling and sets the speed accordingly.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param revolutions Desired revolutions of the loop per minute.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_setSpeedRPM(stepper_controller_t *controller, uint8_t stepperIndex, double revolutions);

/**
 * @brief Sets the movement period for a stepper motor.
 *
 * Defines the time duration between individual steps. This value is used to calculate
 * the stepper speed in terms of **seconds per step**.
 *
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @param seconds Duration for 1 revolution of the wheel in seconds.
 * @return Success flag (1 if successful, 0 otherwise).
 */
extern uint8_t steppers_setSpeedPeriod(stepper_controller_t *controller, uint8_t stepperIndex, double seconds);

// === Run Functions (Continuous Motion) ===

/**
 * @brief Runs all stepper motors towards their target positions.
 * @param controller Pointer to the stepper controller structure.
 * @return Whether a stepper still needs to move to reach it's destination.
 */
extern uint8_t steppers_run(stepper_controller_t *controller);

/**
 * @brief Runs all stepper motors at their specific speed.
 * @param controller Pointer to the stepper controller structure.
 * @return Whether a stepper still needs to move to reach it's destination.
 */
extern uint8_t steppers_runSpeed(stepper_controller_t *controller);

/**
 * @brief Runs all stepper motors at roughtly simultaneous speed.
 * @param controller Pointer to the stepper controller structure.
 * @return Whether a stepper still needs to move to reach it's destination.
 */
extern uint8_t steppers_runSimultaneous(stepper_controller_t *controller);

/**
 * @brief Runs a specific stepper towards its target position.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Whether the stepper still needs to move to reach it's destination.
 */
extern uint8_t steppers_runStepper(stepper_controller_t *controller, uint8_t stepperIndex);

/**
 * @brief Runs a specific stepper at a specific speed.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 * @return Whether the stepper still needs to move to reach it's destination.
 */
extern uint8_t steppers_runSpeedStepper(stepper_controller_t *controller, uint8_t stepperIndex);

// === Stopping Functions ===

/**
 * @brief Stops all stepper motors immediately.
 * @param controller Pointer to the stepper controller structure.
 */
extern void steppers_stopAll(stepper_controller_t *controller);

/**
 * @brief Stops a specific stepper motor.
 * @param controller Pointer to the stepper controller structure.
 * @param stepperIndex Index of the stepper motor.
 */
extern void steppers_stopStepper(stepper_controller_t *controller, uint8_t stepperIndex);

extern uint8_t steppers_setPosition(stepper_controller_t *controller, double position[]);
extern uint8_t steppers_setStepperPosition(stepper_controller_t *controller, uint8_t stepperIndex, double position);

#endif