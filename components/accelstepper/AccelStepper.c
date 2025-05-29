/**
 ******************************************************************************
 * @file    AccelStepper.c
 * @author  Christopher Ho
 * @version V1.0.0
 * @date    21-05-2025
 * @brief   This is a modified version of AccelStepper Arduino library ported to
 *          esp32 microcontrollers.
            This code is based on the stm32F4xx port obtained at:
			<https://github.com/VojislavM/UART_TLV/>. This 
			version is Copyright (C) 2016 Matej Gomboc, Institute IRNAS Raèe
 *          The original version of this library can be obtained at:
 *          <http://www.airspayce.com/mikem/arduino/AccelStepper/>. The original
 *          version is Copyright (C) 2009-2018 Mike McCauley.
 ******************************************************************************
 **/

#include <math.h>
#include "AccelStepper.h"
#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#undef max
#define max(a, b) ((a) > (b) ? (a) : (b))
#undef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#undef HIGH
#define HIGH 1
#undef LOW
#define LOW 0


/// Called to execute a step using stepper functions (pins = 0) Only called when a new step is
/// required. Calls _forward() or _backward() to perform the step
/// \param[in] step The current step phase number (0 to 7)
static void step0(accelstepper_t* motor, long step);

/// Called to execute a step on a stepper driver (ie where pins == 1). Only called when a new step is
/// required. Sets or clears the outputs of Step pin1 to step,
/// and sets the output of _pin2 to the desired direction. The Step pin (_pin1) is pulsed for 1 microsecond
/// which is the minimum STEP pulse width for the 3967 driver.
/// \param[in] step The current step phase number (0 to 7)
static void step1(accelstepper_t* motor, long step);

/// Called to execute a step on a 2 pin motor. Only called when a new step is
/// required. Sets or clears the outputs of pin1 and pin2.
/// \param[in] step The current step phase number (0 to 7)
static void step2(accelstepper_t* motor, long step);

/// Called to execute a step on a 3 pin motor, such as HDD spindle. Only called when a new step is
/// required. Sets or clears the outputs of pin1, pin2, pin3.
/// \param[in] step The current step phase number (0 to 7)
static void step3(accelstepper_t* motor, long step);

/// Called to execute a step on a 4 pin motor. Only called when a new step is
/// required. Sets or clears the outputs of pin1, pin2, pin3, pin4.
/// \param[in] step The current step phase number (0 to 7)
static void step4(accelstepper_t* motor, long step);

/// Called to execute a step on a 3 pin motor, such as HDD spindle. Only called when a new step is
/// required. Sets or clears the outputs of pin1, pin2, pin3.
/// \param[in] step The current step phase number (0 to 7)
static void step6(accelstepper_t* motor, long step);

/// Called to execute a step on a 4 pin half-steper motor. Only called when a new step is
/// required. Sets or clears the outputs of pin1, pin2, pin3, pin4.
/// \param[in] step The current step phase number (0 to 7)
static void step8(accelstepper_t* motor, long step);

void astepper_moveTo(accelstepper_t* motor, long absolute)
{
    if (motor->_targetPos != absolute)
    {
    	motor->_targetPos = absolute;
		astepper_computeNewSpeed(motor);
		// compute new n?
    }
}

void astepper_move(accelstepper_t* motor, long relative)
{
    astepper_moveTo(motor, motor->_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
uint8_t astepper_runSpeed(accelstepper_t* motor)
{
    // Dont do anything unless we actually have a step interval
    if (!motor->_stepInterval) return 0; // false

	unsigned long time = pdTICKS_TO_MS( xTaskGetTickCount() );
	
	if(time - motor->_lastStepTime >= motor->_stepInterval)
	{
		if (motor->_direction == DIRECTION_CW)
		{	// Clockwise
			motor->_currentPos += 1;
		}
		else
		{	// Anticlockwise
			motor->_currentPos -= 1;
		}

		astepper_step(motor, motor->_currentPos);

		motor->_lastStepTime = time;

		return 1; // true
    }
    else
    {
    	return 0; // false
    }
}

long astepper_distanceToGo(accelstepper_t* motor)
{
    return (motor->_targetPos - motor->_currentPos);
}

long astepper_targetPosition(accelstepper_t* motor)
{
    return motor->_targetPos;
}

long astepper_currentPosition(accelstepper_t* motor)
{
    return motor->_currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void astepper_setCurrentPosition(accelstepper_t* motor, long position)
{
	motor->_targetPos = motor->_currentPos = position;
	motor->_n = 0;
	motor->_stepInterval = 0;
	motor->_speed = 0.0;
}

void astepper_computeNewSpeed(accelstepper_t* motor)
{
    long distanceTo = astepper_distanceToGo(motor); // +ve is clockwise from curent location

    long stepsToStop = (long)((motor->_speed * motor->_speed) / (2.0 * motor->_acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
    	// We are at the target and its time to astepper_stop
    	motor->_stepInterval = 0;
    	motor->_speed = 0.0;
    	motor->_n = 0;
    	return;
    }

    if (distanceTo > 0)
    {
		// We are anticlockwise from the target
		// Need to go clockwise from here, maybe decelerate now
		if (motor->_n > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= distanceTo) || motor->_direction == DIRECTION_CCW)
				motor->_n = -stepsToStop; // Start deceleration
		}
		else if (motor->_n < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < distanceTo) && motor->_direction == DIRECTION_CW)
				motor->_n = -motor->_n; // Start accceleration
		}
	}
	else if (distanceTo < 0)
	{
		// We are clockwise from the target
		// Need to go anticlockwise from here, maybe decelerate
		if (motor->_n > 0)
		{
			// Currently accelerating, need to decel now? Or maybe going the wrong way?
			if ((stepsToStop >= -distanceTo) || motor->_direction == DIRECTION_CW)
				motor->_n = -stepsToStop; // Start deceleration
		}
		else if (motor->_n < 0)
		{
			// Currently decelerating, need to accel again?
			if ((stepsToStop < -distanceTo) && motor->_direction == DIRECTION_CCW)
				motor->_n = -motor->_n; // Start accceleration
		}
	}

	// Need to accelerate or decelerate
	if (motor->_n == 0)
	{
		// First step from stopped
		motor->_cn = motor->_c0;
		motor->_direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
	}
	else
	{
		// Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
		motor->_cn = motor->_cn - ((2.0 * motor->_cn) / ((4.0 * motor->_n) + 1)); // Equation 13
		motor->_cn = (motor->_cn > motor->_cmin) ? motor->_cn : motor->_cmin; //max(motor->_cn, motor->_cmin);
	}

	motor->_n++;
	motor->_stepInterval = motor->_cn;
	motor->_speed = 1000000.0 / motor->_cn;

	if (motor->_direction == DIRECTION_CCW)
		motor->_speed = -motor->_speed;

#ifdef DEBUG_MODE
    printf("%f\n", motor->_speed);
    printf("%f\n", motor->_acceleration);
    printf("%f\n", motor->_cn);
    printf("%f\n", motor->_c0);
    printf("%ld\n", motor->_n);
    printf("%lu\n", motor->_stepInterval);
//    Serial.println(distanceTo);
//    Serial.println(stepsToStop);
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
uint8_t astepper_run(accelstepper_t* motor)
{
    if (astepper_runSpeed(motor)) astepper_computeNewSpeed(motor);

    return motor->_speed != 0.0 || astepper_distanceToGo(motor) != 0;
}

void astepper_initialize(accelstepper_t* motor, uint8_t interface, gpio_num_t GPIOxPin1, gpio_num_t GPIOxPin2, gpio_num_t GPIOxPin3, gpio_num_t GPIOxPin4, uint8_t enable)
{
	motor->_interface = interface;
	motor->_currentPos = 0;
	motor->_targetPos = 0;
	motor->_speed = 0.0;
	motor->_maxSpeed = 1.0;
	motor->_acceleration = 0.0;
	motor->_sqrt_twoa = 1.0;
	motor->_stepInterval = 0;
	motor->_minPulseWidth = 1;
	motor->_GPIOxEnablePin = GPIO_NUM_NC;
	motor->_lastStepTime = 0;
	motor->_GPIOxPin[0] = GPIOxPin1;
	motor->_GPIOxPin[1] = GPIOxPin2;
	motor->_GPIOxPin[2] = GPIOxPin3;
	motor->_GPIOxPin[3] = GPIOxPin4;

    // NEW
	motor->_n = 0;
	motor->_c0 = 0.0;
    motor->_cn = 0.0;
    motor->_cmin = 1.0;
    motor->_direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
    	motor->_pinInverted[i] = 0;

    if (enable)
    	astepper_enableOutputs(motor);

    // Some reasonable default
    astepper_setAcceleration(motor, 1);
}

void astepper_InitStepperFunct(accelstepper_t* motor, void (*forward)(), void (*backward)())
{
	motor->_interface = 0;
	motor->_currentPos = 0;
	motor->_targetPos = 0;
	motor->_speed = 0.0;
	motor->_maxSpeed = 1.0;
	motor->_acceleration = 0.0;
	motor->_sqrt_twoa = 1.0;
	motor->_stepInterval = 0;
	motor->_minPulseWidth = 1;
	motor->_GPIOxEnablePin = GPIO_NUM_NC;
	motor->_lastStepTime = 0;
	motor->_GPIOxPin[0] = GPIO_NUM_NC;
	motor->_GPIOxPin[1] = GPIO_NUM_NC;
	motor->_GPIOxPin[2] = GPIO_NUM_NC;
	motor->_GPIOxPin[3] = GPIO_NUM_NC;
	motor->_forward = forward;
	motor->_backward = backward;

    // NEW
	motor->_n = 0;
	motor->_c0 = 0.0;
	motor->_cn = 0.0;
	motor->_cmin = 1.0;
	motor->_direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
    	motor->_pinInverted[i] = 0;

    // Some reasonable default
    astepper_setAcceleration(motor, 1);
}

void astepper_setMaxSpeed(accelstepper_t* motor, float speed)
{
    if (motor->_maxSpeed != speed)
    {
    	motor->_maxSpeed = speed;
    	motor->_cmin = 1000000.0 / speed;
		// Recompute _n from current speed and adjust speed if accelerating or cruising
		if (motor->_n > 0)
		{
			motor->_n = (long)((motor->_speed * motor->_speed) / (2.0 * motor->_acceleration)); // Equation 16
			astepper_computeNewSpeed(motor);
		}
    }
}

float astepper_maxSpeed(accelstepper_t* motor)
{
    return motor->_maxSpeed;
}

void astepper_setAcceleration(accelstepper_t* motor, float acceleration)
{
    if (acceleration == 0.0) return;
	if (acceleration < 0.0)  acceleration = -acceleration;
    if (motor->_acceleration != acceleration)
    {
	    // Recompute _n per Equation 17
    	motor->_n = motor->_n * (motor->_acceleration / acceleration);
		// New c0 per Equation 7, with correction per Equation 15
    	motor->_c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0;// Equation 15
    	motor->_acceleration = acceleration;
		astepper_computeNewSpeed(motor);
    }
}

void astepper_setSpeed(accelstepper_t* motor, float speed)
{
    if (speed == motor->_speed)
        return;

    speed = constrain(speed, -motor->_maxSpeed, motor->_maxSpeed);

    if (speed == 0.0)
    {
    	motor->_stepInterval = 0;
    }
    else
    {
    	motor->_stepInterval = fabs(1000000.0 / speed);
    	motor->_direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }

    motor->_speed = speed;
}

float astepper_speed(accelstepper_t* motor)
{
    return motor->_speed;
}

// Subclasses can override
void astepper_step(accelstepper_t* motor, long step)
{
    switch (motor->_interface)
    {
        case FUNCTION:
            step0(motor, step);
            break;

		case DRIVER:
			step1(motor, step);
			break;

		case FULL2WIRE:
			step2(motor, step);
			break;

		case FULL3WIRE:
			step3(motor, step);
			break;

		case FULL4WIRE:
			step4(motor, step);
			break;

		case HALF3WIRE:
			step6(motor, step);
			break;

		case HALF4WIRE:
			step8(motor, step);
			break;
    }
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void astepper_setOutputPins(accelstepper_t* motor, uint8_t mask)
{
    uint8_t numpins = 2;
    if (motor->_interface == FULL4WIRE || motor->_interface == HALF4WIRE) numpins = 4;
    else if (motor->_interface == FULL3WIRE || motor->_interface == HALF3WIRE) numpins = 3;

	uint8_t i;
	for (i = 0; i < numpins; i++)
	{
		gpio_set_level(motor->_GPIOxPin[i], (mask & (1 << i)) ? (HIGH ^ motor->_pinInverted[i]) : (LOW ^ motor->_pinInverted[i]));
	}
}

// 0 pin step function (ie for functional usage)
void step0(accelstepper_t* motor, long step)
{
	if (motor->_speed > 0)
		motor->_forward();
	else
		motor->_backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void step1(accelstepper_t* motor, long step)
{
    // _pin[0] is step, _pin[1] is direction
    astepper_setOutputPins(motor, motor->_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    astepper_setOutputPins(motor, motor->_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time
    // Delay the minimum allowed pulse width
	vTaskDelay(motor->_minPulseWidth / (portTICK_PERIOD_MS * 1000));
    astepper_setOutputPins(motor, motor->_direction ? 0b10 : 0b00); // step LOW

}


// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void step2(accelstepper_t* motor, long step)
{
    switch (step & 0x3)
    {
		case 0: /* 01 */
			astepper_setOutputPins(motor, 0b10);
			break;

		case 1: /* 11 */
			astepper_setOutputPins(motor, 0b11);
			break;

		case 2: /* 10 */
			astepper_setOutputPins(motor, 0b01);
			break;

		case 3: /* 00 */
			astepper_setOutputPins(motor, 0b00);
			break;
    }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void astepper_step3(accelstepper_t* motor, long step)
{
    switch (step % 3)
    {
		case 0:    // 100
			astepper_setOutputPins(motor, 0b100);
			break;

		case 1:    // 010
			astepper_setOutputPins(motor, 0b010);
			break;

		case 2:    //001
			astepper_setOutputPins(motor, 0b001);
			break;
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void step4(accelstepper_t* motor, long step)
{
    switch (step & 0x3)
    {
		case 0:    // 1010
			astepper_setOutputPins(motor, 0b0101);
			break;

		case 1:    // 0110
			astepper_setOutputPins(motor, 0b0110);
			break;

		case 2:    //0101
			astepper_setOutputPins(motor, 0b1010);
			break;

		case 3:    //1001
			astepper_setOutputPins(motor, 0b1001);
			break;
    }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void step6(accelstepper_t* motor, long step)
{
    switch (step % 6)
    {
		case 0:    // 100
			astepper_setOutputPins(motor, 0b100);
			break;

		case 1:    // 110
			astepper_setOutputPins(motor, 0b110);
			break;

		case 2:    // 010
			astepper_setOutputPins(motor, 0b010);
			break;

		case 3:    // 011
			astepper_setOutputPins(motor, 0b011);
			break;

		case 4:    // 001
			astepper_setOutputPins(motor, 0b001);
			break;

		case 5:    // 101
			astepper_setOutputPins(motor, 0b101);
			break;
    }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void step8(accelstepper_t* motor, long step)
{
    switch (step & 0x7)
    {
		case 0:    // 1000
			astepper_setOutputPins(motor, 0b1000);
			break;

		case 1:    // 1100
			astepper_setOutputPins(motor, 0b1100);
			break;

		case 2:    // 0100
			astepper_setOutputPins(motor, 0b0100);
			break;

		case 3:    // 0110
			astepper_setOutputPins(motor, 0b0110);
			break;

		case 4:    // 0010
			astepper_setOutputPins(motor, 0b0010);
			break;

		case 5:    //0011
			astepper_setOutputPins(motor, 0b0011);
			break;

		case 6:    // 0001
			astepper_setOutputPins(motor, 0b0001);
			break;

		case 7:    //1001
			astepper_setOutputPins(motor, 0b1001);
			break;
    }
}

// Prevents power consumption on the outputs
void astepper_disableOutputs(accelstepper_t* motor)
{
    if (! motor->_interface) return;

    astepper_setOutputPins(motor, 0); // Handles inversion automatically

    if (motor->_GPIOxEnablePin != GPIO_NUM_NC) // If enable pin used
    {
		gpio_reset_pin(motor->_GPIOxEnablePin);
        gpio_set_direction(motor->_GPIOxEnablePin, GPIO_MODE_OUTPUT);
        gpio_set_level(motor->_GPIOxEnablePin, LOW ^ motor->_enableInverted);
	}
}

void astepper_enableOutputs(accelstepper_t* motor)
{
    if (! motor->_interface) return;

	gpio_reset_pin(motor->_GPIOxPin[0]);
    gpio_set_direction(motor->_GPIOxPin[0], GPIO_MODE_OUTPUT);

    gpio_reset_pin(motor->_GPIOxPin[1]);
    gpio_set_direction(motor->_GPIOxPin[1], GPIO_MODE_OUTPUT);

    if (motor->_interface == FULL4WIRE || motor->_interface == HALF4WIRE)
    {
    	gpio_reset_pin(motor->_GPIOxPin[2]);
		gpio_set_direction(motor->_GPIOxPin[2], GPIO_MODE_OUTPUT);

		gpio_reset_pin(motor->_GPIOxPin[3]);
		gpio_set_direction(motor->_GPIOxPin[3], GPIO_MODE_OUTPUT);
	}
    else if (motor->_interface == FULL3WIRE || motor->_interface == HALF3WIRE)
    {
    	gpio_reset_pin(motor->_GPIOxPin[2]);
    	gpio_set_direction(motor->_GPIOxPin[2], GPIO_MODE_OUTPUT);
    }

    if (motor->_GPIOxEnablePin != GPIO_NUM_NC)
    {
        gpio_set_direction(motor->_GPIOxEnablePin, GPIO_MODE_OUTPUT);
        gpio_set_level(motor->_GPIOxEnablePin, HIGH ^ motor->_enableInverted);
    }
}

void astepper_setMinPulseWidth(accelstepper_t* motor, unsigned int minWidth)
{
	motor->_minPulseWidth = minWidth;
}

void astepper_setEnablePin(accelstepper_t* motor, gpio_num_t GPIOxEnablePin)
{
	motor->_GPIOxEnablePin = GPIOxEnablePin;

    // This happens after construction, so init pin now.
    if (motor->_GPIOxEnablePin != GPIO_NUM_NC)
    {
        gpio_set_direction(motor->_GPIOxEnablePin, GPIO_MODE_OUTPUT);
        gpio_set_level(motor->_GPIOxEnablePin, HIGH ^ motor->_enableInverted);
	}
}

void astepper_setPinsInvertedStpDir(accelstepper_t* motor, uint8_t directionInvert, uint8_t stepInvert, uint8_t enableInvert)
{
	motor->_pinInverted[0] = stepInvert; // bool
	motor->_pinInverted[1] = directionInvert; // bool
	motor->_enableInverted = enableInvert; // bool
}

void astepper_setPinsInverted(accelstepper_t* motor, uint8_t pin1Invert, uint8_t pin2Invert, uint8_t pin3Invert, uint8_t pin4Invert, uint8_t enableInvert)
{
	motor->_pinInverted[0] = pin1Invert; // bool
	motor->_pinInverted[1] = pin2Invert; // bool
	motor->_pinInverted[2] = pin3Invert; // bool
	motor->_pinInverted[3] = pin4Invert; // bool
	motor->_enableInverted = enableInvert; // bool
}

// Blocks until the target position is reached and stopped
void astepper_runToPosition(accelstepper_t* motor)
{
    while (astepper_run(motor)) ;
}

uint8_t astepper_runSpeedToPosition(accelstepper_t* motor)
{
    if (motor->_targetPos == motor->_currentPos)
    	return 0; // false
    if (motor->_targetPos > motor->_currentPos)
    	motor->_direction = DIRECTION_CW;
    else
    	motor->_direction = DIRECTION_CCW;

    return astepper_runSpeed(motor);
}

// Blocks until the new target position is reached
void astepper_runToNewPosition(accelstepper_t* motor, long position)
{
    astepper_moveTo(motor, position);
    astepper_runToPosition(motor);
}

void astepper_stop(accelstepper_t* motor)
{
    if (motor->_speed != 0.0)
    {
		long stepsToStop = (long)((motor->_speed * motor->_speed) / (2.0 * motor->_acceleration)) + 1; // Equation 16 (+integer rounding)

		if (motor->_speed > 0)
			astepper_move(motor, stepsToStop);

		else
			astepper_move(motor, -stepsToStop);
    }
}

uint8_t astepper_isRunning(accelstepper_t* motor)
{
    return !(motor->_speed == 0.0 && motor->_targetPos == motor->_currentPos);
}
