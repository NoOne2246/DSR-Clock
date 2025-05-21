/**
 ******************************************************************************
 * @file    MultiStepper.c
 * @author  Christopher Ho
 * @version V1.0.0
 * @date    21-05-2025
 * @brief   This is a modified version of AccelStepper Arduino library ported to
 *          esp32 microcontrollers.
 *          The original version of this library can be obtained at:
 *          <http://www.airspayce.com/mikem/arduino/AccelStepper/>. The original
 *          version is Copyright (C) 2009-2018 Mike McCauley.
 ******************************************************************************
 **/

#include "MultiStepper.h"
#include "AccelStepper.h"

void initMultiStepper(multiStepper_t* multistepper){

	multistepper->_num_steppers = 0;
}

uint8_t addStepperMulti(multiStepper_t* multistepper, accelstepper_t* stepper)
{
	if (multistepper->_num_steppers >= MULTISTEPPER_MAX_STEPPERS)
		return 0; // No room for more
	multistepper->_steppers[multistepper->_num_steppers++] = stepper;
	return 1;
}

void moveToMulti(multiStepper_t* multistepper, long _absolute[])
{
	// First find the stepper that will take the longest time to move
	float longestTime = 0.0;

	uint8_t i;
	for (i = 0; i < multistepper->_num_steppers; i++)
	{
		long thisDistance = _absolute[i] - currentPosition(multistepper->_steppers[i]);
		float thisTime = abs(thisDistance) / maxSpeed(multistepper->_steppers[i]);

		if (thisTime > longestTime)
			longestTime = thisTime;
	}

	if (longestTime > 0.0)
	{
		// Now work out a new max speed for each stepper so they will all
		// arrived at the same time of longestTime
		for (i = 0; i < multistepper->_num_steppers; i++)
		{
			long thisDistance = _absolute[i] - currentPosition(multistepper->_steppers[i]);
			float thisSpeed = thisDistance / longestTime;
			moveTo(multistepper->_steppers[i], _absolute[i]); // New target position (resets speed)
			setSpeed(multistepper->_steppers[i], thisSpeed); // New speed
		}
	}
}

uint8_t runMulti(multiStepper_t* multistepper)
{
	uint8_t i;
	uint8_t ret = 0;
	for (i = 0; i < multistepper -> _num_steppers; i++)
	{
		if (distanceToGo(multistepper->_steppers[i]) != 0)
		{
			runSpeed(multistepper->_steppers[i]);
			ret = 1;
		}
		// Caution: it has een reported that if any motor is used with acceleration outside of
		// MultiStepper, this code is necessary, you get
		// strange results where it moves in the wrong direction for a while then
		// slams back the correct way.
	#if 0
		else
		{
			// Need to call this to clear _stepInterval, _speed and _n 
			otherwise future calls will fail.
			_multistepper->_steppers[i]->setCurrentPosition(_steppers[i]->currentPosition());
		}
	#endif
	}
	return ret;
}



// Blocks until all steppers reach their target position and are stopped
void runSpeedToPositionMulti( multiStepper_t* multistepper){
	while (runMulti(multistepper));
}
