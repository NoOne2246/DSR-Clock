// AccelStepper.cpp
//
// Copyright (C) 2009-2013 Mike McCauley
// $Id: AccelStepper.cpp,v 1.23 2016/08/09 00:39:10 mikem Exp $

#include "AccelStepper.h"
#include "esp_timer.h"
#include <math.h>
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

#if 0
// Some debugging assistance
void dump(uint8_t* p, int l)
{
    int i;

    for (i = 0; i < l; i++)
    {
	printf("%i", p[i], HEX); // ??
	printf(" ");
    }
    printf("\n");
}
#endif

void AccelStepper::moveTo(long absolute)
{
    if (_targetPos != absolute)
    {
        _targetPos = absolute;
        computeNewSpeed();
        // compute new n?
    }
}

void AccelStepper::move(long relative)
{
    moveTo(_currentPos + relative);
}

// Implements steps according to the current step interval
// You must call this at least once per step
// returns true if a step occurred
bool AccelStepper::runSpeed()
{
    // Dont do anything unless we actually have a step interval
    if (!_stepInterval)
        return false;

    int64_t time = esp_timer_get_time();
    if (time - _lastStepTime >= _stepInterval)
    {
        if (_direction == DIRECTION_CW)
        {
            // Clockwise
            _currentPos += 1;
        }
        else
        {
            // Anticlockwise
            _currentPos -= 1;
        }
        step(_currentPos);

        _lastStepTime = time; // Caution: does not account for costs in step()

        return true;
    }
    else
    {
        return false;
    }
}

long AccelStepper::distanceToGo()
{
    return _targetPos - _currentPos;
}

long AccelStepper::targetPosition()
{
    return _targetPos;
}

long AccelStepper::currentPosition()
{
    return _currentPos;
}

// Useful during initialisations or after initial positioning
// Sets speed to 0
void AccelStepper::setCurrentPosition(long position)
{
    _targetPos = _currentPos = position;
    _n = 0;
    _stepInterval = 0;
    _speed = 0.0;
}

void AccelStepper::computeNewSpeed()
{
    long distanceTo = distanceToGo(); // +ve is clockwise from curent location

    long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16

    if (distanceTo == 0 && stepsToStop <= 1)
    {
        // We are at the target and its time to stop
        _stepInterval = 0;
        _speed = 0.0;
        _n = 0;
        return;
    }

    if (distanceTo > 0)
    {
        // We are anticlockwise from the target
        // Need to go clockwise from here, maybe decelerate now
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= distanceTo) || _direction == DIRECTION_CCW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < distanceTo) && _direction == DIRECTION_CW)
                _n = -_n; // Start accceleration
        }
    }
    else if (distanceTo < 0)
    {
        // We are clockwise from the target
        // Need to go anticlockwise from here, maybe decelerate
        if (_n > 0)
        {
            // Currently accelerating, need to decel now? Or maybe going the wrong way?
            if ((stepsToStop >= -distanceTo) || _direction == DIRECTION_CW)
                _n = -stepsToStop; // Start deceleration
        }
        else if (_n < 0)
        {
            // Currently decelerating, need to accel again?
            if ((stepsToStop < -distanceTo) && _direction == DIRECTION_CCW)
                _n = -_n; // Start accceleration
        }
    }

    // Need to accelerate or decelerate
    if (_n == 0)
    {
        // First step from stopped
        _cn = _c0;
        _direction = (distanceTo > 0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    else
    {
        // Subsequent step. Works for accel (n is +_ve) and decel (n is -ve).
        _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1)); // Equation 13
        _cn = max(_cn, _cmin);
    }
    _n++;
    _stepInterval = _cn;
    _speed = 1000000.0 / _cn;
    if (_direction == DIRECTION_CCW)
        _speed = -_speed;

#if 0
    printf("%f\n", _speed);
    printf("%f\n", _acceleration);
    printf("%f\n", _cn);
    printf("%f\n", _c0);
    printf("%ld\n", _n);
    printf("%lu\n", _stepInterval);
    printf("%ld\n", distanceTo);
    printf("%ld\n", stepsToStop);
    printf("-----\n");
#endif
}

// Run the motor to implement speed and acceleration in order to proceed to the target position
// You must call this at least once per step, preferably in your main loop
// If the motor is in the desired position, the cost is very small
// returns true if the motor is still running to the target position.
bool AccelStepper::run()
{
    if (runSpeed())
        computeNewSpeed();
    return _speed != 0.0 || distanceToGo() != 0;
}

AccelStepper::AccelStepper(uint8_t interface, gpio_num_t pin1, gpio_num_t pin2, gpio_num_t pin3, gpio_num_t pin4, bool enable)
{
    _interface = interface;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = (gpio_num_t)0xff;
    _lastStepTime = 0;
    _pin[0] = pin1;
    _pin[1] = pin2;
    _pin[2] = pin3;
    _pin[3] = pin4;
    _enableInverted = false;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
        _pinInverted[i] = 0;
    if (enable)
        enableOutputs();
    // Some reasonable default
    setAcceleration(1);
}

AccelStepper::AccelStepper(void (*forward)(), void (*backward)())
{
    _interface = 0;
    _currentPos = 0;
    _targetPos = 0;
    _speed = 0.0;
    _maxSpeed = 1.0;
    _acceleration = 0.0;
    _sqrt_twoa = 1.0;
    _stepInterval = 0;
    _minPulseWidth = 1;
    _enablePin = (gpio_num_t)0xff;
    _lastStepTime = 0;
    _pin[0] = (gpio_num_t)0;
    _pin[1] = (gpio_num_t)0;
    _pin[2] = (gpio_num_t)0;
    _pin[3] = (gpio_num_t)0;
    _forward = forward;
    _backward = backward;

    // NEW
    _n = 0;
    _c0 = 0.0;
    _cn = 0.0;
    _cmin = 1.0;
    _direction = DIRECTION_CCW;

    int i;
    for (i = 0; i < 4; i++)
        _pinInverted[i] = 0;
    // Some reasonable default
    setAcceleration(1);
}

void AccelStepper::setMaxSpeed(float speed)
{
    if (speed < 0.0)
        speed = -speed;
    if (_maxSpeed != speed)
    {
        _maxSpeed = speed;
        _cmin = 1000000.0 / speed;
        // Recompute _n from current speed and adjust speed if accelerating or cruising
        if (_n > 0)
        {
            _n = (long)((_speed * _speed) / (2.0 * _acceleration)); // Equation 16
            computeNewSpeed();
        }
    }
}

float AccelStepper::maxSpeed()
{
    return _maxSpeed;
}

void AccelStepper::setAcceleration(float acceleration)
{
    if (acceleration == 0.0)
        return;
    if (acceleration < 0.0)
        acceleration = -acceleration;
    if (_acceleration != acceleration)
    {
        // Recompute _n per Equation 17
        _n = _n * (_acceleration / acceleration);
        // New c0 per Equation 7, with correction per Equation 15
        _c0 = 0.676 * sqrt(2.0 / acceleration) * 1000000.0; // Equation 15
        _acceleration = acceleration;
        computeNewSpeed();
    }
}

void AccelStepper::setSpeed(float speed)
{
    if (speed == _speed)
        return;
    speed = constrain(speed, -_maxSpeed, _maxSpeed);
    if (speed == 0.0)
        _stepInterval = 0;
    else
    {
        _stepInterval = fabs(1000000.0 / speed);
        _direction = (speed > 0.0) ? DIRECTION_CW : DIRECTION_CCW;
    }
    _speed = speed;
}

float AccelStepper::speed()
{
    return _speed;
}

// Subclasses can override
void AccelStepper::step(long step)
{
    switch (_interface)
    {
    case FUNCTION:
        step0(step);
        break;

    case DRIVER:
        step1(step);
        break;

    case FULL2WIRE:
        step2(step);
        break;

    case FULL3WIRE:
        step3(step);
        break;

    case FULL4WIRE:
        step4(step);
        break;

    case HALF3WIRE:
        step6(step);
        break;

    case HALF4WIRE:
        step8(step);
        break;
    }
}

// You might want to override this to implement eg serial output
// bit 0 of the mask corresponds to _pin[0]
// bit 1 of the mask corresponds to _pin[1]
// ....
void AccelStepper::setOutputPins(uint8_t mask)
{
    uint8_t numpins = 2;
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
        numpins = 4;
    else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
        numpins = 3;
    uint8_t i;
    for (i = 0; i < numpins; i++)
        gpio_set_level(_pin[i], (mask & (1 << i)) ? (HIGH ^ _pinInverted[i]) : (LOW ^ _pinInverted[i]));
}

// 0 pin step function (ie for functional usage)
void AccelStepper::step0(long step)
{
    (void)(step); // Unused
    if (_speed > 0)
        _forward();
    else
        _backward();
}

// 1 pin step function (ie for stepper drivers)
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step1(long step)
{
    (void)(step); // Unused

    // _pin[0] is step, _pin[1] is direction
    setOutputPins(_direction ? 0b10 : 0b00); // Set direction first else get rogue pulses
    setOutputPins(_direction ? 0b11 : 0b01); // step HIGH
    // Caution 200ns setup time
    // Delay the minimum allowed pulse width
    vTaskDelay(_minPulseWidth / (portTICK_PERIOD_MS * 1000));
    setOutputPins(_direction ? 0b10 : 0b00); // step LOW
}

// 2 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step2(long step)
{
    switch (step & 0x3)
    {
    case 0: /* 01 */
        setOutputPins(0b10);
        break;

    case 1: /* 11 */
        setOutputPins(0b11);
        break;

    case 2: /* 10 */
        setOutputPins(0b01);
        break;

    case 3: /* 00 */
        setOutputPins(0b00);
        break;
    }
}
// 3 pin step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step3(long step)
{
    switch (step % 3)
    {
    case 0: // 100
        setOutputPins(0b100);
        break;

    case 1: // 001
        setOutputPins(0b001);
        break;

    case 2: //010
        setOutputPins(0b010);
        break;
    }
}

// 4 pin step function for half stepper
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step4(long step)
{
    switch (step & 0x3)
    {
    case 0: // 1010
        setOutputPins(0b0101);
        break;

    case 1: // 0110
        setOutputPins(0b0110);
        break;

    case 2: //0101
        setOutputPins(0b1010);
        break;

    case 3: //1001
        setOutputPins(0b1001);
        break;
    }
}

// 3 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step6(long step)
{
    switch (step % 6)
    {
    case 0: // 100
        setOutputPins(0b100);
        break;

    case 1: // 101
        setOutputPins(0b101);
        break;

    case 2: // 001
        setOutputPins(0b001);
        break;

    case 3: // 011
        setOutputPins(0b011);
        break;

    case 4: // 010
        setOutputPins(0b010);
        break;

    case 5: // 011
        setOutputPins(0b110);
        break;
    }
}

// 4 pin half step function
// This is passed the current step number (0 to 7)
// Subclasses can override
void AccelStepper::step8(long step)
{
    switch (step & 0x7)
    {
    case 0: // 1000
        setOutputPins(0b0001);
        break;

    case 1: // 1010
        setOutputPins(0b0101);
        break;

    case 2: // 0010
        setOutputPins(0b0100);
        break;

    case 3: // 0110
        setOutputPins(0b0110);
        break;

    case 4: // 0100
        setOutputPins(0b0010);
        break;

    case 5: //0101
        setOutputPins(0b1010);
        break;

    case 6: // 0001
        setOutputPins(0b1000);
        break;

    case 7: //1001
        setOutputPins(0b1001);
        break;
    }
}

// Prevents power consumption on the outputs
void AccelStepper::disableOutputs()
{
    if (!_interface)
        return;

    setOutputPins(0); // Handles inversion automatically
    if (_enablePin != 0xff)
    {
        esp_rom_gpio_pad_select_gpio(_pin[2]);
        gpio_set_direction(_enablePin, GPIO_MODE_OUTPUT);
        gpio_set_level(_enablePin, LOW ^ _enableInverted);
    }
}

void AccelStepper::enableOutputs()
{
    if (!_interface)
        return;

    esp_rom_gpio_pad_select_gpio(_pin[0]);
    gpio_set_direction(_pin[0], GPIO_MODE_OUTPUT);
    esp_rom_gpio_pad_select_gpio(_pin[1]);
    gpio_set_direction(_pin[1], GPIO_MODE_OUTPUT);
    if (_interface == FULL4WIRE || _interface == HALF4WIRE)
    {
        esp_rom_gpio_pad_select_gpio(_pin[2]);
        gpio_set_direction(_pin[2], GPIO_MODE_OUTPUT);
        esp_rom_gpio_pad_select_gpio(_pin[3]);
        gpio_set_direction(_pin[3], GPIO_MODE_OUTPUT);
    }
    else if (_interface == FULL3WIRE || _interface == HALF3WIRE)
    {
        esp_rom_gpio_pad_select_gpio(_pin[2]);
        gpio_set_direction(_pin[2], GPIO_MODE_OUTPUT);
    }

    if (_enablePin != 0xff)
    {
        esp_rom_gpio_pad_select_gpio(_enablePin);
        gpio_set_direction(_enablePin, GPIO_MODE_OUTPUT);
        gpio_set_level(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepper::setMinPulseWidth(unsigned int minWidth)
{
    _minPulseWidth = minWidth;
}

void AccelStepper::setEnablePin(gpio_num_t enablePin)
{
    _enablePin = enablePin;

    // This happens after construction, so init pin now.
    if (_enablePin != 0xff)
    {
        esp_rom_gpio_pad_select_gpio(_enablePin);
        gpio_set_direction(_enablePin, GPIO_MODE_OUTPUT);
        gpio_set_level(_enablePin, HIGH ^ _enableInverted);
    }
}

void AccelStepper::setPinsInverted(bool directionInvert, bool stepInvert, bool enableInvert)
{
    _pinInverted[0] = stepInvert;
    _pinInverted[1] = directionInvert;
    _enableInverted = enableInvert;
}

void AccelStepper::setPinsInverted(bool pin1Invert, bool pin2Invert, bool pin3Invert, bool pin4Invert, bool enableInvert)
{
    _pinInverted[0] = pin1Invert;
    _pinInverted[1] = pin2Invert;
    _pinInverted[2] = pin3Invert;
    _pinInverted[3] = pin4Invert;
    _enableInverted = enableInvert;
}

// Blocks until the target position is reached and stopped
void AccelStepper::runToPosition()
{
    while (run())
        ;
}

bool AccelStepper::runSpeedToPosition()
{
    if (_targetPos == _currentPos)
        return false;
    if (_targetPos > _currentPos)
        _direction = DIRECTION_CW;
    else
        _direction = DIRECTION_CCW;
    return runSpeed();
}

// Blocks until the new target position is reached
void AccelStepper::runToNewPosition(long position)
{
    moveTo(position);
    runToPosition();
}

void AccelStepper::stop()
{
    if (_speed != 0.0)
    {
        long stepsToStop = (long)((_speed * _speed) / (2.0 * _acceleration)) + 1; // Equation 16 (+integer rounding)
        if (_speed > 0)
            move(stepsToStop);
        else
            move(-stepsToStop);
    }
}

bool AccelStepper::isRunning()
{
    return !(_speed == 0.0 && _targetPos == _currentPos);
}
