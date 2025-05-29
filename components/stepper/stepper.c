#include "stepper.h"
#include "AccelStepper.h"
#include "TMC2209.h"
#include <math.h>

void steppers_initialize(stepper_controller_t *controller)
{
    controller->_num_steppers = 0;
}

uint8_t steppers_addStepper(stepper_controller_t *controller, gpio_num_t stepPin, gpio_num_t dirPin, uart_port_t uartPort, gpio_num_t rxPin, gpio_num_t txPin, tmc_SerialAddress Address)
{
    if (controller->_num_steppers >= STEPPER_LIMIT)
        return -1; // No room for more
    controller->_num_steppers++;
    accelstepper_t *stepper= malloc(sizeof(accelstepper_t));
    astepper_initialize(stepper, 8, stepPin, dirPin, GPIO_NUM_NC, GPIO_NUM_NC, false);
    controller->_steppers[controller->_num_steppers] = stepper;
    TMC2209_t *driver= malloc(sizeof(TMC2209_t));
    tmc_initialize(driver);
    tmc_setup(driver, uartPort, 115200, Address, rxPin, txPin);
    controller->_drivers[controller->_num_steppers] = driver;
    controller->_looping_limit[controller->_num_steppers] = -1;
    tmc_moveUsingStepDirInterface(controller->_drivers[controller->_num_steppers]);
    return controller->_num_steppers;
}

uint8_t steppers_setupRevolution(stepper_controller_t *controller, uint8_t stepperIndex, long loopingLimit)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    controller->_looping_limit[stepperIndex] = loopingLimit;
    return 1;
}

uint8_t steppers_setCurrentPercentages(stepper_controller_t *controller, uint8_t stepperIndex, uint8_t runCurrentPercent, uint8_t holdCurrentPercent, uint8_t holdDelayPercent)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    tmc_setAllCurrentValues(controller->_drivers[stepperIndex], runCurrentPercent, holdCurrentPercent, holdDelayPercent);
    return 1;
}

uint8_t steppers_setRMSCurrent(stepper_controller_t *controller, uint8_t stepperIndex, float rmsCurrent, float rSense, float holdMultiplier)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    tmc_setRMSCurrent(controller->_drivers[stepperIndex], rmsCurrent, rSense, holdMultiplier);
    return 1;
}

uint8_t steppers_setMaxSpeed(stepper_controller_t *controller, uint8_t stepperIndex, float speed)
{

    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    astepper_setMaxSpeed(controller->_steppers[stepperIndex], speed);
    return 1;
}

uint8_t steppers_enableStepper(stepper_controller_t *controller, uint8_t stepper)
{
    if (stepper >= controller->_num_steppers)
        return 0; // Stepper does not exist
    tmc_enable(controller->_drivers[stepper]);
    return 1;
}

uint8_t steppers_disableStepper(stepper_controller_t *controller, uint8_t stepper)
{
    if (stepper >= controller->_num_steppers)
        return 0; // Stepper does not exist
    tmc_disable(controller->_drivers[stepper]);
    return 1;
}

uint8_t steppers_enableAll(stepper_controller_t *controller)
{
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        tmc_enable(controller->_drivers[i]);
    }
    return 1;
}

uint8_t steppers_disableAll(stepper_controller_t *controller)
{
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        tmc_disable(controller->_drivers[i]);
    }
    return 1;
}

uint8_t steppers_enableStealth(stepper_controller_t *controller, uint8_t stepperIndex)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    tmc_enableStealthChop(controller->_drivers[stepperIndex]);
    tmc_enableAutomaticCurrentScaling(controller->_drivers[stepperIndex]);
    return 1;
}

uint8_t steppers_disableStealth(stepper_controller_t *controller, uint8_t stepperIndex)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    tmc_disableStealthChop(controller->_drivers[stepperIndex]);
    tmc_disableAutomaticCurrentScaling(controller->_drivers[stepperIndex]);
    return 1;
}
static uint16_t calculateMicrostep(uint16_t microsteps)
{
    microsteps = ((microsteps) < (1U) ? (1U) : ((microsteps) > (256U) ? (256U) : (microsteps))) >> 1;
    uint16_t exponent = 0;
    while (microsteps > 0)
    {
        microsteps = microsteps >> 1;
        ++exponent;
    }
    return 1 << exponent;
}

uint8_t steppers_setMicrostepAll(stepper_controller_t *controller, uint16_t microsteps[])
{
    double pos = 0;
    if (steppers_run(controller))
    {
        return 0;
    }
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        pos = controller->_last_full_step[i] + controller->_last_micro_step[i] / (double)256 + astepper_currentPosition(controller->_steppers[i]) / (double)tmc_getMicrostepsPerStep(controller->_drivers[i]);
        controller->_last_full_step[i] = trunc(pos);
        controller->_last_micro_step[i] = (uint8_t)((pos - trunc(pos)) * 256);
        astepper_setCurrentPosition(controller->_steppers[i], 0);
        tmc_setMicrostepsPerStep(controller->_drivers[i], calculateMicrostep(microsteps[i]));
    }
    return 1;
}

uint16_t steppers_setMicrostep(stepper_controller_t *controller, uint8_t stepperIndex, uint16_t microsteps)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    uint16_t futureSteps = calculateMicrostep(microsteps);
    uint16_t currSteps = tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]);
    long current = astepper_currentPosition(controller->_steppers[stepperIndex]);
    long target = astepper_targetPosition(controller->_steppers[stepperIndex]);
    if (currSteps < futureSteps || target == current)
    {
        double pos = controller->_last_full_step[stepperIndex] + controller->_last_micro_step[stepperIndex] / (double)256 + current / (double)currSteps;
        controller->_last_full_step[stepperIndex] = trunc(pos);
        controller->_last_micro_step[stepperIndex] = (uint8_t)((pos - trunc(pos)) * 256);
        astepper_setCurrentPosition(controller->_steppers[stepperIndex], 0);
        astepper_move(controller->_steppers[stepperIndex], (target - current) * futureSteps / currSteps);
    }
    else
    {
        double remaining = ((double)target - current) / (futureSteps / currSteps);
        long offset = (long)(remaining - trunc(remaining)) * (futureSteps / currSteps);
        while (astepper_currentPosition(controller->_steppers[stepperIndex]) != offset + current)
        {
            astepper_run(controller->_steppers[stepperIndex]);
        }
        double pos = controller->_last_full_step[stepperIndex] + controller->_last_micro_step[stepperIndex] / (double)256 + current / (double)currSteps;
        controller->_last_full_step[stepperIndex] = trunc(pos);
        controller->_last_micro_step[stepperIndex] = (uint8_t)((pos - trunc(pos)) * 256);
        astepper_setCurrentPosition(controller->_steppers[stepperIndex], 0);
        astepper_move(controller->_steppers[stepperIndex], trunc(remaining));
    }
    tmc_setMicrostepsPerStep(controller->_drivers[stepperIndex], microsteps);
    astepper_computeNewSpeed(controller->_steppers[stepperIndex]);
    return futureSteps;
}

double steppers_currentPosition(stepper_controller_t *controller, uint8_t stepperIndex)
{
    double ret = controller->_last_full_step[stepperIndex];
    ret += controller->_last_micro_step[stepperIndex] / (double)256;
    ret += astepper_currentPosition(controller->_steppers[stepperIndex]) / (double)tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]);
    if (controller->_looping_limit[stepperIndex] > 0)
    {
        ret -= trunc(ret / controller->_looping_limit[stepperIndex]) * controller->_looping_limit[stepperIndex];
    }
    return ret;
}
double steppers_targetPosition(stepper_controller_t *controller, uint8_t stepperIndex)
{
    double ret = controller->_last_full_step[stepperIndex];
    ret += controller->_last_micro_step[stepperIndex] / (double)256;
    ret += astepper_targetPosition(controller->_steppers[stepperIndex]) / (double)tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]);
    if (controller->_looping_limit[stepperIndex] > 0)
    {
        ret -= trunc(ret / controller->_looping_limit[stepperIndex]) * controller->_looping_limit[stepperIndex];
    }
    return ret;
}

double steppers_currentPositionPercent(stepper_controller_t *controller, uint8_t stepperIndex)
{
    double ret = steppers_currentPosition(controller, stepperIndex);
    return ret / controller->_looping_limit[stepperIndex];
}
double steppers_targetPositionPercent(stepper_controller_t *controller, uint8_t stepperIndex)
{
    double ret = steppers_targetPosition(controller, stepperIndex);
    return (ret / controller->_looping_limit[stepperIndex]);
}

void steppers_moveTo(stepper_controller_t *controller, double absolute[])
{
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        steppers_moveToStepper(controller, i, absolute[i]);
    }
}
long steppers_moveToStepper(stepper_controller_t *controller, uint8_t stepperIndex, double absolute)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    double current = (controller->_last_full_step[stepperIndex] + controller->_last_micro_step[stepperIndex] / (double)256);
    current += astepper_currentPosition(controller->_steppers[stepperIndex]) / (double)tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]);

    double relative = 0;
    if (controller->_looping_limit[stepperIndex] > 0)
    {
        current -= trunc(current / controller->_looping_limit[stepperIndex]) * controller->_looping_limit[stepperIndex];
        absolute -= trunc(absolute / controller->_looping_limit[stepperIndex]) * controller->_looping_limit[stepperIndex];
        relative = absolute - current;
        double d2 = -((controller->_looping_limit[stepperIndex] - absolute) + current);
        double d3 = controller->_looping_limit[stepperIndex] - current + absolute;
        if (fabs(relative) > fabs(d2))
        {
            relative = d2;
        }
        if (fabs(relative) > fabs(d3))
        {
            relative = d3;
        }
    }
    else
    {
        relative = absolute - current;
    }
    long distance = (long)round(relative * tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]));
    astepper_move(controller->_steppers[stepperIndex], distance);

    return distance;
}

void steppers_move(stepper_controller_t *controller, double relative[])
{
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        steppers_moveStepper(controller, i, relative[i]);
    }
}

long steppers_moveStepper(stepper_controller_t *controller, uint8_t stepperIndex, double relative)
{
    long distance = (long)round(relative * tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]));
    astepper_move(controller->_steppers[stepperIndex], distance);
    return distance;
}

void steppers_moveToSimultaneous(stepper_controller_t *controller, double absolute[])
{
    float longestTime = 0.0;
    long distances[controller->_num_steppers];

    for (int i = 0; i < controller->_num_steppers; i++)
    {
        distances[i] = steppers_moveToStepper(controller, i, absolute[i]);
        float thisTime = abs(distances[i]) / astepper_maxSpeed(controller->_steppers[i]);
        if (thisTime > longestTime)
        {
            controller->_controlling = i;
            longestTime = thisTime;
        }
    }
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        controller->_fraction[i] = distances[i] / distances[controller->_controlling];
    }
}

void steppers_moveSimultaneous(stepper_controller_t *controller, double relative[])
{
    float longestTime = 0.0;
    long distances[controller->_num_steppers];

    for (int i = 0; i < controller->_num_steppers; i++)
    {
        distances[i] = steppers_moveStepper(controller, i, relative[i]);
        float thisTime = abs(distances[i]) / astepper_maxSpeed(controller->_steppers[i]);
        if (thisTime > longestTime)
        {
            controller->_controlling = i;
            longestTime = thisTime;
        }
    }
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        controller->_fraction[i] = distances[i] / distances[controller->_controlling];
    }
}

void steppers_moveToPercent(stepper_controller_t *controller, double percent[])
{
    double absolute = 0;
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        absolute = controller->_looping_limit[i] * percent[i];
        steppers_moveToStepper(controller, i, absolute);
    }
}

long steppers_moveToStepperPercent(stepper_controller_t *controller, uint8_t stepperIndex, double percent)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    double absolute = controller->_looping_limit[stepperIndex] * percent;
    return steppers_moveToStepper(controller, stepperIndex, absolute);
}

uint8_t steppers_setSpeedSPS(stepper_controller_t *controller, uint8_t stepperIndex, double speed)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    astepper_setSpeed(controller->_steppers[stepperIndex], speed * tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]));
    return 1;
}

uint8_t steppers_setSpeedRPM(stepper_controller_t *controller, uint8_t stepperIndex, double revolutions)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    double steps = revolutions * controller->_looping_limit[stepperIndex] / 60;
    astepper_setSpeed(controller->_steppers[stepperIndex], steps * tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]));
    return 1;
}
uint8_t steppers_setSpeedPeriod(stepper_controller_t *controller, uint8_t stepperIndex, double seconds)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    double speed = controller->_looping_limit[stepperIndex] / seconds;
    astepper_setSpeed(controller->_steppers[stepperIndex], speed * tmc_getMicrostepsPerStep(controller->_drivers[stepperIndex]));
    return 1;
}

uint8_t steppers_run(stepper_controller_t *controller)
{
    uint8_t ret = 0;
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        ret |= astepper_run(controller->_steppers[i]) << i;
    }
    return ret;
}

uint8_t steppers_runSpeed(stepper_controller_t *controller)
{
    uint8_t ret = 0;
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        astepper_runSpeed(controller->_steppers[i]);
        ret |= (astepper_distanceToGo(controller->_steppers[i]) != 0) << i;
    }
    return ret;
}

uint8_t steppers_runSimultaneous(stepper_controller_t *controller)
{
    if (controller->_controlling < 0)
        return 0;
    uint8_t ret = 0;
    ret |= astepper_run(controller->_steppers[controller->_controlling]) << controller->_controlling;
    float speed = astepper_speed(controller->_steppers[controller->_controlling]);
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        if (i == controller->_controlling)
        {
            continue;
        }
        if (controller->_last_speed - speed > 0.5) // when the main motors starts to decelerate, mov other motors to their final position.
        {
            ret |= astepper_run(controller->_steppers[i]);
        }
        else
        {
            astepper_setSpeed(controller->_steppers[i], speed * controller->_fraction[i]);
            astepper_runSpeed(controller->_steppers[i]);
            ret |= ((astepper_distanceToGo(controller->_steppers[i])) != 0) << i;
        }
    }
    controller->_last_speed = speed;
    if (ret == 0)
        controller->_controlling = -1;
    return ret;
}

uint8_t steppers_runStepper(stepper_controller_t *controller, uint8_t stepperIndex)
{
    return astepper_run(controller->_steppers[stepperIndex]);
}

uint8_t steppers_runSpeedStepper(stepper_controller_t *controller, uint8_t stepperIndex)
{
    astepper_runSpeed(controller->_steppers[stepperIndex]);
    return (astepper_distanceToGo(controller->_steppers[stepperIndex]) != 0);
}

void steppers_stopStepper(stepper_controller_t *controller, uint8_t stepperIndex)
{
    astepper_stop(controller->_steppers[stepperIndex]);
}

void steppers_stopAll(stepper_controller_t *controller)
{
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        astepper_stop(controller->_steppers[i]);
    }
}

uint8_t steppers_setPosition(stepper_controller_t *controller, double position[])
{
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        if (astepper_targetPosition(controller->_steppers[i]) != astepper_currentPosition(controller->_steppers[i]))
        {
            return 0;
        }
    }
    for (int i = 0; i < controller->_num_steppers; i++)
    {
        astepper_setCurrentPosition(controller->_steppers[i], 0);
        controller->_last_full_step[i] = (long)trunc(position[i]);
        controller->_last_micro_step[i] = (long)round((position[i] - trunc(position[i])) * 256);
    }
    return 1;
}
uint8_t steppers_setStepperPosition(stepper_controller_t *controller, uint8_t stepperIndex, double position)
{
    if (stepperIndex >= controller->_num_steppers)
    {
        return 0;
    }
    if (astepper_targetPosition(controller->_steppers[stepperIndex]) != astepper_currentPosition(controller->_steppers[stepperIndex]))
    {
        return 0;
    }
    astepper_setCurrentPosition(controller->_steppers[stepperIndex], 0);
    controller->_last_full_step[stepperIndex] = (long)trunc(position);
    controller->_last_micro_step[stepperIndex] = (long)round((position - trunc(position)) * 256);
    return 1;
}