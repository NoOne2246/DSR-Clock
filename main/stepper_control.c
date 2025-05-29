#include "stepper_control.h"
#include "gpio_definitions.h"
#include "stepper_config.h"
#include "freertos/FreeRTOS.h"
#include <time.h>
#include "math.h"

#include "stepper.h"

#define HOUR_STEPPER 0
#define MIN_STEPPER 1
// Define stepper motor pins and AccelStepper object

stepper_controller_t *controller;
clock_Mode_t ClockMode;

TaskHandle_t runTime = NULL;
TaskHandle_t HourHome = NULL;
TaskHandle_t MinHome = NULL;

static void setCurrentStepMode(void);
static uint8_t homeStepper(uint8_t stepperIndex, gpio_num_t senssorPin, long Loop);
static void regular_Move(void);
static uint8_t correctTime(void);
static void homeHour(void *pvParameter);
static void homeMin(void *pvParameter);
static double percent(double val);
static double withinRange(double val, double low, double high);

// Function declarations
void stepper_init(void)
{
    // Initialize the stepper controller
    steppers_initialize(controller);
    steppers_addStepper(controller, MOTOR_HOUR_STEP, MOTOR_HOUR_DIR, UART_NUM_0, MOTOR_HOUR_RX, MOTOR_HOUR_TX, 0);
    steppers_addStepper(controller, MOTOR_MIN_STEP, MOTOR_MIN_DIR, UART_NUM_0, MOTOR_MIN_RX, MOTOR_MIN_TX, 1);
    steppers_setupRevolution(controller, HOUR_STEPPER, 43 * 12 * 200);
    steppers_setupRevolution(controller, MIN_STEPPER, 20 * 12 * 200);
    steppers_setRMSCurrent(controller, HOUR_STEPPER, SANYO_RMS_CURRENT, SANYO_RSENSE, SANYO_HOLD_MULTIPLIER);
    steppers_setRMSCurrent(controller, MIN_STEPPER, LIN_RMS_CURRENT, LIN_RSENSE, LIN_HOLD_MULTIPLIER);
    steppers_setMaxSpeed(controller, HOUR_STEPPER, SANYO_MAX_SPEED);
    steppers_setMaxSpeed(controller, MIN_STEPPER, LIN_MAX_SPEED);
    steppers_enableAll(controller);
    ClockMode = REGULAR_CLOCK;
}

void setCurrentStepMode(void)
{
    uint16_t microsteps[2];
    switch (ClockMode)
    {
    case ADJUST_CLOCK:
        microsteps[0] = SANYO_STEP_ADJUST;
        microsteps[1] = LIN_STEP_ADJUST;
        steppers_setCurrentPercentages(controller, HOUR_STEPPER, SANYO_IRUN_FAST, SANYO_IHOLD_FAST, SANYO_IHOLD_DELAY_FAST);
        steppers_setCurrentPercentages(controller, MIN_STEPPER, LIN_IRUN_FAST, LIN_IHOLD_FAST, LIN_IHOLD_DELAY_FAST);
        break;
    case REWIND_CLOCK:
    case FORWARD_CLOCK:
        microsteps[0] = SANYO_STEP_MAX;
        microsteps[1] = LIN_STEP_MAX;
        steppers_setCurrentPercentages(controller, HOUR_STEPPER, SANYO_IRUN_ULTRA, SANYO_IHOLD_ULTRA, SANYO_IHOLD_DELAY_ULTRA);
        steppers_setCurrentPercentages(controller, MIN_STEPPER, LIN_IRUN_ULTRA, LIN_IHOLD_ULTRA, LIN_IHOLD_DELAY_ULTRA);
        break;
    case HOMING_CLOCK:
        return;
    case REGULAR_CLOCK:
    default:
        microsteps[0] = SANYO_STEP_REGULAR;
        microsteps[1] = LIN_STEP_REGULAR;
        steppers_setCurrentPercentages(controller, HOUR_STEPPER, SANYO_IRUN_SLOW, SANYO_IHOLD_SLOW, SANYO_IHOLD_DELAY_SLOW);
        steppers_setCurrentPercentages(controller, MIN_STEPPER, LIN_IRUN_SLOW, LIN_IHOLD_SLOW, LIN_IHOLD_DELAY_SLOW);
        break;
    }
    steppers_setMicrostepAll(controller, microsteps);
    return;
}

uint8_t runSteppers(void)
{
    uint8_t ret = 0;
    switch (ClockMode)
    {
    case ADJUST_CLOCK:
        ret = steppers_runSpeed(controller);
        if (ret == 0)
        {
            correctTime();
        }
        return ret;
    case REWIND_CLOCK:
    case FORWARD_CLOCK:
        ret = steppers_runSimultaneous(controller);
        if (ret == 0)
        {
            correctTime();
        }
        return ret;
    case HOMING_CLOCK:
        return 0;
    case REGULAR_CLOCK:
    default:
        return steppers_runSpeed(controller);
    }
}

void homeSteppers(void)
{
    ClockMode = HOMING_CLOCK;
    setCurrentStepMode();
    if (runTime != NULL)
    {
        vTaskSuspend(runTime);
    }

    xTaskCreate(homeHour, "Home Hour", 4 * 1024, NULL, 1, &HourHome);
    xTaskCreate(homeMin, "Home Minute", 4 * 1024, NULL, 1, &MinHome);

    while (HourHome != NULL && MinHome != NULL)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    double pos[2] = {0, 0};
    steppers_setPosition(controller, pos);
    if (runTime != NULL)
    {
        vTaskResume(runTime);
    }
    regular_Move();
}

void homeHour(void *pvParameter)
{
    homeStepper(HOUR_STEPPER, SENSOR_HOUR, 43 * 12 * 200);
    vTaskDelete(NULL);
}

void homeMin(void *pvParameter)
{
    homeStepper(MIN_STEPPER, SENSOR_MINUTE, 20 * 12 * 200);
    vTaskDelete(NULL);
}

uint8_t homeStepper(uint8_t stepperIndex, gpio_num_t sensorPin, long Loop)
{

    bool enteredLowRange = false;
    long lowRangeStart = 0;
    if (debouncePin(sensorPin, 0, 1)) // Sensor reads low
    {
        steppers_moveStepper(controller, stepperIndex, -Loop * 0.1); // Start moving toward the home position
        while (steppers_runStepper(controller, stepperIndex))
        {
            if (debouncePin(sensorPin, 1, 1)) // Sensor reads high
            {
                steppers_stop(controller, stepperIndex);
                while (steppers_runStepper(controller, stepperIndex))
                {
                    vTaskDelay(0.5 / portTICK_PERIOD_MS);
                }
            }
            vTaskDelay(0.5 / portTICK_PERIOD_MS);
        }
    }
    steppers_moveStepper(controller, stepperIndex, Loop * 1.1); // Start moving toward the home position

    while (steppers_runStepper(controller, stepperIndex))
    {

        if (debouncePin(sensorPin,0,1)) // Sensor reads low
        {
            if (!enteredLowRange)
            {
                enteredLowRange = true;                                             // Mark that we've entered the low range
                lowRangeStart = steppers_currentPosition(controller, stepperIndex); // Store position while in low range
            }
            continue;
        }
        else if (enteredLowRange) // Sensor reads high again, meaning we exited the low range
        {
            vTaskDelay(1 / portTICK_PERIOD_MS); // Debounce
            if (gpio_get_level(sensorPin) == 1)
            {
                break; // Stop moving
            }
        }
        vTaskDelay(0.5 / portTICK_PERIOD_MS); // Small delay to allow movement timing
    }
    if (!enteredLowRange)
    {
        return 0;
    }

    // Move back to the estimated center of the low range
    long homePosition = (lowRangeStart + steppers_currentPosition(controller, stepperIndex)) / 2;
    steppers_moveToStepper(controller, stepperIndex, homePosition);

    while (steppers_runStepper(controller, stepperIndex))
    {
        vTaskDelay(0.5 / portTICK_PERIOD_MS);
    }
    return 1;
}

bool debouncePin(gpio_num_t pin, uint8_t val, float period)
{
    if (gpio_get_level(pin) != val)
    {
        return false;
    }
    vTaskDelay(period / portTICK_PERIOD_MS);
    return gpio_get_level(pin) == val;
}
void regular_Move(void)
{
    ClockMode = REGULAR_CLOCK;
    setCurrentStepMode();
    steppers_setSpeedPeriod(controller, HOUR_STEPPER, 12 * 60 * 60);
    steppers_setSpeedPeriod(controller, MIN_STEPPER, 12 * 60 * 60);
    return;
}

uint8_t rewindTime(void)
{
    if (ClockMode != REGULAR_CLOCK)
    {
        return 0;
    }
    ClockMode = REWIND_CLOCK;
    setCurrentStepMode();
    double pos[2];
    pos[0] = -43 * 12 * 200;
    pos[1] = -20 * 12 * 200;
    steppers_moveSimultaneous(controller, pos);
    return 1;
}

uint8_t forwardTime(void)
{
    if (ClockMode != REGULAR_CLOCK)
    {
        return 0;
    }
    ClockMode = FORWARD_CLOCK;
    setCurrentStepMode();
    double pos[2];
    pos[0] = 43 * 12 * 200;
    pos[1] = 20 * 12 * 200;
    steppers_moveSimultaneous(controller, pos);
    return 1;
}

uint8_t correctTime(void)
{
    if (ClockMode == REGULAR_CLOCK)
    {
        ClockMode = ADJUST_CLOCK;
        setCurrentStepMode();
    }
    if (ClockMode != ADJUST_CLOCK)
    {
        return 0;
    }
    time_t now;
    struct tm realtime;
    time(&now);
    localtime_r(&now, &realtime);
    double currPos = percent(realtime.tm_hour / (double)12 + realtime.tm_min / (double)(60 * 12) + realtime.tm_sec / (double)(60 * 60 * 12));
    double futPos = percent(currPos + 10.0 / (60 * 60 * 12));
    double hourPos = steppers_currentPositionPercent(controller, HOUR_STEPPER);
    double minPos = steppers_currentPositionPercent(controller, MIN_STEPPER);
    uint8_t reached = 1;
    if (!withinRange(hourPos, currPos, futPos) || !withinRange(minPos, currPos, hourPos))
    {
        steppers_moveToStepperPercent(controller, HOUR_STEPPER, futPos);
        steppers_moveToStepperPercent(controller, MIN_STEPPER, futPos);
        reached = 0;
    }
    if (reached)
    {
        regular_Move();
        double off = (hourPos > currPos ? hourPos - currPos : hourPos + 1.0 - currPos) * 12 * 60 * 60;
        off = off > 1.0 ? trunc(off) : 0;
        vTaskDelay(off * 1000 / portTICK_PERIOD_MS);
    }
    return 1;
}

double percent(double val)
{
    return val - trunc(val);
}

double withinRange(double val, double low, double high)
{
    val = percent(val);
    low = percent(low);
    high = percent(high);
    if (high > low)
    {
        return val > low && val < high;
    }
    return val < high && val > low;
}