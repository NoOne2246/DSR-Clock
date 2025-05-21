/**
 ******************************************************************************
 * @file    TMC2209.h
 * @author  Christopher Ho
 * @version V1.0.0
 * @date    21-05-2025
 * @brief   This is a modified version of TMC2209 Arduino library ported to
 *          esp32 microcontrollers.
 *          The original version of this library can be obtained at:
 *          <https://github.com/janelia-arduino/TMC2209/>. The original
 *          version is Copyright (C) 2009-2018 Peter Polidoro.
 ******************************************************************************
 **/

#ifndef TMC2209_H
#define TMC2209_H

#include "TMC2209_Struct.h"
#include "TMC2209_config.h"

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "driver/uart.h"
#include "driver/gpio.h"

typedef struct
{
  uart_port_t hardware_serial_port_;
  uint32_t serial_baud_rate_;
  uint8_t serial_address_;
  gpio_num_t hardware_enable_pin_;
  uint8_t toff_;
  GlobalConfig global_config_;
  DriverCurrent driver_current_;
  CoolConfig cool_config_;
  bool cool_step_enabled_;
  ChopperConfig chopper_config_;
  PwmConfig pwm_config_;
} TMC2209_t;

typedef enum
{
  SERIAL_ADDRESS_0 = 0,
  SERIAL_ADDRESS_1 = 1,
  SERIAL_ADDRESS_2 = 2,
  SERIAL_ADDRESS_3 = 3,
} SerialAddress;

typedef enum
{
  NORMAL = 0,
  FREEWHEELING = 1,
  STRONG_BRAKING = 2,
  BRAKING = 3,
} StandstillMode;

typedef enum
{
  CURRENT_INCREMENT_1 = 0,
  CURRENT_INCREMENT_2 = 1,
  CURRENT_INCREMENT_4 = 2,
  CURRENT_INCREMENT_8 = 3,
} CurrentIncrement;

typedef enum
{
  MEASUREMENT_COUNT_32 = 0,
  MEASUREMENT_COUNT_8 = 1,
  MEASUREMENT_COUNT_2 = 2,
  MEASUREMENT_COUNT_1 = 3,
} MeasurementCount;

extern void setup(TMC2209_t *TMC, uart_port_t serial, long serial_baud_rate, SerialAddress serial_address, gpio_num_t alternate_rx_pin, gpio_num_t alternate_tx_pin);

// unidirectional methods

// driver must be enabled before use it is disabled by default
extern void setHardwareEnablePin(TMC2209_t *TMC, gpio_num_t hardware_enable_pin);
extern void enable(TMC2209_t *TMC);
extern void disable(TMC2209_t *TMC);

// valid values = 1,2,4,8,...128,256, other values get rounded down
extern void setMicrostepsPerStep(TMC2209_t *TMC, uint16_t microsteps_per_step);

// valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
// https://en.wikipedia.org/wiki/Power_of_two
extern void setMicrostepsPerStepPowerOfTwo(TMC2209_t *TMC, uint8_t exponent);

// range 0-100
extern void setRunCurrent(TMC2209_t *TMC, uint8_t percent);
// range 0-100
extern void setHoldCurrent(TMC2209_t *TMC, uint8_t percent);
// range 0-100
extern void setHoldDelay(TMC2209_t *TMC, uint8_t percent);
// range 0-100
extern void setAllCurrentValues(TMC2209_t *TMC, uint8_t run_current_percent, uint8_t hold_current_percent, uint8_t hold_delay_percent);
extern void setRMSCurrent(TMC2209_t *TMC, uint16_t mA, float rSense, float holdMultiplier);

extern void enableDoubleEdge(TMC2209_t *TMC);
extern void disableDoubleEdge(TMC2209_t *TMC);

extern void enableVSense(TMC2209_t *TMC);
extern void disableVSense(TMC2209_t *TMC);

extern void enableInverseMotorDirection(TMC2209_t *TMC);
extern void disableInverseMotorDirection(TMC2209_t *TMC);

extern void setStandstillMode(TMC2209_t *TMC, StandstillMode mode);

extern void enableAutomaticCurrentScaling(TMC2209_t *TMC);
extern void disableAutomaticCurrentScaling(TMC2209_t *TMC);
extern void enableAutomaticGradientAdaptation(TMC2209_t *TMC);
extern void disableAutomaticGradientAdaptation(TMC2209_t *TMC);
// range 0-255
extern void setPwmOffset(TMC2209_t *TMC, uint8_t pwm_amplitude);
// range 0-255
extern void setPwmGradient(TMC2209_t *TMC, uint8_t pwm_amplitude);

// default = 20
// mimimum of 2 for StealthChop auto tuning
extern void setPowerDownDelay(TMC2209_t *TMC, uint8_t power_down_delay);

// mimimum of 2 when using multiple serial addresses
// in bidirectional communication
const static uint8_t REPLY_DELAY_MAX = 15;
extern void setReplyDelay(TMC2209_t *TMC, uint8_t delay);

extern void moveAtVelocity(TMC2209_t *TMC, int32_t microsteps_per_period);
extern void moveUsingStepDirInterface(TMC2209_t *TMC);

extern void enableStealthChop(TMC2209_t *TMC);
extern void disableStealthChop(TMC2209_t *TMC);

extern void setStealthChopDurationThreshold(TMC2209_t *TMC, uint32_t duration_threshold);

extern void setStallGuardThreshold(TMC2209_t *TMC, uint8_t stall_guard_threshold);

// lower_threshold: min = 1, max = 15
// upper_threshold: min = 0, max = 15, 0-2 recommended
extern void enableCoolStep(TMC2209_t *TMC, uint8_t lower_threshold, uint8_t upper_threshold);
extern void disableCoolStep(TMC2209_t *TMC);

extern void setCoolStepCurrentIncrement(TMC2209_t *TMC, CurrentIncrement current_increment);

extern void setCoolStepMeasurementCount(TMC2209_t *TMC, MeasurementCount measurement_count);
extern void setCoolStepDurationThreshold(TMC2209_t *TMC, uint32_t duration_threshold);

extern void enableAnalogCurrentScaling(TMC2209_t *TMC);
extern void disableAnalogCurrentScaling(TMC2209_t *TMC);

extern void useExternalSenseResistors(TMC2209_t *TMC);
extern void useInternalSenseResistors(TMC2209_t *TMC);

// bidirectional methods
extern uint8_t getVersion(TMC2209_t *TMC);

// if driver is not communicating, check power and communication connections
extern bool isCommunicating(TMC2209_t *TMC);

// check to make sure TMC2209 is properly setup and communicating
extern bool isSetupAndCommunicating(TMC2209_t *TMC);

// driver may be communicating but not setup if driver power is lost then
// restored after setup so that defaults are loaded instead of setup options
extern bool isCommunicatingButNotSetup(TMC2209_t *TMC);

// driver may also be disabled by the hardware enable input pin
// this pin must be grounded or disconnected before driver may be enabled
extern bool hardwareDisabled();

extern uint16_t getMicrostepsPerStep();

extern Settings getSettings();

const static uint8_t CURRENT_SCALING_MAX = 31;
extern Status getStatus(TMC2209_t *TMC);

extern GlobalStatus getGlobalStatus();

extern void clearReset();
extern void clearDriveError();

extern uint8_t getInterfaceTransmissionCounter();

extern uint32_t getInterstepDuration();

extern uint16_t getStallGuardResult();

extern uint8_t getPwmScaleSum();
extern int16_t getPwmScaleAuto();
extern uint8_t getPwmOffsetAuto();
extern uint8_t getPwmGradientAuto();

extern uint16_t getMicrostepCounter();

#endif
