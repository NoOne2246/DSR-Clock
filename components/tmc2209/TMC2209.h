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
  tmc_GlobalConfig_t global_config_;
  tmc_DriverCurrent_t driver_current_;
  tmc_CoolConfig_t cool_config_;
  bool cool_step_enabled_;
  tmc_ChopperConfig_t chopper_config_;
  tmc_PwmConfig_t pwm_config_;
} TMC2209_t;

typedef enum
{
  SERIAL_ADDRESS_0 = 0,
  SERIAL_ADDRESS_1 = 1,
  SERIAL_ADDRESS_2 = 2,
  SERIAL_ADDRESS_3 = 3,
} tmc_SerialAddress;

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


extern void tmc_initialize(TMC2209_t *tmc);

extern void tmc_setup(TMC2209_t *TMC, uart_port_t uart_port, long serial_baud_rate, tmc_SerialAddress serial_address, gpio_num_t alternate_rx_pin, gpio_num_t alternate_tx_pin);

// unidirectional methods

// driver must be enabled before use it is disabled by default
extern void tmc_setHardwareEnablePin(TMC2209_t *TMC, gpio_num_t hardware_enable_pin);
extern void tmc_enable(TMC2209_t *TMC);
extern void tmc_disable(TMC2209_t *TMC);

// valid values = 1,2,4,8,...128,256, other values get rounded down
extern void tmc_setMicrostepsPerStep(TMC2209_t *TMC, uint16_t microsteps_per_step);

// valid values = 0-8, microsteps = 2^exponent, 0=1,1=2,2=4,...8=256
// https://en.wikipedia.org/wiki/Power_of_two
extern void tmc_setMicrostepsPerStepPowerOfTwo(TMC2209_t *TMC, uint8_t exponent);

// range 0-100
extern void tmc_setRunCurrent(TMC2209_t *TMC, uint8_t percent);
// range 0-100
extern void tmc_setHoldCurrent(TMC2209_t *TMC, uint8_t percent);
// range 0-100
extern void tmc_setHoldDelay(TMC2209_t *TMC, uint8_t percent);
// range 0-100
extern void tmc_setAllCurrentValues(TMC2209_t *TMC, uint8_t run_current_percent, uint8_t hold_current_percent, uint8_t hold_delay_percent);
extern void tmc_setRMSCurrent(TMC2209_t *TMC, uint16_t mA, float rSense, float holdMultiplier);

extern void tmc_enableDoubleEdge(TMC2209_t *TMC);
extern void tmc_disableDoubleEdge(TMC2209_t *TMC);

extern void tmc_enableVSense(TMC2209_t *TMC);
extern void tmc_disableVSense(TMC2209_t *TMC);

extern void tmc_enableInverseMotorDirection(TMC2209_t *TMC);
extern void tmc_disableInverseMotorDirection(TMC2209_t *TMC);

extern void tmc_setStandstillMode(TMC2209_t *TMC, StandstillMode mode);

extern void tmc_enableAutomaticCurrentScaling(TMC2209_t *TMC);
extern void tmc_disableAutomaticCurrentScaling(TMC2209_t *TMC);
extern void tmc_enableAutomaticGradientAdaptation(TMC2209_t *TMC);
extern void tmc_disableAutomaticGradientAdaptation(TMC2209_t *TMC);
// range 0-255
extern void tmc_setPwmOffset(TMC2209_t *TMC, uint8_t pwm_amplitude);
// range 0-255
extern void tmc_setPwmGradient(TMC2209_t *TMC, uint8_t pwm_amplitude);

// default = 20
// mimimum of 2 for StealthChop auto tuning
extern void tmc_setPowerDownDelay(TMC2209_t *TMC, uint8_t power_down_delay);

// mimimum of 2 when using multiple serial addresses
// in bidirectional communication
extern void tmc_setReplyDelay(TMC2209_t *TMC, uint8_t delay);

extern void tmc_moveAtVelocity(TMC2209_t *TMC, int32_t microsteps_per_period);
extern void tmc_moveUsingStepDirInterface(TMC2209_t *TMC);
extern void tmc_enableStealthChop(TMC2209_t *TMC);
extern void tmc_disableStealthChop(TMC2209_t *TMC);
extern void tmc_setStealthChopDurationThreshold(TMC2209_t *TMC, uint32_t duration_threshold);
extern void tmc_setStallGuardThreshold(TMC2209_t *TMC, uint8_t stall_guard_threshold);

// lower_threshold: min = 1, max = 15
// upper_threshold: min = 0, max = 15, 0-2 recommended
extern void tmc_enableCoolStep(TMC2209_t *TMC, uint8_t lower_threshold, uint8_t upper_threshold);
extern void tmc_disableCoolStep(TMC2209_t *TMC);

extern void tmc_setCoolStepCurrentIncrement(TMC2209_t *TMC, CurrentIncrement current_increment);

extern void tmc_setCoolStepMeasurementCount(TMC2209_t *TMC, MeasurementCount measurement_count);
extern void tmc_setCoolStepDurationThreshold(TMC2209_t *TMC, uint32_t duration_threshold);

extern void tmc_enableAnalogCurrentScaling(TMC2209_t *TMC);
extern void tmc_disableAnalogCurrentScaling(TMC2209_t *TMC);

extern void tmc_useExternalSenseResistors(TMC2209_t *TMC);
extern void tmc_useInternalSenseResistors(TMC2209_t *TMC);

// bidirectional methods
extern uint8_t tmc_getVersion(TMC2209_t *TMC);

// if driver is not communicating, check power and communication connections
extern bool tmc_isCommunicating(TMC2209_t *TMC);

// check to make sure TMC2209 is properly setup and communicating
extern bool tmc_isSetupAndCommunicating(TMC2209_t *TMC);

// driver may be communicating but not setup if driver power is lost then
// restored after setup so that defaults are loaded instead of setup options
extern bool tmc_isCommunicatingButNotSetup(TMC2209_t *TMC);

// driver may also be disabled by the hardware enable input pin
// this pin must be grounded or disconnected before driver may be enabled
extern bool tmc_hardwareDisabled();

extern uint16_t tmc_getMicrostepsPerStep();

extern tmc_Settings_t tmc_getSettings();

extern tmc_Status_t tmc_etStatus(TMC2209_t *TMC);

extern tmc_GlobalStatus_t tmc_getGlobalStatus();

extern void tmc_clearReset();
extern void tmc_clearDriveError();

extern uint8_t tmc_getInterfaceTransmissionCounter();

extern uint32_t tmc_getInterstepDuration();

extern uint16_t tmc_getStallGuardResult();

extern uint8_t tmc_getPwmScaleSum();
extern int16_t tmc_getPwmScaleAuto();
extern uint8_t tmc_getPwmOffsetAuto();
extern uint8_t tmc_getPwmGradientAuto();

extern uint16_t tmc_getMicrostepCounter();

#endif
