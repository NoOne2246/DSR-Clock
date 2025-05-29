/**
 ******************************************************************************
 * @file    TMC2209.c
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

#include "TMC2209.h"
#include "TMC2209_config.h"
#include "TMC2209_Struct.h"

#include "driver/gpio.h"
#include "driver/uart.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "stdint.h"
#include "esp_log.h"
#include <string.h>

typedef enum
{
  WRITE_READ,
  READ_REQUEST
} DatagramType;

typedef struct
{
  DatagramType type;
  union
  {
    tmc_WriteReadReplyDatagram_t writeRead;
    tmc_ReadRequestDatagram_t readRequest;
  } datagram;
} DatagramContainer;


static const char *TAG = "TMC";

static void initialize(TMC2209_t *TMC, tmc_SerialAddress serial_address);
static int serialAvailable(TMC2209_t *TMC);
static size_t serialWrite(TMC2209_t *TMC, uint8_t c);
static int serialRead(TMC2209_t *TMC);
static void serialFlush(TMC2209_t *TMC);

static void setOperationModeToSerial(TMC2209_t *TMC, tmc_SerialAddress serial_address);
static void setRegistersToDefaults(TMC2209_t *TMC);
static void readAndStoreRegisters(TMC2209_t *TMC);
static bool serialOperationMode(TMC2209_t *TMC);
static void minimizeMotorCurrent(TMC2209_t *TMC);

static uint32_t reverseData(uint32_t data);

static uint8_t calculateCrcWrite(tmc_WriteReadReplyDatagram_t datagram, uint8_t datagram_size);
static uint8_t calculateCrcRead(tmc_ReadRequestDatagram_t datagram, uint8_t datagram_size);
static void sendDatagramUnidirectional(TMC2209_t *TMC, tmc_WriteReadReplyDatagram_t datagram, uint8_t datagram_size);
static void sendDatagramBidirectional(TMC2209_t *TMC, tmc_ReadRequestDatagram_t datagram, uint8_t datagram_size);

static void write(TMC2209_t *TMC, uint8_t register_address, uint32_t data);
static uint32_t read(TMC2209_t *TMC, uint8_t register_address);

static uint8_t percentToCurrentSetting(uint8_t percent);
static uint8_t currentSettingToPercent(uint8_t current_setting);
static uint8_t percentToHoldDelaySetting(uint8_t percent);
static uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting);

// static uint8_t pwmAmplitudeToPwmAmpl(uint8_t pwm_amplitude);
// static uint8_t pwmAmplitudeToPwmGrad(uint8_t pwm_amplitude);

static void writeStoredGlobalConfig(TMC2209_t *TMC);
static uint32_t readGlobalConfigBytes(TMC2209_t *TMC);
static void writeStoredDriverCurrent(TMC2209_t *TMC);
static void writeStoredChopperConfig(TMC2209_t *TMC);
static uint32_t readChopperConfigBytes(TMC2209_t *TMC);
static void writeStoredPwmConfig(TMC2209_t *TMC);
static uint32_t readPwmConfigBytes(TMC2209_t *TMC);

static uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high);
static uint8_t map(uint8_t value, uint8_t fromLow, uint8_t fromHigh, uint8_t toLow, uint8_t toHigh);

void tmc_initialize(TMC2209_t *tmc)
{
  tmc->hardware_serial_port_ = UART_NUM_MAX;
  tmc->serial_baud_rate_ = 115200;
  tmc->serial_address_ = SERIAL_ADDRESS_0;
  tmc->hardware_enable_pin_ = GPIO_NUM_NC;
  tmc->cool_step_enabled_ = false;
  tmc->toff_ = TOFF_DEFAULT;
}

void tmc_setup(TMC2209_t *TMC, uart_port_t uart_port, long serial_baud_rate, tmc_SerialAddress serial_address, gpio_num_t alternate_rx_pin, gpio_num_t alternate_tx_pin)
{
  TMC->hardware_serial_port_ = uart_port;
  TMC->serial_baud_rate_ = serial_baud_rate;

  if(uart_is_driver_installed(TMC->hardware_serial_port_)){
    uart_driver_delete(TMC->hardware_serial_port_);
  }
  // Install UART driver
  uart_driver_install(TMC->hardware_serial_port_, 1024, 0, 0, NULL, 0);

  // Configure UART parameters
  uart_config_t uart_config = {
      .baud_rate = serial_baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  uart_param_config(TMC->hardware_serial_port_, &uart_config);

  if ((alternate_rx_pin != GPIO_NUM_NC) || (alternate_tx_pin != GPIO_NUM_NC))
  {
    uart_set_pin(TMC->hardware_serial_port_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  }
  else
  {
    uart_set_pin(TMC->hardware_serial_port_, alternate_tx_pin, alternate_rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  }

  initialize(TMC, serial_address);
}

// unidirectional methods

void tmc_setHardwareEnablePin(TMC2209_t *TMC, gpio_num_t hardware_enable_pin)
{
  TMC->hardware_enable_pin_ = hardware_enable_pin;
  if (TMC->hardware_enable_pin_ != GPIO_NUM_NC)
  {
    gpio_set_direction(TMC->hardware_enable_pin_, GPIO_MODE_OUTPUT);
    gpio_set_level(TMC->hardware_enable_pin_, HIGH);
  }
}

void tmc_enable(TMC2209_t *TMC)
{
  if (TMC->hardware_enable_pin_ != GPIO_NUM_NC)
  {
    gpio_set_level(TMC->hardware_enable_pin_, LOW);
  }
  TMC->chopper_config_.toff = TOFF_DEFAULT;
  writeStoredChopperConfig(TMC);
}

void tmc_disable(TMC2209_t *TMC)
{
  if (TMC->hardware_enable_pin_ != GPIO_NUM_NC)
  {
    gpio_set_level(TMC->hardware_enable_pin_, HIGH);
  }
  TMC->chopper_config_.toff = TOFF_DISABLE;
  writeStoredChopperConfig(TMC);
}

void tmc_setMicrostepsPerStep(TMC2209_t *TMC, uint16_t microsteps_per_step)
{
  uint16_t microsteps_per_step_shifted = constrain_(microsteps_per_step,
                                                    MICROSTEPS_PER_STEP_MIN,
                                                    MICROSTEPS_PER_STEP_MAX);
  microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
  uint16_t exponent = 0;
  while (microsteps_per_step_shifted > 0)
  {
    microsteps_per_step_shifted = microsteps_per_step_shifted >> 1;
    ++exponent;
  }
  tmc_setMicrostepsPerStepPowerOfTwo(TMC, exponent);
}

void tmc_setMicrostepsPerStepPowerOfTwo(TMC2209_t *TMC, uint8_t exponent)
{
  switch (exponent)
  {
  case 0:
  {
    TMC->chopper_config_.mres = MRES_001;
    break;
  }
  case 1:
  {
    TMC->chopper_config_.mres = MRES_002;
    break;
  }
  case 2:
  {
    TMC->chopper_config_.mres = MRES_004;
    break;
  }
  case 3:
  {
    TMC->chopper_config_.mres = MRES_008;
    break;
  }
  case 4:
  {
    TMC->chopper_config_.mres = MRES_016;
    break;
  }
  case 5:
  {
    TMC->chopper_config_.mres = MRES_032;
    break;
  }
  case 6:
  {
    TMC->chopper_config_.mres = MRES_064;
    break;
  }
  case 7:
  {
    TMC->chopper_config_.mres = MRES_128;
    break;
  }
  case 8:
  default:
  {
    TMC->chopper_config_.mres = MRES_256;
    break;
  }
  }
  writeStoredChopperConfig(TMC);
}

void tmc_setRunCurrent(TMC2209_t *TMC, uint8_t percent)
{
  uint8_t run_current = percentToCurrentSetting(percent);
  TMC->driver_current_.irun = run_current;
  writeStoredDriverCurrent(TMC);
}

void tmc_setHoldCurrent(TMC2209_t *TMC, uint8_t percent)
{
  uint8_t hold_current = percentToCurrentSetting(percent);
  TMC->driver_current_.ihold = hold_current;
  writeStoredDriverCurrent(TMC);
}

void tmc_setHoldDelay(TMC2209_t *TMC, uint8_t percent)
{
  uint8_t hold_delay = percentToHoldDelaySetting(percent);
  TMC->driver_current_.iholddelay = hold_delay;
  writeStoredDriverCurrent(TMC);
}

void tmc_setAllCurrentValues(TMC2209_t *TMC, uint8_t run_current_percent,
                         uint8_t hold_current_percent,
                         uint8_t hold_delay_percent)
{
  uint8_t run_current = percentToCurrentSetting(run_current_percent);
  uint8_t hold_current = percentToCurrentSetting(hold_current_percent);
  uint8_t hold_delay = percentToHoldDelaySetting(hold_delay_percent);

  TMC->driver_current_.irun = run_current;
  TMC->driver_current_.ihold = hold_current;
  TMC->driver_current_.iholddelay = hold_delay;
  writeStoredDriverCurrent(TMC);
}

void tmc_setRMSCurrent(TMC2209_t *TMC, uint16_t mA, float rSense, float holdMultiplier)
{
  // Taken from https://github.com/teemuatlut/TMCStepper/blob/74e8e6881adc9241c2e626071e7328d7652f361a/src/source/TMCStepper.cpp#L41.

  uint8_t CS = 32.0 * 1.41421 * mA / 1000.0 * (rSense + 0.02) / 0.325 - 1;
  // If Current Scale is too low, turn on high sensitivity R_sense and calculate again
  if (CS < 16)
  {
    tmc_enableVSense(TMC);
    CS = 32.0 * 1.41421 * mA / 1000.0 * (rSense + 0.02) / 0.180 - 1;
  }
  else
  { // If CS >= 16, turn off high_sense_r
    tmc_disableVSense(TMC);
  }

  if (CS > 31)
  {
    CS = 31;
  }

  TMC->driver_current_.irun = CS;
  TMC->driver_current_.ihold = CS * holdMultiplier;
  writeStoredDriverCurrent(TMC);
}

void tmc_enableDoubleEdge(TMC2209_t *TMC)
{
  TMC->chopper_config_.double_edge = DOUBLE_EDGE_ENABLE;
  writeStoredChopperConfig(TMC);
}

void tmc_disableDoubleEdge(TMC2209_t *TMC)
{
  TMC->chopper_config_.double_edge = DOUBLE_EDGE_DISABLE;
  writeStoredChopperConfig(TMC);
}

void tmc_enableVSense(TMC2209_t *TMC)
{
  TMC->chopper_config_.vsense = VSENSE_ENABLE;
  writeStoredChopperConfig(TMC);
}

void tmc_disableVSense(TMC2209_t *TMC)
{
  TMC->chopper_config_.vsense = VSENSE_DISABLE;
  writeStoredChopperConfig(TMC);
}

void tmc_enableInverseMotorDirection(TMC2209_t *TMC)
{
  TMC->global_config_.shaft = 1;
  writeStoredGlobalConfig(TMC);
}

void tmc_disableInverseMotorDirection(TMC2209_t *TMC)
{
  TMC->global_config_.shaft = 0;
  writeStoredGlobalConfig(TMC);
}

void tmc_setStandstillMode(TMC2209_t *TMC, StandstillMode mode)
{
  TMC->pwm_config_.freewheel = mode;
  writeStoredPwmConfig(TMC);
}

void tmc_enableAutomaticCurrentScaling(TMC2209_t *TMC)
{
  TMC->pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig(TMC);
}

void tmc_disableAutomaticCurrentScaling(TMC2209_t *TMC)
{
  TMC->pwm_config_.pwm_autoscale = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig(TMC);
}

void tmc_enableAutomaticGradientAdaptation(TMC2209_t *TMC)
{
  TMC->pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_ON;
  writeStoredPwmConfig(TMC);
}

void tmc_disableAutomaticGradientAdaptation(TMC2209_t *TMC)
{
  TMC->pwm_config_.pwm_autograd = STEPPER_DRIVER_FEATURE_OFF;
  writeStoredPwmConfig(TMC);
}

void tmc_setPwmOffset(TMC2209_t *TMC, uint8_t pwm_amplitude)
{
  TMC->pwm_config_.pwm_offset = pwm_amplitude;
  writeStoredPwmConfig(TMC);
}

void tmc_setPwmGradient(TMC2209_t *TMC, uint8_t pwm_amplitude)
{
  TMC->pwm_config_.pwm_grad = pwm_amplitude;
  writeStoredPwmConfig(TMC);
}

void tmc_setPowerDownDelay(TMC2209_t *TMC, uint8_t power_down_delay)
{
  write(TMC, ADDRESS_TPOWERDOWN, power_down_delay);
}

void tmc_setReplyDelay(TMC2209_t *TMC, uint8_t reply_delay)
{
  if (reply_delay > REPLY_DELAY_MAX)
  {
    reply_delay = REPLY_DELAY_MAX;
  }
  tmc_ReplyDelay_t reply_delay_data;
  reply_delay_data.bytes = 0;
  reply_delay_data.replydelay = reply_delay;
  write(TMC, ADDRESS_REPLYDELAY, reply_delay_data.bytes);
}

void tmc_moveAtVelocity(TMC2209_t *TMC, int32_t microsteps_per_period)
{
  write(TMC, ADDRESS_VACTUAL, microsteps_per_period);
}

void tmc_moveUsingStepDirInterface(TMC2209_t *TMC)
{
  write(TMC, ADDRESS_VACTUAL, VACTUAL_STEP_DIR_INTERFACE);
}

void tmc_enableStealthChop(TMC2209_t *TMC)
{
  TMC->global_config_.enable_spread_cycle = 0;
  writeStoredGlobalConfig(TMC);
}

void tmc_disableStealthChop(TMC2209_t *TMC)
{
  TMC->global_config_.enable_spread_cycle = 1;
  writeStoredGlobalConfig(TMC);
}

void tmc_setCoolStepDurationThreshold(TMC2209_t *TMC, uint32_t duration_threshold)
{
  write(TMC, ADDRESS_TCOOLTHRS, duration_threshold);
}

void tmc_setStealthChopDurationThreshold(TMC2209_t *TMC, uint32_t duration_threshold)
{
  write(TMC, ADDRESS_TPWMTHRS, duration_threshold);
}

void tmc_setStallGuardThreshold(TMC2209_t *TMC, uint8_t stall_guard_threshold)
{
  write(TMC, ADDRESS_SGTHRS, stall_guard_threshold);
}

void tmc_enableCoolStep(TMC2209_t *TMC, uint8_t lower_threshold, uint8_t upper_threshold)
{
  lower_threshold = constrain_(lower_threshold, SEMIN_MIN, SEMIN_MAX);
  TMC->cool_config_.semin = lower_threshold;
  upper_threshold = constrain_(upper_threshold, SEMAX_MIN, SEMAX_MAX);
  TMC->cool_config_.semax = upper_threshold;
  write(TMC, ADDRESS_COOLCONF, TMC->cool_config_.bytes);
  TMC->cool_step_enabled_ = true;
}

void tmc_disableCoolStep(TMC2209_t *TMC)
{
  TMC->cool_config_.semin = SEMIN_OFF;
  write(TMC, ADDRESS_COOLCONF, TMC->cool_config_.bytes);
  TMC->cool_step_enabled_ = false;
}

void tmc_setCoolStepCurrentIncrement(TMC2209_t *TMC, CurrentIncrement current_increment)
{
  TMC->cool_config_.seup = current_increment;
  write(TMC, ADDRESS_COOLCONF, TMC->cool_config_.bytes);
}

void tmc_setCoolStepMeasurementCount(TMC2209_t *TMC, MeasurementCount measurement_count)
{
  TMC->cool_config_.sedn = measurement_count;
  write(TMC, ADDRESS_COOLCONF, TMC->cool_config_.bytes);
}

void tmc_enableAnalogCurrentScaling(TMC2209_t *TMC)
{
  TMC->global_config_.i_scale_analog = 1;
  writeStoredGlobalConfig(TMC);
}

void tmc_disableAnalogCurrentScaling(TMC2209_t *TMC)
{
  TMC->global_config_.i_scale_analog = 0;
  writeStoredGlobalConfig(TMC);
}

void tmc_useExternalSenseResistors(TMC2209_t *TMC)
{
  TMC->global_config_.internal_rsense = 0;
  writeStoredGlobalConfig(TMC);
}

void tmc_useInternalSenseResistors(TMC2209_t *TMC)
{
  TMC->global_config_.internal_rsense = 1;
  writeStoredGlobalConfig(TMC);
}

// bidirectional methods

uint8_t tmc_getVersion(TMC2209_t *TMC)
{
  tmc_Input_t input;
  input.bytes = read(TMC, ADDRESS_IOIN);

  return input.version;
}

bool tmc_isCommunicating(TMC2209_t *TMC)
{
  return (tmc_getVersion(TMC) == VERSION);
}

bool tmc_isSetupAndCommunicating(TMC2209_t *TMC)
{
  return serialOperationMode(TMC);
}

bool tmc_isCommunicatingButNotSetup(TMC2209_t *TMC)
{
  return (tmc_isCommunicating(TMC) && (!tmc_isSetupAndCommunicating(TMC)));
}

bool tmc_hardwareDisabled(TMC2209_t *TMC)
{
  tmc_Input_t input;
  input.bytes = read(TMC, ADDRESS_IOIN);

  return input.enn;
}

uint16_t tmc_getMicrostepsPerStep(TMC2209_t *TMC)
{
  uint16_t microsteps_per_step_exponent;
  switch (TMC->chopper_config_.mres)
  {
  case MRES_001:
  {
    microsteps_per_step_exponent = 0;
    break;
  }
  case MRES_002:
  {
    microsteps_per_step_exponent = 1;
    break;
  }
  case MRES_004:
  {
    microsteps_per_step_exponent = 2;
    break;
  }
  case MRES_008:
  {
    microsteps_per_step_exponent = 3;
    break;
  }
  case MRES_016:
  {
    microsteps_per_step_exponent = 4;
    break;
  }
  case MRES_032:
  {
    microsteps_per_step_exponent = 5;
    break;
  }
  case MRES_064:
  {
    microsteps_per_step_exponent = 6;
    break;
  }
  case MRES_128:
  {
    microsteps_per_step_exponent = 7;
    break;
  }
  case MRES_256:
  default:
  {
    microsteps_per_step_exponent = 8;
    break;
  }
  }
  return 1 << microsteps_per_step_exponent;
}

tmc_Settings_t tmc_getSettings(TMC2209_t *TMC)
{
  tmc_Settings_t settings;
  settings.is_communicating = tmc_isCommunicating(TMC);

  if (settings.is_communicating)
  {
    readAndStoreRegisters(TMC);

    settings.is_setup = TMC->global_config_.pdn_disable;
    settings.software_enabled = (TMC->chopper_config_.toff > TOFF_DISABLE);
    settings.microsteps_per_step = tmc_getMicrostepsPerStep(TMC);
    settings.inverse_motor_direction_enabled = TMC->global_config_.shaft;
    settings.stealth_chop_enabled = !(TMC->global_config_.enable_spread_cycle);
    settings.standstill_mode = TMC->pwm_config_.freewheel;
    settings.irun_percent = currentSettingToPercent(TMC->driver_current_.irun);
    settings.irun_register_value = TMC->driver_current_.irun;
    settings.ihold_percent = currentSettingToPercent(TMC->driver_current_.ihold);
    settings.ihold_register_value = TMC->driver_current_.ihold;
    settings.iholddelay_percent = holdDelaySettingToPercent(TMC->driver_current_.iholddelay);
    settings.iholddelay_register_value = TMC->driver_current_.iholddelay;
    settings.automatic_current_scaling_enabled = TMC->pwm_config_.pwm_autoscale;
    settings.automatic_gradient_adaptation_enabled = TMC->pwm_config_.pwm_autograd;
    settings.pwm_offset = TMC->pwm_config_.pwm_offset;
    settings.pwm_gradient = TMC->pwm_config_.pwm_grad;
    settings.cool_step_enabled = TMC->cool_step_enabled_;
    settings.analog_current_scaling_enabled = TMC->global_config_.i_scale_analog;
    settings.internal_sense_resistors_enabled = TMC->global_config_.internal_rsense;
  }
  else
  {
    settings.is_setup = false;
    settings.software_enabled = false;
    settings.microsteps_per_step = 0;
    settings.inverse_motor_direction_enabled = false;
    settings.stealth_chop_enabled = false;
    settings.standstill_mode = TMC->pwm_config_.freewheel;
    settings.irun_percent = 0;
    settings.irun_register_value = 0;
    settings.ihold_percent = 0;
    settings.ihold_register_value = 0;
    settings.iholddelay_percent = 0;
    settings.iholddelay_register_value = 0;
    settings.automatic_current_scaling_enabled = false;
    settings.automatic_gradient_adaptation_enabled = false;
    settings.pwm_offset = 0;
    settings.pwm_gradient = 0;
    settings.cool_step_enabled = false;
    settings.analog_current_scaling_enabled = false;
    settings.internal_sense_resistors_enabled = false;
  }

  return settings;
}

tmc_Status_t tmc_getStatus(TMC2209_t *TMC)
{
  tmc_DriveStatus_t drive_status;
  drive_status.bytes = 0;
  drive_status.bytes = read(TMC, ADDRESS_DRV_STATUS);
  return drive_status.status;
}

tmc_GlobalStatus_t tmc_getGlobalStatus(TMC2209_t *TMC)
{
  tmc_GlobalStatusUnion_t global_status_union;
  global_status_union.bytes = 0;
  global_status_union.bytes = read(TMC, ADDRESS_GSTAT);
  return global_status_union.global_status;
}

void tmc_clearReset(TMC2209_t *TMC)
{
  tmc_GlobalStatusUnion_t global_status_union;
  global_status_union.bytes = 0;
  global_status_union.global_status.reset = 1;
  write(TMC, ADDRESS_GSTAT, global_status_union.bytes);
}

void tmc_clearDriveError(TMC2209_t *TMC)
{
  tmc_GlobalStatusUnion_t global_status_union;
  global_status_union.bytes = 0;
  global_status_union.global_status.drv_err = 1;
  write(TMC, ADDRESS_GSTAT, global_status_union.bytes);
}

uint8_t tmc_getInterfaceTransmissionCounter(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_IFCNT);
}

uint32_t tmc_getInterstepDuration(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_TSTEP);
}

uint16_t tmc_getStallGuardResult(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_SG_RESULT);
}

uint8_t tmc_getPwmScaleSum(TMC2209_t *TMC)
{
  tmc_PwmScale_t pwm_scale;
  pwm_scale.bytes = read(TMC, ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_sum;
}

int16_t tmc_getPwmScaleAuto(TMC2209_t *TMC)
{
  tmc_PwmScale_t pwm_scale;
  pwm_scale.bytes = read(TMC, ADDRESS_PWM_SCALE);

  return pwm_scale.pwm_scale_auto;
}

uint8_t tmc_getPwmOffsetAuto(TMC2209_t *TMC)
{
  tmc_PwmAuto_t pwm_auto;
  pwm_auto.bytes = read(TMC, ADDRESS_PWM_AUTO);

  return pwm_auto.pwm_offset_auto;
}

uint8_t tmc_getPwmGradientAuto(TMC2209_t *TMC)
{
  tmc_PwmAuto_t pwm_auto;
  pwm_auto.bytes = read(TMC, ADDRESS_PWM_AUTO);

  return pwm_auto.pwm_gradient_auto;
}

uint16_t tmc_getMicrostepCounter(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_MSCNT);
}

// private
static void initialize(TMC2209_t *TMC, tmc_SerialAddress serial_address)
{
  ESP_LOGI(TAG,"Set Mode to Serial");
  setOperationModeToSerial(TMC, serial_address);
  ESP_LOGI(TAG,"Attempting to set defaults");
  setRegistersToDefaults(TMC);
  ESP_LOGI(TAG,"Clear Drive Error");
  tmc_clearDriveError(TMC);

  ESP_LOGI(TAG,"Minimize motor current");
  minimizeMotorCurrent(TMC);
  tmc_disable(TMC);
  tmc_disableAutomaticCurrentScaling(TMC);
  tmc_disableAutomaticGradientAdaptation(TMC);
  
  ESP_LOGI(TAG, "chopper_config_ address: %p", (void *)&TMC->chopper_config_);
}

static int serialAvailable(TMC2209_t *TMC)
{
  if (TMC->hardware_serial_port_ != UART_NUM_MAX)
  {
    size_t available_bytes = 0;
    uart_get_buffered_data_len(TMC->hardware_serial_port_, &available_bytes);
    return (int)available_bytes; // Return number of bytes available
  }
  return 0;
}

size_t serialWrite(TMC2209_t *TMC, uint8_t c)
{
  if (TMC->hardware_serial_port_ != UART_NUM_MAX)
  {
    return uart_write_bytes(TMC->hardware_serial_port_, &c, sizeof(c));
  }
  return 0;
}

static int serialRead(TMC2209_t *TMC)
{
  uint8_t byte;

  if (TMC->hardware_serial_port_ != UART_NUM_MAX)
  {
    int length = uart_read_bytes(TMC->hardware_serial_port_, &byte, 1, 100 / portTICK_PERIOD_MS); // Read 1 byte with timeout
    return (length > 0) ? byte : -1;                                                              // Return byte if read successfully, otherwise return -1
  }
  return -1;
}

static void serialFlush(TMC2209_t *TMC)
{
  if (TMC->hardware_serial_port_ != UART_NUM_MAX)
  {
    uart_flush(TMC->hardware_serial_port_);
  }
  return;
}

static void setOperationModeToSerial(TMC2209_t *TMC, tmc_SerialAddress serial_address)
{
  TMC->serial_address_ = serial_address;

  TMC->global_config_.bytes = 0;
  TMC->global_config_.i_scale_analog = 0;
  TMC->global_config_.pdn_disable = 1;
  TMC->global_config_.mstep_reg_select = 1;
  TMC->global_config_.multistep_filt = 1;

  writeStoredGlobalConfig(TMC);
}

static void setRegistersToDefaults(TMC2209_t *TMC)
{
  TMC->driver_current_.bytes = 0;
  TMC->driver_current_.ihold = IHOLD_DEFAULT;
  TMC->driver_current_.irun = IRUN_DEFAULT;
  TMC->driver_current_.iholddelay = IHOLDDELAY_DEFAULT;
  write(TMC, ADDRESS_IHOLD_IRUN, TMC->driver_current_.bytes);

  TMC->chopper_config_.bytes = CHOPPER_CONFIG_DEFAULT;
  TMC->chopper_config_.tbl = TBL_DEFAULT;
  TMC->chopper_config_.hend = HEND_DEFAULT;
  TMC->chopper_config_.hstart = HSTART_DEFAULT;
  TMC->chopper_config_.toff = TOFF_DEFAULT;
  write(TMC, ADDRESS_CHOPCONF, TMC->chopper_config_.bytes);

  TMC->pwm_config_.bytes = PWM_CONFIG_DEFAULT;
  write(TMC, ADDRESS_PWMCONF, TMC->pwm_config_.bytes);

  TMC->cool_config_.bytes = COOLCONF_DEFAULT;
  write(TMC, ADDRESS_COOLCONF, TMC->cool_config_.bytes);

  write(TMC, ADDRESS_TPOWERDOWN, TPOWERDOWN_DEFAULT);
  write(TMC, ADDRESS_TPWMTHRS, TPWMTHRS_DEFAULT);
  write(TMC, ADDRESS_VACTUAL, VACTUAL_DEFAULT);
  write(TMC, ADDRESS_TCOOLTHRS, TCOOLTHRS_DEFAULT);
  write(TMC, ADDRESS_SGTHRS, SGTHRS_DEFAULT);
  write(TMC, ADDRESS_COOLCONF, COOLCONF_DEFAULT);
}

static void readAndStoreRegisters(TMC2209_t *TMC)
{
  TMC->global_config_.bytes = readGlobalConfigBytes(TMC);
  TMC->chopper_config_.bytes = readChopperConfigBytes(TMC);
  TMC->pwm_config_.bytes = readPwmConfigBytes(TMC);
}

bool serialOperationMode(TMC2209_t *TMC)
{
  tmc_GlobalConfig_t global_config;
  global_config.bytes = readGlobalConfigBytes(TMC);

  return global_config.pdn_disable;
}

void minimizeMotorCurrent(TMC2209_t *TMC)
{
  TMC->driver_current_.irun = CURRENT_SETTING_MIN;
  TMC->driver_current_.ihold = CURRENT_SETTING_MIN;
  writeStoredDriverCurrent(TMC);
}

static uint32_t reverseData(uint32_t data)
{
  uint32_t reversed_data = 0;
  uint8_t right_shift;
  uint8_t left_shift;
  for (uint8_t i = 0; i < DATA_SIZE; ++i)
  {
    right_shift = (DATA_SIZE - i - 1) * BITS_PER_BYTE;
    left_shift = i * BITS_PER_BYTE;
    reversed_data |= ((data >> right_shift) & BYTE_MAX_VALUE) << left_shift;
  }
  return reversed_data;
}

static uint8_t calculateCrcWrite(tmc_WriteReadReplyDatagram_t datagram, uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t byte;
  for (uint8_t i = 0; i < (datagram_size - 1); ++i)
  {
    byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    for (uint8_t j = 0; j < BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

static uint8_t calculateCrcRead(tmc_ReadRequestDatagram_t datagram, uint8_t datagram_size)
{
  uint8_t crc = 0;
  uint8_t byte;
  for (uint8_t i = 0; i < (datagram_size - 1); ++i)
  {
    byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    for (uint8_t j = 0; j < BITS_PER_BYTE; ++j)
    {
      if ((crc >> 7) ^ (byte & 0x01))
      {
        crc = (crc << 1) ^ 0x07;
      }
      else
      {
        crc = crc << 1;
      }
      byte = byte >> 1;
    }
  }
  return crc;
}

static void sendDatagramUnidirectional(TMC2209_t *TMC, tmc_WriteReadReplyDatagram_t datagram, uint8_t datagram_size)
{
  uint8_t byte;

  for (uint8_t i = 0; i < datagram_size; ++i)
  {
    byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    serialWrite(TMC, byte);
  }
}

static void sendDatagramBidirectional(TMC2209_t *TMC, tmc_ReadRequestDatagram_t datagram, uint8_t datagram_size)
{
  uint8_t byte;

  // Wait for the transmission of outgoing serial data to complete
  serialFlush(TMC);

  // clear the serial receive buffer if necessary
  while (serialAvailable(TMC) > 0)
  {
    byte = serialRead(TMC);
  }

  // write datagram
  for (uint8_t i = 0; i < datagram_size; ++i)
  {
    byte = (datagram.bytes >> (i * BITS_PER_BYTE)) & BYTE_MAX_VALUE;
    serialWrite(TMC, byte);
  }

  // Wait for the transmission of outgoing serial data to complete
  serialFlush(TMC);

  // wait for bytes sent out on TX line to be echoed on RX line
  uint32_t echo_delay = 0;
  while ((serialAvailable(TMC) < datagram_size) && (echo_delay < ECHO_DELAY_MAX_MICROSECONDS))
  {
    vTaskDelay(ECHO_DELAY_INC_MICROSECONDS / portTICK_PERIOD_MS);
    echo_delay += ECHO_DELAY_INC_MICROSECONDS;
  }

  if (echo_delay >= ECHO_DELAY_MAX_MICROSECONDS)
  {
    return;
  }

  // clear RX buffer of echo bytes
  for (uint8_t i = 0; i < datagram_size; ++i)
  {
    byte = serialRead(TMC);
  }
}

static void write(TMC2209_t *TMC, uint8_t register_address, uint32_t data)
{
  tmc_WriteReadReplyDatagram_t write_datagram;
  write_datagram.bytes = 0;
  write_datagram.sync = SYNC;
  write_datagram.serial_address = TMC->serial_address_;
  write_datagram.register_address = register_address;
  write_datagram.rw = RW_WRITE;
  write_datagram.data = reverseData(data);
  write_datagram.crc = calculateCrcWrite(write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);

  sendDatagramUnidirectional(TMC, write_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
}

static uint32_t read(TMC2209_t *TMC, uint8_t register_address)
{
  tmc_ReadRequestDatagram_t read_request_datagram;
  read_request_datagram.bytes = 0;
  read_request_datagram.sync = SYNC;
  read_request_datagram.serial_address = TMC->serial_address_;
  read_request_datagram.register_address = register_address;
  read_request_datagram.rw = RW_READ;
  read_request_datagram.crc = calculateCrcRead(read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

  for (uint8_t retry = 0; retry < MAX_READ_RETRIES; retry++)
  {
    sendDatagramBidirectional(TMC, read_request_datagram, READ_REQUEST_DATAGRAM_SIZE);

    uint32_t reply_delay = 0;
    while ((serialAvailable(TMC) < WRITE_READ_REPLY_DATAGRAM_SIZE) && (reply_delay < REPLY_DELAY_MAX_MICROSECONDS))
    {
      vTaskDelay(REPLY_DELAY_INC_MICROSECONDS / portTICK_PERIOD_MS);
      reply_delay += REPLY_DELAY_INC_MICROSECONDS;
    }

    if (reply_delay >= REPLY_DELAY_MAX_MICROSECONDS)
    {
      return 0;
    }

    uint64_t byte;
    uint8_t byte_count = 0;
    tmc_WriteReadReplyDatagram_t read_reply_datagram;
    read_reply_datagram.bytes = 0;
    for (uint8_t i = 0; i < WRITE_READ_REPLY_DATAGRAM_SIZE; ++i)
    {
      byte = serialRead(TMC);
      read_reply_datagram.bytes |= (byte << (byte_count++ * BITS_PER_BYTE));
    }

    uint8_t crc = calculateCrcWrite(read_reply_datagram, WRITE_READ_REPLY_DATAGRAM_SIZE);
    if (crc == read_reply_datagram.crc)
    {
      return reverseData(read_reply_datagram.data);
    }

    vTaskDelay(READ_RETRY_DELAY_MS / portTICK_PERIOD_MS);
  }

  return 0;
}

static uint8_t percentToCurrentSetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent, PERCENT_MIN, PERCENT_MAX);
  uint8_t current_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX);
  return current_setting;
}

static uint8_t currentSettingToPercent(uint8_t current_setting)
{
  uint8_t percent = map(current_setting, CURRENT_SETTING_MIN, CURRENT_SETTING_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

static uint8_t percentToHoldDelaySetting(uint8_t percent)
{
  uint8_t constrained_percent = constrain_(percent, PERCENT_MIN, PERCENT_MAX);
  uint8_t hold_delay_setting = map(constrained_percent, PERCENT_MIN, PERCENT_MAX, HOLD_DELAY_MIN, HOLD_DELAY_MAX);
  return hold_delay_setting;
}

static uint8_t holdDelaySettingToPercent(uint8_t hold_delay_setting)
{
  uint8_t percent = map(hold_delay_setting, HOLD_DELAY_MIN, HOLD_DELAY_MAX, PERCENT_MIN, PERCENT_MAX);
  return percent;
}

static void writeStoredGlobalConfig(TMC2209_t *TMC)
{
  write(TMC, ADDRESS_GCONF, TMC->global_config_.bytes);
}

static uint32_t readGlobalConfigBytes(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_GCONF);
}

static void writeStoredDriverCurrent(TMC2209_t *TMC)
{
  write(TMC, ADDRESS_IHOLD_IRUN, TMC->driver_current_.bytes);

  if (TMC->driver_current_.irun >= SEIMIN_UPPER_CURRENT_LIMIT)
  {
    TMC->cool_config_.seimin = SEIMIN_UPPER_SETTING;
  }
  else
  {
    TMC->cool_config_.seimin = SEIMIN_LOWER_SETTING;
  }
  if (TMC->cool_step_enabled_)
  {
    write(TMC, ADDRESS_COOLCONF, TMC->cool_config_.bytes);
  }
}

static void writeStoredChopperConfig(TMC2209_t *TMC)
{
  write(TMC, ADDRESS_CHOPCONF, TMC->chopper_config_.bytes);
}

static uint32_t readChopperConfigBytes(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_CHOPCONF);
}

static void writeStoredPwmConfig(TMC2209_t *TMC)
{
  write(TMC, ADDRESS_PWMCONF, TMC->pwm_config_.bytes);
}

static uint32_t readPwmConfigBytes(TMC2209_t *TMC)
{
  return read(TMC, ADDRESS_PWMCONF);
}

static uint32_t constrain_(uint32_t value, uint32_t low, uint32_t high)
{
  return ((value) < (low) ? (low) : ((value) > (high) ? (high) : (value)));
}
static uint8_t map(uint8_t value, uint8_t fromLow, uint8_t fromHigh, uint8_t toLow, uint8_t toHigh)
{
  return toLow + ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow);
}
