/**
 ******************************************************************************
 * @file    TMC2209_Struct.h
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

#ifndef TMC2209_STRUCT_H
#define TMC2209_STRUCT_H

#include <stdint.h>
#include "TMC2209.h"
#include <stddef.h>
#include "stdbool.h"

typedef struct
{
  bool is_communicating;
  bool is_setup;
  bool software_enabled;
  uint16_t microsteps_per_step;
  bool inverse_motor_direction_enabled;
  bool stealth_chop_enabled;
  uint8_t standstill_mode;
  uint8_t irun_percent;
  uint8_t irun_register_value;
  uint8_t ihold_percent;
  uint8_t ihold_register_value;
  uint8_t iholddelay_percent;
  uint8_t iholddelay_register_value;
  bool automatic_current_scaling_enabled;
  bool automatic_gradient_adaptation_enabled;
  uint8_t pwm_offset;
  uint8_t pwm_gradient;
  bool cool_step_enabled;
  bool analog_current_scaling_enabled;
  bool internal_sense_resistors_enabled;
} tmc_Settings_t;

typedef struct
{
  uint32_t reset : 1;
  uint32_t drv_err : 1;
  uint32_t uv_cp : 1;
  uint32_t reserved : 29;
} tmc_GlobalStatus_t;

typedef struct
{
  uint32_t over_temperature_warning : 1;
  uint32_t over_temperature_shutdown : 1;
  uint32_t short_to_ground_a : 1;
  uint32_t short_to_ground_b : 1;
  uint32_t low_side_short_a : 1;
  uint32_t low_side_short_b : 1;
  uint32_t open_load_a : 1;
  uint32_t open_load_b : 1;
  uint32_t over_temperature_120c : 1;
  uint32_t over_temperature_143c : 1;
  uint32_t over_temperature_150c : 1;
  uint32_t over_temperature_157c : 1;
  uint32_t reserved0 : 4;
  uint32_t current_scaling : 5;
  uint32_t reserved1 : 9;
  uint32_t stealth_chop_mode : 1;
  uint32_t standstill : 1;
} tmc_Status_t;

typedef union
{
  struct
  {
    uint64_t sync : 4;
    uint64_t reserved : 4;
    uint64_t serial_address : 8;
    uint64_t register_address : 7;
    uint64_t rw : 1;
    uint64_t data : 32;
    uint64_t crc : 8;
  };
  uint64_t bytes;
} tmc_WriteReadReplyDatagram_t;

typedef union
{
  struct
  {
    uint32_t sync : 4;
    uint32_t reserved : 4;
    uint32_t serial_address : 8;
    uint32_t register_address : 7;
    uint32_t rw : 1;
    uint32_t crc : 8;
  };
  uint32_t bytes;
} tmc_ReadRequestDatagram_t;

typedef union
{
  struct
  {
    uint32_t i_scale_analog : 1;
    uint32_t internal_rsense : 1;
    uint32_t enable_spread_cycle : 1;
    uint32_t shaft : 1;
    uint32_t index_otpw : 1;
    uint32_t index_step : 1;
    uint32_t pdn_disable : 1;
    uint32_t mstep_reg_select : 1;
    uint32_t multistep_filt : 1;
    uint32_t test_mode : 1;
    uint32_t reserved : 22;
  };
  uint32_t bytes;
} tmc_GlobalConfig_t;

typedef union
{
  struct
  {
    tmc_GlobalStatus_t global_status;
  };
  uint32_t bytes;
} tmc_GlobalStatusUnion_t;

typedef union
{
  struct
  {
    uint32_t reserved_0 : 8;
    uint32_t replydelay : 4;
    uint32_t reserved_1 : 20;
  };
  uint32_t bytes;
} tmc_ReplyDelay_t;

typedef union
{
  struct
  {
    uint32_t enn : 1;
    uint32_t reserved_0 : 1;
    uint32_t ms1 : 1;
    uint32_t ms2 : 1;
    uint32_t diag : 1;
    uint32_t reserved_1 : 1;
    uint32_t pdn_serial : 1;
    uint32_t step : 1;
    uint32_t spread_en : 1;
    uint32_t dir : 1;
    uint32_t reserved_2 : 14;
    uint32_t version : 8;
  };
  uint32_t bytes;
} tmc_Input_t;

typedef union
{
  struct
  {
    uint32_t ihold : 5;
    uint32_t reserved_0 : 3;
    uint32_t irun : 5;
    uint32_t reserved_1 : 3;
    uint32_t iholddelay : 4;
    uint32_t reserved_2 : 12;
  };
  uint32_t bytes;
} tmc_DriverCurrent_t;

typedef union
{
  struct
  {
    uint32_t semin : 4;
    uint32_t reserved_0 : 1;
    uint32_t seup : 2;
    uint32_t reserved_1 : 1;
    uint32_t semax : 4;
    uint32_t reserved_2 : 1;
    uint32_t sedn : 2;
    uint32_t seimin : 1;
    uint32_t reserved_3 : 16;
  };
  uint32_t bytes;
} tmc_CoolConfig_t;

typedef union
{
  struct
  {
    uint32_t toff : 4;
    uint32_t hstart : 3;
    uint32_t hend : 4;
    uint32_t reserved_0 : 4;
    uint32_t tbl : 2;
    uint32_t vsense : 1;
    uint32_t reserved_1 : 6;
    uint32_t mres : 4;
    uint32_t interpolation : 1;
    uint32_t double_edge : 1;
    uint32_t diss2g : 1;
    uint32_t diss2vs : 1;
  };
  uint32_t bytes;
} tmc_ChopperConfig_t;

typedef union
{
  struct
  {
    tmc_Status_t status;
  };
  uint32_t bytes;
} tmc_DriveStatus_t;

typedef union
{
  struct
  {
    uint32_t pwm_offset : 8;
    uint32_t pwm_grad : 8;
    uint32_t pwm_freq : 2;
    uint32_t pwm_autoscale : 1;
    uint32_t pwm_autograd : 1;
    uint32_t freewheel : 2;
    uint32_t reserved : 2;
    uint32_t pwm_reg : 4;
    uint32_t pwm_lim : 4;
  };
  uint32_t bytes;
} tmc_PwmConfig_t;

typedef union
{
  struct
  {
    uint32_t pwm_scale_sum : 8;
    uint32_t reserved_0 : 8;
    uint32_t pwm_scale_auto : 9;
    uint32_t reserved_1 : 7;
  };
  uint32_t bytes;
} tmc_PwmScale_t;

typedef union
{
  struct
  {
    uint32_t pwm_offset_auto : 8;
    uint32_t reserved_0 : 8;
    uint32_t pwm_gradient_auto : 8;
    uint32_t reserved_1 : 8;
  };
  uint32_t bytes;
} tmc_PwmAuto_t;

#endif