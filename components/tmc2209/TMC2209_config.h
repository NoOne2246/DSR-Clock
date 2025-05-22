/**
 ******************************************************************************
 * @file    TMC2209_config.h
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

#ifndef TMC2209_CONFIG_H
#define TMC2209_CONFIG_H

#include <stdint.h>

// Serial tmc_Settings_t
const static uint8_t BYTE_MAX_VALUE = 0xFF;
const static uint8_t BITS_PER_BYTE = 8;

const static uint32_t ECHO_DELAY_INC_MICROSECONDS = 1;
const static uint32_t ECHO_DELAY_MAX_MICROSECONDS = 4000;

const static uint32_t REPLY_DELAY_INC_MICROSECONDS = 1;
const static uint32_t REPLY_DELAY_MAX_MICROSECONDS = 10000;

const static uint8_t REPLY_DELAY_MAX = 15;

const static uint8_t STEPPER_DRIVER_FEATURE_OFF = 0;
const static uint8_t STEPPER_DRIVER_FEATURE_ON = 1;

const static uint8_t MAX_READ_RETRIES = 5;
const static uint32_t READ_RETRY_DELAY_MS = 20;

// Datagrams
const static uint8_t WRITE_READ_REPLY_DATAGRAM_SIZE = 8;
const static uint8_t DATA_SIZE = 4;
const static uint8_t SYNC = 0b101;
const static uint8_t RW_READ = 0;
const static uint8_t RW_WRITE = 1;

const static uint8_t READ_REPLY_SERIAL_ADDRESS = 0b11111111;
const static uint8_t READ_REQUEST_DATAGRAM_SIZE = 4;

// General Configuration Registers
const static uint8_t ADDRESS_GCONF = 0x00;
const static uint8_t ADDRESS_GSTAT = 0x01;
const static uint8_t ADDRESS_IFCNT = 0x02;
const static uint8_t ADDRESS_REPLYDELAY = 0x03;
const static uint8_t ADDRESS_IOIN = 0x06;

const static uint8_t VERSION = 0x21;

// Velocity Dependent Driver Feature Control Register Set
const static uint8_t ADDRESS_IHOLD_IRUN = 0x10;

const static uint8_t PERCENT_MIN = 0;
const static uint8_t PERCENT_MAX = 100;
const static uint8_t CURRENT_SETTING_MIN = 0;
const static uint8_t CURRENT_SETTING_MAX = 31;
const static uint8_t HOLD_DELAY_MIN = 0;
const static uint8_t HOLD_DELAY_MAX = 15;
const static uint8_t IHOLD_DEFAULT = 16;
const static uint8_t IRUN_DEFAULT = 31;
const static uint8_t IHOLDDELAY_DEFAULT = 1;

const static uint8_t ADDRESS_TPOWERDOWN = 0x11;
const static uint8_t TPOWERDOWN_DEFAULT = 20;

const static uint8_t ADDRESS_TSTEP = 0x12;

const static uint8_t ADDRESS_TPWMTHRS = 0x13;
const static uint32_t TPWMTHRS_DEFAULT = 0;

const static uint8_t ADDRESS_VACTUAL = 0x22;
const static int32_t VACTUAL_DEFAULT = 0;
const static int32_t VACTUAL_STEP_DIR_INTERFACE = 0;

// CoolStep and StallGuard Control Register Set
const static uint8_t ADDRESS_TCOOLTHRS = 0x14;
const static uint8_t TCOOLTHRS_DEFAULT = 0;
const static uint8_t ADDRESS_SGTHRS = 0x40;
const static uint8_t SGTHRS_DEFAULT = 0;
const static uint8_t ADDRESS_SG_RESULT = 0x41;

const static uint8_t ADDRESS_COOLCONF = 0x42;
const static uint8_t COOLCONF_DEFAULT = 0;

const static uint8_t SEIMIN_UPPER_CURRENT_LIMIT = 20;
const static uint8_t SEIMIN_LOWER_SETTING = 0;
const static uint8_t SEIMIN_UPPER_SETTING = 1;
const static uint8_t SEMIN_OFF = 0;
const static uint8_t SEMIN_MIN = 1;
const static uint8_t SEMIN_MAX = 15;
const static uint8_t SEMAX_MIN = 0;
const static uint8_t SEMAX_MAX = 15;

// Microstepping Control Register Set
const static uint8_t ADDRESS_MSCNT = 0x6A;
const static uint8_t ADDRESS_MSCURACT = 0x6B;

// Driver Register Set
const static uint8_t ADDRESS_CHOPCONF = 0x6C;

const static uint32_t CHOPPER_CONFIG_DEFAULT = 0x10000053;
const static uint8_t TBL_DEFAULT = 0b10;
const static uint8_t HEND_DEFAULT = 0;
const static uint8_t HSTART_DEFAULT = 5;
const static uint8_t TOFF_DEFAULT = 3;
const static uint8_t TOFF_DISABLE = 0;

const static uint8_t MRES_256 = 0b0000;
const static uint8_t MRES_128 = 0b0001;
const static uint8_t MRES_064 = 0b0010;
const static uint8_t MRES_032 = 0b0011;
const static uint8_t MRES_016 = 0b0100;
const static uint8_t MRES_008 = 0b0101;
const static uint8_t MRES_004 = 0b0110;
const static uint8_t MRES_002 = 0b0111;
const static uint8_t MRES_001 = 0b1000;

const static uint8_t DOUBLE_EDGE_DISABLE = 0;
const static uint8_t DOUBLE_EDGE_ENABLE = 1;
const static uint8_t VSENSE_DISABLE = 0;
const static uint8_t VSENSE_ENABLE = 1;

const static size_t MICROSTEPS_PER_STEP_MIN = 1;
const static size_t MICROSTEPS_PER_STEP_MAX = 256;

const static uint8_t ADDRESS_DRV_STATUS = 0x6F;
const static uint8_t ADDRESS_PWMCONF = 0x70;

const static uint32_t PWM_CONFIG_DEFAULT = 0xC10D0024;
const static uint8_t PWM_OFFSET_MIN = 0;
const static uint8_t PWM_OFFSET_MAX = 255;
const static uint8_t PWM_OFFSET_DEFAULT = 0x24;
const static uint8_t PWM_GRAD_MIN = 0;
const static uint8_t PWM_GRAD_MAX = 255;
const static uint8_t PWM_GRAD_DEFAULT = 0x14;

const static uint8_t ADDRESS_PWM_SCALE = 0x71;
const static uint8_t ADDRESS_PWM_AUTO = 0x72;

const static uint8_t HIGH = 1;
const static uint8_t LOW = 0;


#endif