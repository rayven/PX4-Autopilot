/**
 * @file board_config.h
 *
 * RPI internal definitions
 */

#pragma once

#define BOARD_OVERRIDE_UUID "RPIID00000000000" // must be of length 16
#define PX4_SOC_ARCH_ID     PX4_SOC_ARCH_ID_RPI


// I2C
#define CONFIG_I2C 1
#define PX4_NUMBER_I2C_BUSES    2


// SPI
#define CONFIG_SPI 1
#define PX4_NUMBER_SPI_BUSES    2

#define ADC_BATTERY_VOLTAGE_CHANNEL	0
#define ADC_BATTERY_CURRENT_CHANNEL	-1
#define ADC_AIRSPEED_VOLTAGE_CHANNEL 2

#define ADC_DP_V_DIV 1.0f

#include <system_config.h>
#include <px4_platform_common/board_common.h>
