/**
 * @file bme280_defs.h
 * @copyright Evelyn Jones 2023
 * @brief Macros, constants, and definitions for driver
*/

// Handy links:
// https://i2cdevices.org/addresses

#ifndef BME280_DEFS
#define BME280_DEFS

/**
 * \name BME280 Register address constants - for internal driver use only
*/
#define BME280_REG_ID 0xD0
#define BME280_REG_RESET 0xE0
#define BME280_REG_RESET_VAL 0xB6
#define BME280_REG_CTRL_HUM 0xF2
#define BME280_REG_STATUS 0xF3
#define BME280_REG_CTRL_MEAS 0xF4
#define BME280_REG_CONFIG 0xF5
#define BME280_READ_ALL_START_REG 0xF7

#define BME280_CALIB_0_25 0x88
#define BME280_CALIB_26_41 0xE1

/**
 * \name Inactive duration constants for normal mode operation - The time between readings (BME280_INACTIVE_MS_0_5 is 0.5 ms, BME280_INACTIVE_MS_62_5 is 62.5 ms, etc)
*/
/** @brief 0.5 ms between normal mode readings */
#define BME280_INACTIVE_MS_0_5 0
/** @brief 62.5 ms between normal mode readings */
#define BME280_INACTIVE_MS_62_5 0b00100000
/** @brief 125 ms between normal mode readings */
#define BME280_INACTIVE_MS_125 0b01000000
/** @brief 250 ms between normal mode readings */
#define BME280_INACTIVE_MS_250 0b01100000
/** @brief 500 ms between normal mode readings */
#define BME280_INACTIVE_MS_500 0b10000000
/** @brief 1000 ms between normal mode readings */
#define BME280_INACTIVE_MS_1000 0b10100000
/** @brief 10 ms between normal mode readings */
#define BME280_INACTIVE_MS_10 0b11000000
/** @brief 20 ms between normal mode readings */
#define BME280_INACTIVE_MS_20 0b11100000

/**
 * \name IRR Filter Coefficients
*/
/** @brief IIR time filter disabled */
#define BME280_FILTER_OFF 0
/** @brief IIR time filter coefficient 2 */
#define BME280_FILTER_2 0b00100
/** @brief IIR time filter coefficient 4 */
#define BME280_FILTER_4 0b01000
/** @brief IIR time filter coefficient 8 */
#define BME280_FILTER_8 0b01100
/** @brief IIR time filter coefficient 16 */
#define BME280_FILTER_16 0b10000

/**
 * \name Pressure oversampling options
*/
/** @brief No Oversampling */
#define BME280_P_OVERSAMPLE_NONE 0
/** @brief 1x Oversampling */
#define BME280_P_OVERSAMPLE_1 0b00000100
/** @brief 2x Oversampling */
#define BME280_P_OVERSAMPLE_2 0b00001000
/** @brief 4x Oversampling */
#define BME280_P_OVERSAMPLE_4 0b00001100
/** @brief 8x Oversampling */
#define BME280_P_OVERSAMPLE_8 0b00010000
/** @brief 16x Oversampling */
#define BME280_P_OVERSAMPLE_16 0b00010100

/**
 * \name Temperature oversampling options
*/
/** @brief No Oversampling */
#define BME280_T_OVERSAMPLE_NONE 0
/** @brief 1x Oversampling */
#define BME280_T_OVERSAMPLE_1 0b00100000
/** @brief 2x Oversampling */
#define BME280_T_OVERSAMPLE_2 0b01000000
/** @brief 4x Oversampling */
#define BME280_T_OVERSAMPLE_4 0b01100000
/** @brief 8x Oversampling */
#define BME280_T_OVERSAMPLE_8 0b10000000
/** @brief 16x Oversampling */
#define BME280_T_OVERSAMPLE_16 0b10100000

/**
 * \name Humidity oversampling options
*/
/** @brief No Oversampling */
#define BME280_H_OVERSAMPLE_NONE 0
/** @brief 1x Oversampling */
#define BME280_H_OVERSAMPLE_1 0b001
/** @brief 2x Oversampling */
#define BME280_H_OVERSAMPLE_2 0b010
/** @brief 4x Oversampling */
#define BME280_H_OVERSAMPLE_4 0b011
/** @brief 8x Oversampling */
#define BME280_H_OVERSAMPLE_8 0b100
/** @brief 16x Oversampling */
#define BME280_H_OVERSAMPLE_16 0b101

/**
 * \name BME280 Operation modes
*/
#define BME280_SLEEP_MODE 0
#define BME280_FORCED_MODE 0b01
#define BME280_NORMAL_MODE 0b11

/** @brief No error occurred */
#define BME280_OK 0
/** @brief Generic Error Code */
#define BME280_ERR (-1)
/** @brief Failed to read sensor's chip ID during initialisation */
#define BME280_ID_READ_FAIL (-2)
/** @brief Sensor's chip ID does not match BME280 chip ID */
#define BME280_INVALID_ID (-3)
/** @brief Failed to write settings to sensor */
#define BME280_SETTINGS_WRITE_ERR (-4)
/** @brief Failed to read settings from sensor */
#define BME280_SETTINGS_READ_ERR (-5)
/** @brief Failed to read calibration data from sensor during initialisation */
#define BME280_CALIB_RD_ERR (-6)
/** @brief Failed to write to sensor registers - Error likely caused by Pico i2c functions */
#define BME280_WRITE_ERR (-7)
/** @brief Failed to read from sensor registers - Error likely caused by Pico i2c functions */
#define BME280_READ_ERR (-8)

/**
 * \name Formatted data string minimum lengths
*/
/** @brief Minimum formatted temperature string length */
#define BME280_T_STRLEN 10
/** @brief Minimum formatted humidity string length */
#define BME280_H_STRLEN 8
/** @brief Minimum formatted pressure string length */
#define BME280_P_STRLEN 13

#endif // BME280_DEFS
