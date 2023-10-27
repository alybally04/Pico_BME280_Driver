/**
    \mainpage BME280 "Plug and Play" Driver for the Pico/Pico W

    \section intro_sec Driver

    TODO!

    @file bme280_defs.h
    @copyright Evelyn Jones 2023
    @brief Macros, constants, and definitions for driver
*/

// Handy links:
// https://i2cdevices.org/addresses

#ifndef BME280_DEFS
#define BME280_DEFS

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

// Error codes
#define BME280_OK 0
/** @brief Generic Error Code */
#define BME280_ERR (-1)
#define BME280_ID_READ_FAIL (-2)
#define BME280_INVALID_ID (-3)
#define BME280_SETTINGS_WRITE_FAIL (-4)
#define BME280_CALIB_RD_ERR (-5)

#endif // BME280_DEFS