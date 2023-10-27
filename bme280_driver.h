/**
@file bme280_driver.h
@copyright Evelyn Jones 2023
@brief BME280 Pi Pico Driver Header
*/

#ifndef BME280_DRIVER
#define BME280_DRIVER

#include "bme280_defs.h"
#include <stdlib.h>
#include "hardware/i2c.h"

/* NOTE: (Because pico sdk docs are shit on this)

To write bytes a, b, c to I2C device x register y

output = [y, a, b, c]

i2c_write_blocking (i2c, x, &output, 4, false)

To read 6 bytes from I2C device x register y

output = [y]
input = [0, 0, 0, 0, 0, 0]

i2c_write_blocking (i2c, x, &output, 1, true)
i2c_read_blocking (i2c, x, &input, 6, false)

*/

struct bme280_inst {
    i2c_inst_t i2c_b;
    uint8_t sens_addr;

    uint8_t config, ctrl_hum, ctrl_meas;

    uint32_t pressure;
    int32_t temperature;
    uint32_t humidity;

    // Calibration Data - DO NOT MODIFY - Set by init function from constant data on sensor
    // See for more: https://web.archive.org/web/20231027120300if_/https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf#%5B%7B%22num%22%3A118%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C68%2C630%2C0%5D
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
    uint8_t dig_H1;
    int16_t dig_H2;
    uint8_t dig_H3;
    int16_t dig_H4;
    int16_t dig_H5;
    int8_t dig_H6;
};

/** @return Return true if addr is reserved in i2c, false otherwise */
bool i2c_reserved_addr(uint8_t addr);

/**
 * @return Returns array of device addresses on given i2c bus
 * @param i2c_bus Which i2c bus to search, 1 or 0
*/
uint8_t * get_bme280_addrs(uint8_t i2c_bus);

/**
 * @brief Read data from bme280 register
 * @return Returns -1 on error or number of bytes read on success
 * @param sensor_inst BME280 sensor instance to read from
 * @param reg_addr Register to read from
 * @param rt_dest Pointer to array of uint8_t to read data to
 * @param to_rd Number of bytes to read
*/
int read_bme280(struct bme280_inst *sensor_inst, uint8_t reg_addr, uint8_t *rt_dest, int to_rd);

/**
 * @brief write data to bme280 register -
 * @return Returns -1 on error or number of bytes written on success
 * @param sensor_inst BME280 sensor instance to write to
 * @param data Pointer to array of uint8_t to write - First octet must be register addr to write to
 * @param d_len Number of bytes to write
*/
int write_bme280(struct bme280_inst *sensor_inst, uint8_t *data, int d_len);

/**
 * @brief Temperature compensation formula from datasheet
 * @return int32 Containing temperature in DegC to 2 decimal place (Last 2 digits of val are fractional part)
 * @param sensor_inst Sensor reading was taken from
 * @param raw_T Raw temperature value to compensate
*/
int32_t BME280_compensate_T_int32(struct bme280_inst *sensor_inst, int32_t raw_T);

/**
 * @brief Pressure compensation formula from datasheet
 * @return uint32 Containing pressure in Pa, first 24 bits are integer bits and last 8 bits are fractional part
 * @param sensor_inst Sensor reading was taken from
 * @param raw_P Raw pressure value to compensate
*/
uint32_t BME280_compensate_P_int64(struct bme280_inst *sensor_inst, int32_t raw_P);

/**
 * @brief Humidity compensation formula from datasheet
 * @return uint32 Containing humidity as %, first 22 bits are integer bits and last 10 bits are fractional part
 * @param sensor_inst Sensor reading was taken from
 * @param raw_H Raw humidity value to compensate
*/
uint32_t BME280_compensate_H_int32(struct bme280_inst *sensor_inst, int32_t raw_H);

/**
 * @brief Perform a forced sensor read using settings contained in instance
 * @return Returns BME280_OK on success or BME280 error code otherwise
 * @param sensor_inst BME280 Sensor instance to forced read from
*/
int8_t bme280_forced_read_all(struct bme280_inst *sensor_inst);

/**
 * @brief Initialise BME280 sensor
 * @return BME280_OK if success, otherwise BME280_ERR
 * @param i2c_bus Which I2C bus to use
 * @param sens_addr The address of the sensor
 * @param dest_inst The bme280_inst struct to initialise with
 * @param config Configuration octet - See https://web.archive.org/web/20231027120300if_/https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf#%5B%7B%22num%22%3A134%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C68%2C771%2C0%5D For more information
 * @param ctrl_hum Humidity opts octet - See https://web.archive.org/web/20231027120300if_/https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf#%5B%7B%22num%22%3A126%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C68%2C390%2C0%5D For more information
 * @param ctrl_meas Measurement opts octet - See https://web.archive.org/web/20231027120300if_/https://www.mouser.com/datasheet/2/783/BST-BME280-DS002-1509607.pdf#%5B%7B%22num%22%3A131%2C%22gen%22%3A0%7D%2C%7B%22name%22%3A%22XYZ%22%7D%2C68%2C253%2C0%5D For more information
*/
int8_t bme280_init(i2c_inst_t *i2c_bus, uint8_t sens_addr, struct bme280_inst *dest_inst, uint8_t config, uint8_t ctrl_hum, uint8_t ctrl_meas);

/**
 * @brief De-initialise BME280 sensor instance
 * @return BME280_OK if success, otherwise BME280_ERR
 * @param dest_inst BME280 instance to de-initialise
*/
int8_t bme280_deinit(struct bme280_inst *dest_inst);

#endif // BME280_DRIVER