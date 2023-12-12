/**
 * @file bme280_driver.h
 * @copyright Evelyn Jones 2023
 * @brief BME280 Pi Pico Driver Header
*/

#ifndef BME280_DRIVER
#define BME280_DRIVER

#include "bme280_defs.h"
#include "hardware/i2c.h"

/* NOTE:

To write bytes a, b, c to I2C device x register y

output = [y, a, b, c]

i2c_write_blocking (i2c, x, &output, 4, false)

To read 6 bytes from I2C device x register y

output = [y]
input = [0, 0, 0, 0, 0, 0]

i2c_write_blocking (i2c, x, &output, 1, true)
i2c_read_blocking (i2c, x, &input, 6, false)

*/

/**
 * @brief This struct holds a BME280 sensor instance. The members of this struct  should not be accessed directly except
 * temperature, humidity, and pressure.
*/
struct bme280_inst {
    /** @brief i2c bus sensor is connected to */
    i2c_inst_t i2c_b;
    /** @brief Sensor i2c address */
    uint8_t sens_addr;

    /** @brief Settings register values */
    uint8_t config, ctrl_hum, ctrl_meas;

    /** @brief Contains pressure in Pa, first 24 bits contain represent part and last 8 bits represent fractional part. */
    uint32_t pressure;
    /** @brief Contains temperature in Degrees C to 2 decimal place (The last 2 digits of the integer represent the fractional part, e.g. 1321 is 13.21 DegC). */
    int32_t temperature;
    /** @brief Contains humidity as a percentage, first 22 bits represent integer part and last 10 bits represent fractional part. */
    uint32_t humidity;

    /**
     * \name Below is Calibration Data - DO NOT MODIFY - Set by init function from constant data on sensor. See datasheet for more information.
    */
    /** @brief Constant calibration data - DO NOT MODIFY! */
    uint16_t dig_T1;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_T2;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_T3;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    uint16_t dig_P1;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P2;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P3;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P4;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P5;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P6;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P7;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P8;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_P9;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    uint8_t dig_H1;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_H2;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    uint8_t dig_H3;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_H4;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int16_t dig_H5;
    /** @brief Constant calibration data - DO NOT MODIFY! */
    int8_t dig_H6;
};

/**
 * @brief Call with driver error code to get a string explaining the error.
 * @return A string literal containing a description of the error.
 * @param errcode The error code.
*/
const char * bme280_strerr(int8_t errcode);

/**
 * @brief Find if an i2c address is reserved in the i2c protocol.
 * @return Returns true if addr is reserved in i2c, false otherwise.
 * @param addr The address to check.
*/
bool i2c_reserved_addr(uint8_t addr);

/**
 * @brief Find the addresses of all BME280 sensors attached to given i2c bus.
 * @return Returns NULL on error or array of uint8_t, where the first element holds the size of the array.
 * If the array size is 0 then no sensors were found. The array MUST be freed by calling free() once finished with.
 * @param i2c_bus Which Pico i2c bus to search.
*/
uint8_t * get_bme280_addrs(i2c_inst_t *i2c_bus);

/**
 * @brief Read data from BME280 register(s).
 * @return Returns BME280_READ_ERR on error or number of bytes read on success.
 * @param sensor_inst BME280 sensor instance to read from to.
 * @param reg_addr uint8_t containing address of register to begin reading from.
 * @param read_dest Pointer to array of uint8_t to write the read data to.
 * @param to_read Number of bytes to read.
 *
 * This function should typically only need to be used by the driver internally.
*/
int read_bme280(struct bme280_inst *sensor_inst, uint8_t reg_addr, uint8_t *read_dest, int to_read);

/**
 * @brief Write data to BME280 register(s).
 * @return Returns BME280_WRITE_ERR on error or number of bytes written on success.
 * @param sensor_inst BME280 sensor instance to write to.
 * @param data Pointer to array of uint8_t to write - First octet must be register addr to write to.
 * @param to_write Number of bytes to write.
 *
 * This function should typically only need to be used by the driver internally.
*/
int write_bme280(struct bme280_inst *sensor_inst, uint8_t *data, int to_write);

/**
 * @brief Temperature compensation formula implementation from BME280 datasheet.
 * @return Type int32_t containing temperature in Degrees C to 2 decimal place (The last 2 digits of the integer represent the fractional part, e.g. 1321 is 13.21 DegC).
 * @param sensor_inst Sensor instance reading was taken from (to get compensation values).
 * @param raw_T Raw temperature value to compensate.
 *
 * This function should typically only need to be used by the driver internally.
*/
int32_t BME280_compensate_T_int32(struct bme280_inst *sensor_inst, int32_t raw_T);

/**
 * @brief Pressure compensation formula implementation from BME280 datasheet.
 * @return Type uint32_t containing pressure in Pa, first 24 bits contain represent part and last 8 bits represent fractional part.
 * @param sensor_inst Sensor instance reading was taken from (to get compensation values).
 * @param raw_P Raw pressure value to compensate.
 *
 * Note: I have no idea why but in the datasheet the function name contains "int64" despite not taking as param nor returning a 64 bit number,
 * but for clarity of which datasheet function it is I have left the name as is.
 * This function should typically only need to be used by the driver internally.
*/
uint32_t BME280_compensate_P_int64(struct bme280_inst *sensor_inst, int32_t raw_P);

/**
 * @brief Pressure compensation formula implementation from BME280 datasheet.
 * @return Type uint32_t Containing humidity as a percentage, first 22 bits represent integer part and last 10 bits represent fractional part.
 * @param sensor_inst Sensor instance reading was taken from (to get compensation values).
 * @param raw_H Raw humidity value to compensate.
 *
 * This function should typically only need to be used by the driver internally.
*/
uint32_t BME280_compensate_H_int32(struct bme280_inst *sensor_inst, int32_t raw_H);

/**
 * @brief Perform a forced-mode sensor read using settings contained in sensor instance struct (The sensor must be in forced mode!).
 * @return Returns BME280_OK on success or a BME280 error code on error (These can be found in `bme280_defs.h` or \ref constants_sec).
 * @param sensor_inst BME280 Sensor instance to perform a forced read on.
 *
 * The sensor values are read, then compensated, then stored in the sensor instance struct.
 */
int8_t bme280_forced_read(struct bme280_inst *sensor_inst);

/**
 * @brief Read the current values of a sensor during normal-mode operation.
 * @return Returns BME280_OK on success or a BME280 error code on error (These can be found in `bme280_defs.h` or \ref constants_sec).
 * @param sensor_inst BME280 Sensor instance to read from.
 *
 * The sensor values are read, then compensated, then stored in the sensor instance struct.
*/
int8_t bme280_normal_read(struct bme280_inst *sensor_inst);

/**
 * @brief Writes a formatted output of the temperature value currently held in sensor_inst to dest. dest MUST be of at least `BME280_T_STRLEN`.
 * @param sensor_inst The sensor instance to format the temperature value of.
 * @param dest The string to write the formatted string to - MUST be of at least `BME280_T_STRLEN`!
*/
void bme280_fmt_temp(struct bme280_inst *sensor_inst, char * dest);

/**
 * @brief Writes a formatted output of the humidity value currently held in sensor_inst to dest. dest MUST be of at least `BME280_H_STRLEN`.
 * @param sensor_inst The sensor instance to format the humidity value of.
 * @param dest The string to write the formatted string to - MUST be of at least `BME280_H_STRLEN`!
*/
void bme280_fmt_humid(struct bme280_inst *sensor_inst, char * dest);

/**
 * @brief Writes a formatted output of the pressure value currently held in sensor_inst to dest. dest MUST be of at least `BME280_P_STRLEN`.
 * @param sensor_inst The sensor instance to format the pressure value of.
 * @param dest The string to write the formatted string to - MUST be of at least `BME280_P_STRLEN`!
*/
void bme280_fmt_press(struct bme280_inst *sensor_inst, char * dest);

/**
 * @brief Initialise BME280 sensor with given settings.
 * @return Returns BME280_OK on success or a BME280 error code on error (These can be found in `bme280_defs.h` or \ref constants_sec).
 * @param i2c_bus Which i2c bus the sensor is connected to.
 * @param sens_addr The i2c address of the sensor.
 * @param dest_inst The bme280_inst struct to initialise the sensor with.
 * @param mode Operation Mode - Sleep (BME280_SLEEP_MODE) or Forced (BME280_FORCED_MODE) or Normal (BME280_NORMAL_MODE).
 * @param config Configuration Settings - Bitwise OR of Inactive duration (BME280_INACTIVE_MS_<x>) and IIR filter time constant (BME280_FILTER_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 * @param temperature_oversample - The temperature oversampling rate (BME280_T_OVERSAMPLE_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 * @param humidity_oversample - The humidity oversampling rate (BME280_H_OVERSAMPLE_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 * @param pressure_oversample -The pressure oversampling rate (BME280_P_OVERSAMPLE_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
*/
int8_t bme280_init(
    i2c_inst_t *i2c_bus,
    uint8_t sens_addr,
    struct bme280_inst *dest_inst,
    uint8_t mode,
    uint8_t config,
    uint8_t temperature_oversample,
    uint8_t humidity_oversample,
    uint8_t pressure_oversample
);

/**
 * @brief Apply new settings to BME280 sensor instance that has already been initialised by calling `bme280_init`.
 * @return Returns BME280_OK on success or a BME280 error code on error (These can be found in `bme280_defs.h`).
 * @param dest_inst The sensor instance to update the settings of.
 * @param mode Operation Mode - Sleep (BME280_SLEEP_MODE) or Forced (BME280_FORCED_MODE) or Normal (BME280_NORMAL_MODE).
 * @param config Configuration Settings - Bitwise OR of Inactive duration (BME280_INACTIVE_MS_<x>) and IIR filter time constant (BME280_FILTER_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 * @param temperature_oversample - The temperature oversampling rate (BME280_T_OVERSAMPLE_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 * @param humidity_oversample - The humidity oversampling rate (BME280_H_OVERSAMPLE_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 * @param pressure_oversample -The pressure oversampling rate (BME280_P_OVERSAMPLE_<x>). See \ref constants_sec or `bme280_defs.h` for more information.
 *
 * This function should be used to change settings during operation, e.g. switching mode or oversample rate.
 */
int8_t bme280_update_settings(
    struct bme280_inst *dest_inst,
    uint8_t mode,
    uint8_t config,
    uint8_t temperature_oversample,
    uint8_t humidity_oversample,
    uint8_t pressure_oversample
);

/**
 * @brief De-initialise BME280 sensor instance and put sensor into sleep mode.
 * @return Returns BME280_OK on success or a BME280 error code on error (These can be found in `bme280_defs.h` or \ref constants_sec).
 * @param dest_inst BME280 instance to de-initialise and clear.
 *
 * This functions sends a reset code to the sensor, clearing its registers and putting it into sleep mode.
*/
int8_t bme280_deinit(struct bme280_inst *dest_inst);

#endif // BME280_DRIVER