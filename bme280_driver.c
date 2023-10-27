#include "bme280_driver.h"

bool i2c_reserved_addr(uint8_t addr)
{
    // if addr is with 1111xxx or 0000xxx then reserved in i2c
    // 0x78 is 1111000 in binary
    addr &= 0x78;
    if (addr == 0 || addr == 0x78)
    { return true; }
    else
    { return false; }
}

uint8_t * get_bme280_addrs(uint8_t i2c_bus)
{
    // First element contains array length!
    int addr_max = 3;
    uint8_t *addrs = malloc(sizeof(uint8_t) * addr_max);
    addrs[0] = 1;

    // scan for address
    for (uint8_t addr = 0; addr < 128; addr++)
    {
        if (i2c_reserved_addr(addr))
        {
            continue;
        }

        uint8_t rt_data;
        if (i2c_read_blocking((i2c_bus == 0 ? i2c0 : i2c1), addr, &rt_data, 1, false) != PICO_ERROR_GENERIC)
        {
            // found device!
            if (addrs[0] == addr_max)
            {
                void *new = realloc(addrs, addr_max + 2);
                if (new == NULL)
                {
                    free(new);
                    free(addrs);
                    return NULL;
                }
                else
                {
                    addrs = (uint8_t *)new;
                }
            }
            else
            {
                addrs[addrs[0]] = addr;
                addrs[0]++;
            }
        }
    }

    if (addrs[0] == 1)
    {
        free(addrs);
        return NULL;
    }
    else
    {
        return addrs;
    }
}

int read_bme280(struct bme280_inst *sensor_inst, uint8_t reg_addr, uint8_t *rt_dest, int to_rd)
{
    if (i2c_write_blocking(&sensor_inst->i2c_b, sensor_inst->sens_addr, &reg_addr, 1, true) == PICO_ERROR_GENERIC)
    { return -1; }

    int res = i2c_read_blocking(&sensor_inst->i2c_b, sensor_inst->sens_addr, rt_dest, to_rd, true);
    if (res == PICO_ERROR_GENERIC)
    { return -1; }
    return res;
}

int write_bme280(struct bme280_inst *sensor_inst, uint8_t *data, int d_len)
{
    int res = i2c_write_blocking(&sensor_inst->i2c_b, sensor_inst->sens_addr, data, d_len, true);
    if (res == PICO_ERROR_GENERIC)
    {
        return -1;
    }
    return res;
}

// t_fine carries fine temperature as global value
int32_t t_fine;
int32_t BME280_compensate_T_int32(struct bme280_inst *sensor_inst, int32_t raw_T)
{
    int32_t var1, var2, T;
    var1 = ((((raw_T >> 3) - ((int32_t)sensor_inst->dig_T1 << 1))) * ((int32_t)sensor_inst->dig_T2)) >> 11;
    var2 = (((((raw_T >> 4) - ((int32_t)sensor_inst->dig_T1)) * ((raw_T >> 4) - ((int32_t)sensor_inst->dig_T1)))
    >> 12) *
            ((int32_t)sensor_inst->dig_T3)) >> 14;
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

// Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
// Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
uint32_t BME280_compensate_P_int64(struct bme280_inst *sensor_inst, int32_t raw_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)sensor_inst->dig_P6;
    var2 = var2 + ((var1*(int64_t)sensor_inst->dig_P5)<<17);
    var2 = var2 + (((int64_t)sensor_inst->dig_P4)<<35);
    var1 = ((var1 * var1 * (int64_t)sensor_inst->dig_P3)>>8) + ((var1 * (int64_t)sensor_inst->dig_P2)<<12);
    var1 = (((((int64_t)1)<<47)+var1))*((int64_t)sensor_inst->dig_P1)>>33;
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = 1048576 - raw_P;
    p = (((p<<31)-var2)*3125)/var1;
    var1 = (((int64_t)sensor_inst->dig_P9) * (p>>13) * (p>>13)) >> 25;
    var2 = (((int64_t)sensor_inst->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)sensor_inst->dig_P7)<<4);
    return (uint32_t)p;
}

// Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
// Output value of “47445” represents 47445/1024 = 46.333 %RH
uint32_t BME280_compensate_H_int32(struct bme280_inst *sensor_inst, int32_t raw_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (t_fine - ((int32_t)76800));
    v_x1_u32r = (((((raw_H << 14) - (((int32_t)sensor_inst->dig_H4) << 20) - (((int32_t)sensor_inst->dig_H5) *
                                                                              v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *
                    ((int32_t)sensor_inst->dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)sensor_inst->dig_H3)) >> 11) +
                            ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)sensor_inst->dig_H2) +8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *
                               ((int32_t)sensor_inst->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r>>12);
}

int8_t bme280_forced_read_all(struct bme280_inst *sensor_inst)
{
    uint8_t write_buff[2] = {BME280_REG_CONFIG, sensor_inst->config};
    // write config
    if (write_bme280(sensor_inst, write_buff, 2) < 0)
    {
        return BME280_SETTINGS_WRITE_FAIL;
    }
    // write ctrl_hum
    write_buff[0] = BME280_REG_CTRL_HUM;
    write_buff[1] = sensor_inst->ctrl_hum;
    if (write_bme280(sensor_inst, write_buff, 2) < 0)
    {
        return BME280_SETTINGS_WRITE_FAIL;
    }
    // write ctrl_meas
    write_buff[0] = BME280_REG_CTRL_MEAS;
    write_buff[1] = sensor_inst->ctrl_meas;
    if (write_bme280(sensor_inst, write_buff, 2) < 0)
    {
        return BME280_SETTINGS_WRITE_FAIL;
    }

    // wait for reading
    do
    {
        sleep_ms(50);
        if (read_bme280(sensor_inst, BME280_REG_STATUS, &write_buff[0], 1) < 0)
        {
            return BME280_ERR;
        }
    }
    while ((write_buff[0] & 0x8) == 0x8);

    uint8_t buffer[8];
    if (read_bme280(sensor_inst, BME280_READ_ALL_START_REG, buffer, 7) < 0)
    {
        return BME280_ERR;
    }

    int32_t press_raw = (buffer[0] << 12) | (buffer[1] << 4) | (buffer[2] & 0b00001111);
    int32_t temp_raw = (buffer[3] << 12) | (buffer[4] << 4) | (buffer[5] & 0b00001111);
    int32_t hum_raw = (buffer[6] << 8) | buffer[7];

    sensor_inst->temperature = BME280_compensate_T_int32(sensor_inst, temp_raw);
    sensor_inst->pressure = BME280_compensate_P_int64(sensor_inst, press_raw);
    sensor_inst->humidity = BME280_compensate_H_int32(sensor_inst, hum_raw);

    return BME280_OK;
}

int8_t bme280_init(i2c_inst_t *i2c_bus, uint8_t sens_addr, struct bme280_inst *dest_inst, uint8_t config, uint8_t ctrl_hum, uint8_t ctrl_meas)
{
    dest_inst->i2c_b = *i2c_bus;
    dest_inst->sens_addr = sens_addr;

    uint8_t res;
    if (read_bme280(dest_inst, BME280_REG_ID, &res, 1 ) < 0)
    {
        return BME280_ID_READ_FAIL;
    }
    if (res != 0x60)
    {
        return BME280_INVALID_ID;
    }

    uint8_t calib_buff[32];
    if (read_bme280(dest_inst, BME280_CALIB_0_25, &calib_buff[0], 25) < 0)
    { return BME280_CALIB_RD_ERR; }
    if(read_bme280(dest_inst, BME280_CALIB_26_41, &calib_buff[25], 7) < 0)
    { return BME280_CALIB_RD_ERR; }

    // loading calib registers
    dest_inst->dig_T1 = (uint16_t)((calib_buff[1]<<8) | calib_buff[0]);
    dest_inst->dig_T2 = (int16_t)((calib_buff[3]<<8) | calib_buff[2]);
    dest_inst->dig_T3 = (int16_t)((calib_buff[5]<<8) | calib_buff[4]);
    dest_inst->dig_P1 = (uint16_t)((calib_buff[7]<<8) | calib_buff[6]);
    dest_inst->dig_P2 = (int16_t)((calib_buff[9]<<8) | calib_buff[8]);
    dest_inst->dig_P3 = (int16_t)((calib_buff[11]<<8) | calib_buff[10]);
    dest_inst->dig_P4 = (int16_t)((calib_buff[13]<<8) | calib_buff[12]);
    dest_inst->dig_P5 = (int16_t)((calib_buff[15]<<8) | calib_buff[14]);
    dest_inst->dig_P6 = (int16_t)((calib_buff[17]<<8) | calib_buff[16]);
    dest_inst->dig_P7 = (int16_t)((calib_buff[19]<<8) | calib_buff[18]);
    dest_inst->dig_P8 = (int16_t)((calib_buff[21]<<8) | calib_buff[20]);
    dest_inst->dig_P9 = (int16_t)((calib_buff[23]<<8) | calib_buff[22]);
    dest_inst->dig_H1 = calib_buff[24];
    dest_inst->dig_H2 = (int16_t)((calib_buff[26]<<8) | calib_buff[25]);
    dest_inst->dig_H3 = calib_buff[27];
    dest_inst->dig_H4 = (int16_t)((calib_buff[28]<<4) | (calib_buff[29] & 0b00001111));
    dest_inst->dig_H5 = (int16_t)(((calib_buff[29] & 0b11110000)<<4)| calib_buff[30]);
    dest_inst->dig_H6 = (int8_t)calib_buff[31];

    dest_inst->config = config;
    dest_inst->ctrl_hum = ctrl_hum;
    dest_inst->ctrl_meas = ctrl_meas;

    return BME280_OK;
}

int8_t bme280_deinit(struct bme280_inst *dest_inst)
{
    uint8_t reg_reset_val[2] = {BME280_REG_RESET, BME280_REG_RESET_VAL};
    if (write_bme280(dest_inst, reg_reset_val, 2) != BME280_OK)
    {
        return BME280_ERR;
    }
    return BME280_OK;
}
