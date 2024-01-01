#include <stdio.h>
#include "pico/stdlib.h"
#include <stdlib.h>

#include "bme280_driver.h"


int main()
{
    stdio_init_all();
    sleep_ms(5000);

    // start i2c
    i2c_init(i2c0, 100000);
    gpio_set_function(21, GPIO_FUNC_I2C);
    gpio_set_function(20, GPIO_FUNC_I2C);
    gpio_pull_up(21);
    gpio_pull_up(20);

    printf("Finding BME280 Sensor addresses...\n");
    // find bme280 on i2c
    uint8_t *found_addrs = get_bme280_addrs(i2c0);
    if (!found_addrs || found_addrs[0] == 1)
    {
        printf("Error: Failed to find bme280 addr...\n");
        free(found_addrs);
        return 1;
    }

    printf("Sucess! Found addresses:");
    // first number is array len
    for (uint8_t count = 1; count <= found_addrs[0]; ++count)
        printf(" %u", found_addrs[count]);

    printf("\n\nInitialising sensor to perform forced read...\n");

    // initialise sensor instance
    struct bme280_inst sensor;
    int8_t res = bme280_init(i2c0,
        found_addrs[1],
        &sensor,
        BME280_FORCED_MODE,
        BME280_FILTER_OFF,
        BME280_T_OVERSAMPLE_1,
        BME280_H_OVERSAMPLE_1,
        BME280_P_OVERSAMPLE_1
    );
    if (res != BME280_OK)
    {
        printf("Error: failed to initialise sensor with error code %s...\n", bme280_strerr(res));
        free(found_addrs);
        return 1;
    }
    free(found_addrs);

    // for storing formatted readings
    char temp[BME280_T_STRLEN], humid[BME280_H_STRLEN], press[BME280_P_STRLEN];
    printf("Success! Performing forced read...\n");

    for (int i = 0; i < 5; ++i)
    {
        res = bme280_forced_read(&sensor);
        if (res != BME280_OK)
        {
            printf("Error: failed to perform forced read with error code %s...\n", bme280_strerr(res));
            return 1;
        }
        bme280_fmt_temp(&sensor, temp);
        bme280_fmt_humid(&sensor, humid);
        bme280_fmt_press(&sensor, press);
        printf("Result of forced read: Temp: %s - Hum: %s - Pres: %s\n", temp, humid, press);
        sleep_ms(500);
    }

    printf("\nSwitching to normal mode...\n");

    res = bme280_update_settings(&sensor,
        BME280_NORMAL_MODE,
        BME280_FILTER_OFF | BME280_INACTIVE_MS_125,
        BME280_T_OVERSAMPLE_1,
        BME280_H_OVERSAMPLE_1,
        BME280_P_OVERSAMPLE_1
    );
    if (res != BME280_OK)
    {
        printf("Error: failed to update settings with error code %s...\n", bme280_strerr(res));
        return 1;
    }

    for (int i = 0; i < 5; ++i)
    {
        res = bme280_normal_read(&sensor);
        if (res != BME280_OK)
        {
            printf("Error: failed to perform normal read with error code %s...\n", bme280_strerr(res));
            return 1;
        }

        bme280_fmt_temp(&sensor, temp);
        bme280_fmt_humid(&sensor, humid);
        bme280_fmt_press(&sensor, press);
        printf("Result of normal read: Temp: %s - Hum: %s - Pres: %s\n", temp, humid, press);
        sleep_ms(500);
    }

    printf("De-initialising sensor...\n");
    bme280_deinit(&sensor);
    printf("Sucess!\n\nTests run successfully! :)\n");

    return 0;
}
