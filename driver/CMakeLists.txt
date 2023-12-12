add_library(Pico_BME280_Driver STATIC "bme280_driver.c")

target_compile_options(Pico_BME280_Driver PRIVATE -O3)

target_link_libraries(Pico_BME280_Driver
        hardware_i2c
)

target_include_directories(Pico_BME280_Driver PUBLIC
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>"
        "$<INSTALL_INTERFACE:include>"
)
