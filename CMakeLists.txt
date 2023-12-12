cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD pico_w)
include(pico-sdk/pico_sdk_init.cmake)

project(Pico_BME280_Driver)

pico_sdk_init()

add_subdirectory(driver)
add_subdirectory(example)
