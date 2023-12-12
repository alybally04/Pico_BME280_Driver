add_executable(pico_example
        example.c
)

target_compile_options(pico_example PRIVATE -O3)

target_include_directories(pico_example PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(pico_example
        Pico_BME280_Driver
        pico_stdlib
        hardware_i2c
)

# Create map/bin/hex/uf2 files
pico_add_extra_outputs(pico_example)

# Enable USB serial
pico_enable_stdio_usb(pico_example 1)
pico_enable_stdio_uart(pico_example 0)
