# Set Minimum Cmake Version
cmake_minimum_required(VERSION 3.12)

# Include Build Functions
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(electronic_load_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${PROJECT_NAME}
    i2c_lcd_lib.c
    ads1015_lib.c
    mcp4725_lib.c
    main.c
)

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Make UF2
pico_add_extra_outputs(${PROJECT_NAME})

# Link to STDLIB
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    pico_multicore
    hardware_gpio
    hardware_pwm
    hardware_i2c
    hardware_rtc
    hardware_adc
)

add_compile_definitions(PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=5000)
