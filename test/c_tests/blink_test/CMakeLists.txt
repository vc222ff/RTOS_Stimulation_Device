cmake_minimum_required(VERSION 3.13)

# Sets project name
SET(ProjectName blink_test)

# Defines the raspberry board type
set(PICO_BOARD pico2_w)

# Set Pico SDK path
set(PICO_SDK_PATH "/home/victor/Programming/2DT00E/rtos-stimulation-device-for-posture-correction/lib/pico-sdk")
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)


# Sets compiler settings for the RP2040
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)


# Define project
project(${ProjectName})

# Initialize SDK
pico_sdk_init()

# Add executable
add_executable(${ProjectName} main.c)
target_link_libraries(${ProjectName} pico_stdlib)

# Enables or disables terminal output over UART or serial USB.
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# Creates extra map, bin, hex & etc build files.
pico_add_extra_outputs(${ProjectName})
