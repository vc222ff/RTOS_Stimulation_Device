# Cmake Pico project file.
cmake_minimum_required(VERSION 3.13)

# Sets project name.
SET(ProjectName usb_test)

# Set any variables required for importing libraries.
set(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico-sdk)

# Import those libraries.
include(pico_sdk_import.cmake)

# Define project.
project(${ProjectName})

# Initialize the Raspberry Pi Pico SDK.
pico_sdk_init()

# Adds project executables.
add_executable(${ProjectName} usb_test.c)

# Pulls in the dependencies.
target_link_libraries(${ProjectName}
    pico_stdlib
    pico_stdio_usb
)

# Enables or disables terminal output over UART or serial USB.
pico_enable_stdio_usb(${ProjectName} 1)
pico_enable_stdio_uart(${ProjectName} 0)

# Creates extra map, bin, hex & etc build files.
pico_add_extra_outputs(usb_test)
