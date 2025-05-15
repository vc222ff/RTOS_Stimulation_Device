# Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

# Sets project name
SET(ProjectName c_rtos_posture)

# Defines the raspberry board type
set(PICO_BOARD pico2_w)

# Set any variables required for importing libraries
set(PICO_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-sdk)
set(PICO_EXTRAS_PATH ${CMAKE_CURRENT_LIST_DIR}/lib/pico-extras)
SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)

# Import those libraries
include(${PICO_SDK_PATH}/external/pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Sets compiler settings for the RP2040
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define project
project(${ProjectName})

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add codebase subdirectories
add_subdirectory(src)
