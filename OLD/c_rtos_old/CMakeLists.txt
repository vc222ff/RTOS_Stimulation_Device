# Cmake Pico project file
cmake_minimum_required(VERSION 3.13)

# Sets project name
SET(ProjectName c_rtos)

# Sets compiler settings for the RP2040
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.0)
set(toolchainVersion 13_3_Rel1)
set(picotoolVersion 2.1.0)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================

# Sets the board type
set(PICO_BOARD pico_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# Defines the project
project(${ProjectName} C CXX ASM)

# Initialize the Raspberry Pi Pico SDK
pico_sdk_init()

# Add subdirectories
add_subdirectory(src)

pico_set_program_name(${ProjectName} "c_rtos")
pico_set_program_version(${ProjectName} "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${ProjectName} 1)
pico_enable_stdio_usb(${ProjectName} 1)

# create map/bin/hex file etc.
#pico_add_extra_outputs(${ProjectName})




# Set any variables required for importing libraries
#SET(FREERTOS_KERNEL_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/FreeRTOS-Kernel)
#SET(PICO_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/lib/pico-sdk)

# Import SDK and FREERTOS libraries
#include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
#include(${FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)


# Force native GCC for Pioasm (host tool)
#set(PIOASM_C_COMPILER /usr/bin/gcc CACHE FILEPATH "" FORCE)
#set(PIOASM_CXX_COMPILER /usr/bin/g++ CACHE FILEPATH "" FORCE)

# Defines the project
#project(${ProjectName} C CXX ASM)


# Compiler settings for the RP2040
#set(CMAKE_C_STANDARD 11)
#set(CMAKE_CXX_STANDARD 17)

# Set the assembler correctly for ARM
#set(CMAKE_ASM_COMPILER /usr/bin/arm-none-eabi-gcc)

# For embedded code (RP2040)
#if(CMAKE_CROSSCOMPILING)
#    set(CMAKE_C_COMPILER /usr/bin/arm-none-eabi-gcc)
#    set(CMAKE_CXX_COMPILER /usr/bin/arm-none-eabi-g++)
#    set(CMAKE_ASM_COMPILER /usr/bin/arm-none-eabi-gcc)
#    add_compile_options(-mcpu=cortex-m0plus -mthumb)
#    add_link_options(-mcpu=cortex-m0plus -mthumb -nostartfiles -nostdlib -static -lc -lgcc -lnosys)
#else()
    # For host tools like Pioasm
#    set(CMAKE_C_COMPILER /usr/bin/gcc)
#    set(CMAKE_CXX_COMPILER /usr/bin/g++)
#    set(CMAKE_ASM_COMPILER /usr/bin/gcc)  # Native assembler for host tools
#endif()


# Initialize the Raspberry Pi Pico SDK
#pico_sdk_init()

# Sets linker flags to standard C-libraries
#set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lc -lm -lgcc -lnosys")


# Add subdirectories
#add_subdirectory(src)
#add_subdirectory(lib)                                          # If you have anything in "lib" folder then uncomment below. Remember to add a CMakeLists.txt file to the "lib" directory.

# create map/bin/hex file etc.
#pico_add_extra_outputs(${ProjectName})

#Define the target board
#set(PICO_BOARD pico_w)
