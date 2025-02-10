# Adds project executables
add_executable(${ProjectName}
    main.c
)

# Adds standard include files to build
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

# Pulls in the dependencies
target_link_libraries(${ProjectName} 
    hardware_spi
    hardware_gpio
    hardware_i2c
    hardware_timer
    hardware_watchdog
    hardware_clocks
    pico_stdlib 
    FreeRTOS-Kernel-Heap4 
    )
if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${ProjectName} pico_cyw43_arch_none)
endif()

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${ProjectName} 1)
pico_enable_stdio_usb(${ProjectName} 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(${ProjectName})
