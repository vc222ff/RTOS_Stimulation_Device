# Adds project executables.
add_executable(${ProjectName}
    main.c
)

# Adds standard include files to build.
target_include_directories(${ProjectName} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/btstack/src
)

# Pulls in the dependencies.
target_link_libraries(${ProjectName} 
    hardware_spi
    hardware_gpio
    hardware_i2c
    hardware_timer
    hardware_watchdog
    hardware_clocks
    pico_stdlib
    pico_btstack_ble
    pico_cyw43_arch_lwip_threadsafe_background
    FreeRTOS-Kernel-Heap4
)

if (PICO_CYW43_SUPPORTED)
    target_link_libraries(${ProjectName} pico_cyw43_arch_none)
endif()

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(${ProjectName} 0)
pico_enable_stdio_usb(${ProjectName} 1)

# Creates extra map, bin, hex & etc build files.
pico_add_extra_outputs(${ProjectName})
