cmake_minimum_required(VERSION 3.13)
include(pico-sdk/pico_sdk_init.cmake)

project(usb_test)
pico_sdk_init()

add_executable(usb_test usb_test.c)

target_link_libraries(usb_test
    pico_stdlib
    pico_stdio_usb
)

pico_enable_stdio_usb(usb_test 1)
pico_enable_stdio_uart(usb_test 0)
pico_add_extra_outputs(usb_test)