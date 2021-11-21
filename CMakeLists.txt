cmake_minimum_required(VERSION 3.13)
include(/Users/dorsic/Development/Pico/pico-sdk/external/pico_sdk_import.cmake)

project(picoDIV)
pico_sdk_init()

add_executable(picoDIV picoDIV.c picoDIV.pio)

pico_generate_pio_header(picoDIV ${CMAKE_CURRENT_LIST_DIR}/picoDIV.pio)

target_sources(picoDIV PRIVATE picoDIV.c)

target_link_libraries(picoDIV
    pico_stdlib
    hardware_clocks
    hardware_pio
    hardware_pll
    hardware_xosc
)

pico_enable_stdio_usb(picoDIV 0)
pico_enable_stdio_uart(picoDIV 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(picoDIV)
