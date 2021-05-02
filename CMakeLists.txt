cmake_minimum_required(VERSION 3.12)
include(/Users/dorsic/Development/Pico/pico-sdk/external/pico_sdk_import.cmake)

project(picoDIV)
pico_sdk_init()

add_executable(picoDIV picoDIV.c picoDIV.pio)

pico_generate_pio_header(picoDIV ${CMAKE_CURRENT_LIST_DIR}/picoDIV.pio)

target_sources(picoDIV PRIVATE picoDIV.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(picoDIV pico_stdlib
                                hardware_clocks
                                hardware_pio)

# create map/bin/hex file etc.
pico_add_extra_outputs(picoDIV)
