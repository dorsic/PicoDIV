cmake_minimum_required(VERSION 3.12)
include(/Users/dorsic/Development/Pico/pico-sdk/external/pico_sdk_import.cmake)

project(picoDIV_PWM)
pico_sdk_init()

add_executable(picoDIV_PWM picoDIV_PWM.c)

target_sources(picoDIV_PWM PRIVATE picoDIV_PWM.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(picoDIV_PWM pico_stdlib
                                hardware_clocks
                                hardware_pwm
                                hardware_pll)

# create map/bin/hex file etc.
pico_add_extra_outputs(picoDIV_PWM)
