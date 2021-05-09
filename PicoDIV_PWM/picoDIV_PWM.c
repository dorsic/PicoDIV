/*
    Divides the input frequency by factor 10^7, 10MHz to 1Hz using PWM.
    External reference clock has to be connected to GPIO1 (Pin 2).
    PWM output is at GPIO0 (Pin 1).

    Connect 10 MHz 3.3V input signal to GPIO1 (pin 2).
    Output 1MHz, 50-50 duty cycle signal is generated on GPIO0 (pin 1).

    SPDX-License-Identifier: BSD-3-Clause

    Version:
    09-May-2021  Marek Dorsic (.md)
*/

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/xosc.h"
#include "hardware/pwm.h"


int main() {

/*  UNCOMMENT THIS IF YOU WANT TO USE THE INTERNAL XOSC AS REFERENCE CLOCK 
    CONNECT GPIO21 to GPIO1 (Pin 27 and Pin 2) 
    xosc_init();
    int input_freq = 12000000;  // 12MHz is the internal XOSC
    clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, input_freq, input_freq);
    //output XOSC on pin 21
    clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC ,1);
*/

    // Set GPIO0 pin to PWM output (channel A)
    gpio_set_function(0, GPIO_FUNC_PWM);
    // Set GPIO1 pin to PWM input (channel B)
    gpio_set_function(1, GPIO_FUNC_PWM);
    // Get the PWM slice for the pin
    uint slice_num = pwm_gpio_to_slice_num(0);

    // Enable PWM
    pwm_set_enabled(slice_num, true);
    // Set Channel B be the PWM counter clock
    pwm_set_clkdiv_mode(slice_num, PWM_DIV_B_RISING);
    // Set clock divider (use 200.0 for 10MHz to 1PPS). Max 256.0 (8 integer bit and 4 fractional bit)
    pwm_set_clkdiv_int_frac(slice_num, 1, 0);
    // Set counter maximum value before wrap (max 65535), (use 49999 for 10MHz to 1PPS) 
    pwm_set_wrap(slice_num, 9);
    // Set pulse length in ref clock cycles after division (use 500 for 10ms pulse when 10MHz to 1PPS)
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 5);

    // Use your other Pico Cores as you wish
//    while (1)
//        tight_loop_contents();
    

}
