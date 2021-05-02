/*
    Divides the input frequency by factor 10^7, 10MHz to 1Hz using PIO.
    System clock has to be driven from 10MHz external clock through GPIO.
    Based on the pio_blink example https://github.com/raspberrypi/pico-examples/tree/master/pio/pio_blink

    Connect 10 MHz 3.3V input signal to GPIO20 (pin 26).
    Output 1 Hz, 50-50 duty cycle signal is generated on GPIO0 (pin 1) 
    and GPIO25 - LED pin, which then starts to blink.

    SPDX-License-Identifier: BSD-3-Clause

    Version:
    02-May-2021  Marek Dorsic (.md)
*/

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "picoDIV.pio.h"


uint input_freq = 10000000; // 10 MHz

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint freq) {
    picodiv_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
//    pio->txf[sm] = 5000000-3;     // for 10MHz to 1Hz division
    pio->txf[sm] = (input_freq/(2*freq))-3;
}

int main() {

    // configure the clk_ref to run from pin GPIO20 (pin 26)
    clock_configure_gpin(clk_ref, 20, input_freq, input_freq);

    // configure the clk_sys, which drives the PIO to run from pin GPIO20 (pin 26)
    clock_configure_gpin(clk_sys, 20, input_freq, input_freq);

    // stop the clk_sys PPL
    pll_deinit(pll_sys);

    // program the PIOs
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &picodiv_program);

    // assign output pins and outout frequency to PIOs
    blink_pin_forever(pio, 0, offset, 0, 1);        // output on GPIO0 (pin 1)
    blink_pin_forever(pio, 1, offset, 25, 1);       // blink also the LED on GPIO25
}
