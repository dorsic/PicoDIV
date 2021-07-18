/*
    Divides the input frequency by factor 10^7, 10MHz to 1Hz using PIO.
    System clock has to be driven from 10MHz external clock through GPIO.
    Based on the pio_blink example https://github.com/raspberrypi/pico-examples/tree/master/pio/pio_blink

    Connect 10 MHz 3.3V input signal to GPIO20 (pin 26).
    Output 1 Hz, 50-50 duty cycle signal is generated on GPIO0 (pin 1) 
    and GPIO25 - LED pin, which then starts to blink.

    SPDX-License-Identifier: BSD-3-Clause

    Version:
    07-May-2021  Marek Dorsic (.md)
*/

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "picoDIV.pio.h"
#include "hardware/xosc.h"

#define INTERNAL_XOSC 1

uint input_freq = 10000000; // 10 MHz
uint pulse_len = 100000;    // number of clock cycles of the output pulse length
                            // 100000 cycles * 100ns (for 10MHz) = 10ms

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint total_clk, uint pulse_clk) {
    picodiv_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
    pio->txf[sm] = pulse_clk - 3;                   // write number of HIGH clock cycles to PIO register
    pio->txf[sm] =  total_clk - pulse_clk - 3;      // write number off LOW clock cycles to PIO register
}

void configure_clocks() {
    if (true) {
        xosc_init();
        input_freq = 12000000;  // 12MHz is the internal XOSC
        pulse_len = 120000;     // 10ms pulse
        clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, input_freq, input_freq);
        clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, input_freq, input_freq);
        clock_gpio_init(21, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, 1);
        
    } else {
    // configure the clk_ref to run from pin GPIO20 (pin 26)
        clock_configure_gpin(clk_ref, 20, input_freq, input_freq);
        clock_configure_gpin(clk_sys, 20, input_freq, input_freq);
        clock_gpio_init(21, CLOCKS_CLK_GPOUT1_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0, 1);

    }
    // stop the clk_sys PPL
    pll_deinit(pll_sys);
    set_sys_clock_khz(input_freq/1000, true);

}

void configure_pios() {
    // program the PIOs
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &picodiv_program);

    // assign output pins and outout frequency to PIOs
    blink_pin_forever(pio, 0, offset, 18, input_freq, pulse_len);    // 1 PPS output on GPIO0 (pin 1)
    blink_pin_forever(pio, 1, offset, 25, input_freq, pulse_len);   // blink the LED on GPIO25
    blink_pin_forever(pio, 2, offset, 22, 10, 5);                   // 1 MHz output on GPIO21 (pin 27) 50/50 square
    blink_pin_forever(pio, 3, offset, 19, 100, 50);                 // 100 kHz output on GPIO15 (pin 20) 50/50 square
}

unsigned char str[32];

char* read_line() {
    unsigned char u, *p;
    for (p = str, u=getchar(); u!='\r' && p-str<31; u=getchar()) putchar(*p++=u);
    *p = 0;
    return str;
}

long read_long(char* message) {
    char *string;
    unsigned int value = -1;

    printf(message);
    value = atoi(read_line());
    printf("\nGot value: %d\n", value);
    return value;
}

int main() {
    unsigned int value;
    configure_clocks();
    configure_pios();

    // initialize USB
    stdio_init_all();
    //while (!tud_cdc_connected()) { sleep_ms(100);  }
    printf("tud_cdc_connected()\n");

    while (true) {
        value = read_long("\nPlease enter pulse slide delay [ns]: ");
        for (int i=0; i<value; i++)
            pio_sm_exec_wait_blocking(pio0, 0, pio_encode_nop());
            pio_sm_exec_wait_blocking(pio0, 1, pio_encode_nop());
            pio_sm_exec_wait_blocking(pio0, 2, pio_encode_nop());
            pio_sm_exec_wait_blocking(pio0, 3, pio_encode_nop());
    }
    
}
