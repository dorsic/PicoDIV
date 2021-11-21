/*
    Divides the input frequency by factor 10^7, 10MHz to 1Hz and other frequencies using PIO.
    System clock can be driven from 10MHz external clock through GPIO.
    Based on the pio_blink example https://github.com/raspberrypi/pico-examples/tree/master/pio/pio_blink

    EXTERNAL CLOCK REFERENCE
    Connect 10 MHz 3.3V input signal to GPIO20 (pin 26).
    By default output 1 Hz, 10 ms pulse is at GPIO17 (and GPIO25, led)
    and 100 kHz, 50% duty cycle signal is generated on GPIO18.
    GPIO19 is used to synchronize to the output with external signal.
    Short it to GPIO20 when synchronization not needed.
    GPIO21 outputs unsynchronized signal direcly from RP2040 clock divider.
    Use SYNC_TUNE to align generated output to sync signal.

    Note if system clock under 48MHz, USB is not working.

    SPDX-License-Identifier: BSD-3-Clause

    Version:
    v1.0 07-May-2021  Marek Dorsic (.md)
    v2.0 21-Nov-2021  Marek Dorsic (.md) - added synchronization, pinout change, refactor
*/

#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/divider.h"
#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "hardware/pll.h"
#include "hardware/xosc.h"
#include "picoDIV.pio.h"

#define EXT_CLK                   // comment out to use internal TCXO intead of external clock
#define EXT_CLK_FREQ 10*MHZ       // change to you external clock frequency
#define EXT_CLK_GPIO 20           // CAN NOT BE CHANGED, because of RP2040 gpio usages.
#define OUT_UNSYNC_FREQ_Hz 1      // 1Hz = 1PPS RP2040 divided frequency, unsynchronized
#define OUT_UNSYNC_PPS_GPIO 21    // can be disabled, by commenting out
#define SYNC_GPIO 19              // DO NOT CHANGE unless you modify the WAIT instruction in PIO    
#define SYNC_TUNE 7               // fine align the output PPS to sync PPS, change by 1 is change by 100ns @ 10MHz ExtClk


#define OUT_A_FREQ_Hz 1               // 1 Hz ~ 1 PPS
#define OUT_A_PULSELENGTH_ns 10000000 // 10 ms
#define OUT_A_GPIO 17

#define OUT_B_FREQ_Hz 100000           // 100 kHz
#define OUT_B_PULSELENGTH_ns 5000      // 5us ~ 50% duty cycle
#define OUT_B_GPIO 18


uint32_t sys_freq = 0;

void configure_clocks() {
    #ifdef EXT_CLK
        sys_freq = EXT_CLK_FREQ;
        clock_configure_gpin(clk_ref, EXT_CLK_GPIO, sys_freq, sys_freq);
        clock_configure_gpin(clk_sys, EXT_CLK_GPIO, sys_freq, sys_freq);
        gpio_put(PICO_DEFAULT_LED_PIN, true);
        #ifdef OUT_UNSYNC_PPS_GPIO
            uint32_t div = sys_freq/OUT_UNSYNC_FREQ_Hz;
            clock_gpio_init(OUT_UNSYNC_PPS_GPIO, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_CLKSRC_GPIN0, div);
        #endif
        xosc_disable();
    #else
        sys_freq = XOSC_MHZ*MHZ;  // 12MHz is the internal XOSC
        xosc_init();
        clock_configure(clk_ref, CLOCKS_CLK_REF_CTRL_SRC_VALUE_XOSC_CLKSRC, 0, sys_freq, sys_freq);
        clock_configure(clk_sys, CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLK_REF, CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, sys_freq, sys_freq);
        #ifdef OUT_UNSYNC_PPS_GPIO
            uint32_t div = sys_freq/OUT_UNSYNC_FREQ_Hz;
            clock_gpio_init(OUT_UNSYNC_PPS_GPIO, CLOCKS_CLK_GPOUT0_CTRL_AUXSRC_VALUE_XOSC_CLKSRC, div);
        #endif
    #endif
    // stop the clk_sys PPL
    pll_deinit(pll_sys);
}

void blink_pin_forever(PIO pio, uint sm, uint offset, uint pin, uint32_t total_clk, uint32_t pulse_clk) {
    picodiv_program_init(pio, sm, offset, pin);
    pio_sm_set_enabled(pio, sm, true);
    
    pio->txf[sm] = pulse_clk - 3;                   // write number of HIGH clock cycles to PIO register
    pio->txf[sm] = total_clk - pulse_clk - 3;       // write number off LOW clock cycles to PIO register
    pio->txf[sm] = total_clk - SYNC_TUNE;                   // write the first sync period clock cycles to PIO register
}

void configure_pios() {
    // program the PIOs
    PIO pio = pio0;
    uint offset = pio_add_program(pio, &picodiv_program);

    // assign output pins and outout frequency to PIOs
//    blink_pin_forever(pio, 2, offset, OUT_B_GPIO, sys_freq/OUT_B_FREQ, OUT_B_PULSELENGTH_ns*sys_freq/1e9);                  

    uint32_t tot_clk = div_u32u32(sys_freq, (uint32_t) OUT_A_FREQ_Hz);
    uint32_t nomin = (uint32_t)OUT_A_PULSELENGTH_ns * (sys_freq/MHZ);
    uint32_t pls_clk = div_u32u32(nomin, (uint32_t)1000);
    blink_pin_forever(pio, 0, offset, OUT_A_GPIO, tot_clk, pls_clk);
    blink_pin_forever(pio, 1, offset, PICO_DEFAULT_LED_PIN, tot_clk, pls_clk);  // blink the LED with out A freq

    tot_clk = div_u32u32(sys_freq, (uint32_t) OUT_B_FREQ_Hz);
    nomin = (uint32_t)OUT_B_PULSELENGTH_ns * (sys_freq/MHZ);
    pls_clk = div_u32u32(nomin, (uint32_t)1000);
    blink_pin_forever(pio, 2, offset, OUT_B_GPIO, tot_clk, pls_clk);

//    blink_pin_forever(pio, 0, offset, OUT_A_GPIO, 10*MHZ, 10000);
//    blink_pin_forever(pio, 1, offset, PICO_DEFAULT_LED_PIN, 10*MHZ, 10000);
//    blink_pin_forever(pio, 2, offset, OUT_B_GPIO, 100, 50); 

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
    uint value;
    configure_clocks();
    configure_pios();

    // initialize UART
    stdio_init_all();

    while (true) {
        value = read_long("\nPlease enter pulse slide delay [ref_clock_cycles]: ");
        for (int i=0; i<value; i++)
            pio_sm_exec_wait_blocking(pio0, 0, pio_encode_nop());
            pio_sm_exec_wait_blocking(pio0, 1, pio_encode_nop());
            pio_sm_exec_wait_blocking(pio0, 2, pio_encode_nop());
            printf("%i\n");
    }
    
}
