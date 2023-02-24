#include <stdio.h>
// #include <stdlib.h>
#include <unistd.h>
#include "mraa/gpio.h"

// Pin numbers for hub75E panel
#define PIN_A  15
#define PIN_B  16
#define PIN_C  18
#define PIN_D  22
#define PIN_E  10
#define PIN_OE  12
#define PIN_LAT  7
#define PIN_CLK  11
#define PIN_R1  23
#define PIN_G1  13
#define PIN_B1  26
#define PIN_R2  24
#define PIN_G2  21
#define PIN_B2  19

// Set all pins to output mode
void setup_pins() {
    mraa_init();
    mraa_gpio_context gpio_a = mraa_gpio_init(PIN_A);
    mraa_gpio_context gpio_b = mraa_gpio_init(PIN_B);
    mraa_gpio_context gpio_c = mraa_gpio_init(PIN_C);
    mraa_gpio_context gpio_d = mraa_gpio_init(PIN_D);
    mraa_gpio_context gpio_e = mraa_gpio_init(PIN_E);
    mraa_gpio_context gpio_oe = mraa_gpio_init(PIN_OE);
    mraa_gpio_context gpio_lat = mraa_gpio_init(PIN_LAT);
    mraa_gpio_context gpio_clk = mraa_gpio_init(PIN_CLK);
    mraa_gpio_context gpio_r1 = mraa_gpio_init(PIN_R1);
    mraa_gpio_context gpio_g1 = mraa_gpio_init(PIN_G1);
    mraa_gpio_context gpio_b1 = mraa_gpio_init(PIN_B1);
    mraa_gpio_context gpio_r2 = mraa_gpio_init(PIN_R2);
    mraa_gpio_context gpio_g2 = mraa_gpio_init(PIN_G2);
    mraa_gpio_context gpio_b2 = mraa_gpio_init(PIN_B2);

    mraa_gpio_dir(gpio_a, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_b, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_c, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_d, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_e, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_oe, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_lat, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_clk, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_r1, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_g1, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_b1, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_r2, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_g2, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_b2, MRAA_GPIO_OUT);
    printf("output mode set on pins done",'\n');
}
void send_data() {
    // Set all pins to output mode
    // setup_pins();
    mraa_init();
    mraa_gpio_context gpio_a = mraa_gpio_init(PIN_A);
    mraa_gpio_context gpio_b = mraa_gpio_init(PIN_B);
    mraa_gpio_context gpio_c = mraa_gpio_init(PIN_C);
    mraa_gpio_context gpio_d = mraa_gpio_init(PIN_D);
    mraa_gpio_context gpio_e = mraa_gpio_init(PIN_E);
    mraa_gpio_context gpio_oe = mraa_gpio_init(PIN_OE);
    mraa_gpio_context gpio_lat = mraa_gpio_init(PIN_LAT);
    mraa_gpio_context gpio_clk = mraa_gpio_init(PIN_CLK);
    mraa_gpio_context gpio_r1 = mraa_gpio_init(PIN_R1);
    mraa_gpio_context gpio_g1 = mraa_gpio_init(PIN_G1);
    mraa_gpio_context gpio_b1 = mraa_gpio_init(PIN_B1);
    mraa_gpio_context gpio_r2 = mraa_gpio_init(PIN_R2);
    mraa_gpio_context gpio_g2 = mraa_gpio_init(PIN_G2);
    mraa_gpio_context gpio_b2 = mraa_gpio_init(PIN_B2);

    mraa_gpio_dir(gpio_a, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_b, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_c, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_d, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_e, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_oe, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_lat, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_clk, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_r1, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_g1, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_b1, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_r2, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_g2, MRAA_GPIO_OUT);
    mraa_gpio_dir(gpio_b2, MRAA_GPIO_OUT);
    printf("output mode set on pins done",'\n');
    // Set data lines to red (HIGH)
    // mraa_gpio_context gpio_r1 = mraa_gpio_init(PIN_R1);
    mraa_gpio_write(gpio_r1, 1);
    // mraa_gpio_context gpio_g1 = mraa_gpio_init(PIN_G1);
    mraa_gpio_write(gpio_g1, 0);
    // mraa_gpio_context gpio_b1 = mraa_gpio_init(PIN_B1);
    mraa_gpio_write(gpio_b1, 0);
    // mraa_gpio_context gpio_r2 = mraa_gpio_init(PIN_R2);
    mraa_gpio_write(gpio_r2, 1);
    // mraa_gpio_context gpio_g2 = mraa_gpio_init(PIN_G2);
    mraa_gpio_write(gpio_g2, 0);
    // mraa_gpio_context gpio_b2 = mraa_gpio_init(PIN_B2);
    mraa_gpio_write(gpio_b2, 0);
    printf("rbg pins done",'\n');
    // Set control lines to appropriate state
    // mraa_gpio_context gpio_a = mraa_gpio_init(PIN_A);
    // mraa_gpio_context gpio_b = mraa_gpio_init(PIN_B);
    // mraa_gpio_context gpio_c = mraa_gpio_init(PIN_C);
    // mraa_gpio_context gpio_d = mraa_gpio_init(PIN_D);
    // mraa_gpio_context gpio_e = mraa_gpio_init(PIN_E);
    // mraa_gpio_context gpio_oe = mraa_gpio_init(PIN_OE);
    mraa_gpio_write(gpio_oe, 0);
    // mraa_gpio_context gpio_lat = mraa_gpio_init(PIN_LAT);
    mraa_gpio_write(gpio_lat, 0);
    // mraa_gpio_context gpio_clk = mraa_gpio_init(PIN_CLK);
    mraa_gpio_write(gpio_clk, 0);
    mraa_gpio_write(gpio_a, 1);
    mraa_gpio_write(gpio_b, 0);
    mraa_gpio_write(gpio_c, 0);
    mraa_gpio_write(gpio_d, 0);
    mraa_gpio_write(gpio_e, 0);
    printf("row col pins done",'\n');
    // Shift in data and latch to display
    for (int i = 0; i < 4096; i++) {
        if(0< i < 64*2|| 64*10 < i < 64*12){
            mraa_gpio_write(gpio_a, 1);
            mraa_gpio_write(gpio_b, 0);
            mraa_gpio_write(gpio_c, 0);
            mraa_gpio_write(gpio_d, 0);
            mraa_gpio_write(gpio_e, 0);
            if(i == 63*2+1){
                printf("row a");
            }
        }
        if(64*2< i < 64*4 || 64*12 < i < 64*14){
            mraa_gpio_write(gpio_b,1);
            mraa_gpio_write(gpio_a, 0);
            mraa_gpio_write(gpio_c, 0);
            mraa_gpio_write(gpio_d, 0);
            mraa_gpio_write(gpio_e, 0);
            if(i == 64*2-1){
                printf("row b");
            }
        }
        if(64*4< i < 64*6){
            mraa_gpio_write(gpio_c,1);
            mraa_gpio_write(gpio_a, 0);
            mraa_gpio_write(gpio_b, 0);
            mraa_gpio_write(gpio_d, 0);
            mraa_gpio_write(gpio_e, 0);
            if(i == 64*3-1){
                printf("row c");
            }
        }
        if(64*6< i < 64*8){
            mraa_gpio_write(gpio_d,1);
            mraa_gpio_write(gpio_a, 0);
            mraa_gpio_write(gpio_b, 0);
            mraa_gpio_write(gpio_c, 0);
            mraa_gpio_write(gpio_e, 0);
            if(i == 63*4-1){
                printf("row d");
            }
        }
        if(64*8< i < 64*10){
            mraa_gpio_write(gpio_e,1);
            mraa_gpio_write(gpio_a, 0);
            mraa_gpio_write(gpio_b, 0);
            mraa_gpio_write(gpio_d, 0);
            mraa_gpio_write(gpio_c, 0);
            if(i == 63*5-1){
                printf("row d");
            }
        }
        // mraa_gpio_write(gpio_a, 0);

        // mraa_gpio_write(gpio_b, 0);
        // mraa_gpio_write(gpio_c, 0);
        // mraa_gpio_write(gpio_d, 0);
        // mraa_gpio_write(gpio_e, 1);
        // Set data lines to red (HIGH)
        mraa_gpio_write(gpio_r1, 1);
        mraa_gpio_write(gpio_g1, 1);
        mraa_gpio_write(gpio_b1, 0);
        mraa_gpio_write(gpio_r2, 1);
        mraa_gpio_write(gpio_g2, 0);
        mraa_gpio_write(gpio_b2, 0);

        // Pulse clock
        mraa_gpio_write(gpio_clk, 0);
        usleep(10);
        mraa_gpio_write(gpio_clk, 1);
    }

    // Latch data to display
    mraa_gpio_write(gpio_lat, 1);
    usleep(10);
    mraa_gpio_write(gpio_lat, 0);
    printf("latching done",'\n');
    // Set OE to disable output
    mraa_gpio_write(gpio_oe, 1);
    int ar= mraa_gpio_read(gpio_a);
    printf("keeping stable");
    // sleep(100);
    // printf(ar);
    // mraa_gpio_read(gpio_a);
    // mraa_gpio_read(gpio_b);
    // mraa_gpio_read(gpio_c);
    // mraa_gpio_read(gpio_c);
    // mraa_gpio_read(gpio_d);
    // mraa_gpio_read(gpio_e);

}

int main(){
    printf("calling main",'\n');
    send_data();
    return 0;
}