//
// Created by psy on 07-08-21.
//

#include <stdbool.h>
#include <stdint.h>

#ifndef RPI_WS281X_SMI_SMILEDS_H
#define RPI_WS281X_SMI_SMILEDS_H

union color_t {
    uint32_t packed; //packed presentation
    struct  // component-wise representation
    {
        uint8_t b;
        uint8_t r;
        uint8_t g;
        uint8_t a;
    } component ;
};


#ifdef __cplusplus
extern "C"
{
#endif


bool leds_init(int init_led_count);

void leds_set_pixel(uint8_t  channel, uint16_t  pixel,  union color_t   color);

void leds_send();

void leds_clear();

void test();


#ifdef __cplusplus
}
#endif


#endif //RPI_WS281X_SMI_SMILEDS_H

