#pragma once

#include <stdint.h>
#include "pca9685.h"

#define LED_MODE_SOLID 0
#define LED_MODE_BLINK 1
#define LED_MODE_RAINBOW 2

typedef struct
{
    pca9685_t *pca;
    uint8_t channel_r;
    uint8_t channel_g;
    uint8_t channel_b;
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t set_r;
    uint8_t set_g;
    uint8_t set_b;
    uint8_t mode;
    uint8_t applied_r;
    uint8_t applied_g;
    uint8_t applied_b;
    bool applied;
} rgb_led_t;

void rgb_led_init(rgb_led_t *led, pca9685_t *pca, uint8_t channel_r, uint8_t channel_g, uint8_t channel_b);
void rgb_led_run(rgb_led_t *led);
void rgb_led_receive(rgb_led_t *led, uint8_t **data);
