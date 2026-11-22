#include "rgb_led.h"
#include "utils.h"
#include <Arduino.h>

void rgb_led_init(rgb_led_t *led, pca9685_t *pca, uint8_t channel_r, uint8_t channel_g, uint8_t channel_b)
{
    led->pca = pca;
    led->channel_r = channel_r;
    led->channel_g = channel_g;
    led->channel_b = channel_b;
    led->mode = LED_MODE_SOLID;
    led->applied = false;
}

static void rgb_led_apply(rgb_led_t *led, uint8_t r, uint8_t g, uint8_t b)
{
    if (led->applied && r == led->applied_r && g == led->applied_g && b == led->applied_b)
        return;

    // Common-anode LED: the PCA output sinks current, so a higher duty means less light.
    pca9685_set_duty(led->pca, led->channel_r, 255 - r);
    pca9685_set_duty(led->pca, led->channel_g, 255 - g);
    pca9685_set_duty(led->pca, led->channel_b, 255 - b);
    led->applied_r = r;
    led->applied_g = g;
    led->applied_b = b;
    led->applied = true;
}

void rgb_led_receive(rgb_led_t *led, uint8_t **data)
{
    uint8_t mode = ReadUInt8(data);
    led->mode = mode;

    if (mode == LED_MODE_RAINBOW)
        return;

    uint8_t r = ReadUInt8(data);
    uint8_t g = ReadUInt8(data);
    uint8_t b = ReadUInt8(data);
    led->set_r = r;
    led->set_g = g;
    led->set_b = b;
}

#define LED_RAINBOW_PERIOD_MS 3000UL
#define LED_RAINBOW_STEPS_PER_SEGMENT 32

static void rgb_led_rainbow(rgb_led_t *led)
{
    uint16_t step = (millis() % LED_RAINBOW_PERIOD_MS) *
                    (6 * LED_RAINBOW_STEPS_PER_SEGMENT) / LED_RAINBOW_PERIOD_MS;
    uint8_t segment = step / LED_RAINBOW_STEPS_PER_SEGMENT;
    uint8_t f = (step % LED_RAINBOW_STEPS_PER_SEGMENT) * 255 /
                (LED_RAINBOW_STEPS_PER_SEGMENT - 1);

    switch (segment)
    {
    case 0:
        led->r = 255;
        led->g = f;
        led->b = 0;
        break;
    case 1:
        led->r = 255 - f;
        led->g = 255;
        led->b = 0;
        break;
    case 2:
        led->r = 0;
        led->g = 255;
        led->b = f;
        break;
    case 3:
        led->r = 0;
        led->g = 255 - f;
        led->b = 255;
        break;
    case 4:
        led->r = f;
        led->g = 0;
        led->b = 255;
        break;
    default:
        led->r = 255;
        led->g = 0;
        led->b = 255 - f;
        break;
    }
}

void rgb_led_run(rgb_led_t *led)
{
    switch (led->mode)
    {
    case LED_MODE_BLINK:
        if (millis() % 500 < 300)
        {
            led->r = 0;
            led->g = 0;
            led->b = 0;
        }
        else
        {
            led->r = led->set_r;
            led->g = led->set_g;
            led->b = led->set_b;
        }
        break;
    case LED_MODE_RAINBOW:
        rgb_led_rainbow(led);
        break;
    default:
        led->r = led->set_r;
        led->g = led->set_g;
        led->b = led->set_b;
        break;
    }

    rgb_led_apply(led, led->r, led->g, led->b);
}
