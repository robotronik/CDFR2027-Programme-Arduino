#pragma once

#include <Arduino.h>
#include <SoftwareWire.h>

#define PCA9685_CHANNEL_COUNT 16
#define PCA9685_PWM_STEPS 4096UL

typedef struct
{
    SoftwareWire *bus;
    uint8_t address;
} pca9685_t;

bool pca9685_init(pca9685_t *pca, SoftwareWire *bus, uint8_t address, uint16_t frequency);
bool pca9685_set_channel(pca9685_t *pca, uint8_t channel, uint16_t value);
bool pca9685_set_duty(pca9685_t *pca, uint8_t channel, uint8_t duty);
bool pca9685_set_on_off(pca9685_t *pca, uint8_t channel, bool on);
