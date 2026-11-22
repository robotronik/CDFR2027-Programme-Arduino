#pragma once

#include <stdint.h>
#include "pca9685.h"

#define SERVO_MIN_PULSE_US 500
#define SERVO_MAX_PULSE_US 2500
#define SERVO_PWM_PERIOD_US 20000UL

typedef struct
{
    pca9685_t *pca;
    uint8_t channel;
    int min;
    int max;
    int current_angle;
    uint32_t move_time;
    uint32_t move_start;
    int start_angle;
    int target_angle;
    bool moving;
} servo_t;

void servo_init(servo_t *servo, pca9685_t *pca, uint8_t channel, int min, int max);
void servo_target(servo_t *servo, int angle, uint16_t speed);
void servo_run(servo_t *servo);
