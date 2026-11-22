#include "servo.h"

static void servo_write(servo_t *servo, int angle)
{
    noInterrupts();
    servo->current_angle = angle;
    interrupts();
    uint32_t pulse_us = SERVO_MIN_PULSE_US +
                        (uint32_t)angle * (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) / 180;
    pca9685_set_channel(servo->pca, servo->channel,
                        pulse_us * PCA9685_PWM_STEPS / SERVO_PWM_PERIOD_US);
}

void servo_init(servo_t *servo, pca9685_t *pca, uint8_t channel, int min, int max)
{
    servo->pca = pca;
    servo->channel = channel;
    servo->min = min;
    servo->max = max;
    servo->current_angle = -1;
    servo->moving = false;
}

// speed in deg/s, 0 for an immediate move
void servo_target(servo_t *servo, int angle, uint16_t speed)
{
    if (angle < servo->min)
        angle = servo->min;
    else if (angle > servo->max)
        angle = servo->max;

    if (speed == 0 || servo->current_angle == -1)
    {
        servo->moving = false;
        servo_write(servo, angle);
        return;
    }

    servo->move_start = millis();
    servo->start_angle = servo->current_angle;
    servo->target_angle = angle;
    servo->move_time = (uint32_t)abs(angle - servo->start_angle) * 1000 / speed;
    servo->moving = true;
    servo_run(servo);
}

void servo_run(servo_t *servo)
{
    if (!servo->moving)
        return;

    uint32_t progress = millis() - servo->move_start;
    if (progress >= servo->move_time)
    {
        servo->moving = false;
        servo_write(servo, servo->target_angle);
        return;
    }

    int angle = servo->start_angle +
                (int32_t)(servo->target_angle - servo->start_angle) * progress / servo->move_time;
    if (angle != servo->current_angle)
        servo_write(servo, angle);
}
