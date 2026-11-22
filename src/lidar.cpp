#include "lidar.h"
#include <Arduino.h>
#include "config.h"

// Timer1 fast PWM, prescaler 1: 16 MHz / 800 = 20 kHz. OC1A drives D9.
#define LIDAR_PERIOD_TICKS 800

void lidar_init(void)
{
    pinMode(PIN_PWM_LIDAR, OUTPUT);
    digitalWrite(PIN_PWM_LIDAR, LOW);

    TCCR1A = _BV(WGM11); // fast PWM, TOP = ICR1, output disabled until used
    TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10);
    TCNT1 = 0;
    ICR1 = LIDAR_PERIOD_TICKS - 1;
    OCR1A = 0;
}

void lidar_set_duty(uint8_t duty)
{
    if (duty == 0)
    {
        TCCR1A &= ~_BV(COM1A1);
        digitalWrite(PIN_PWM_LIDAR, LOW);
        return;
    }

    TCCR1A |= _BV(COM1A1);
    // OCR1A above TOP means a permanently high output.
    OCR1A = duty == 255 ? LIDAR_PERIOD_TICKS
                        : (uint16_t)duty * LIDAR_PERIOD_TICKS / 255;
}
