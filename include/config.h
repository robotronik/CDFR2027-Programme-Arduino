#pragma once

#include "Arduino.h"

#define BUFFERONRECEIVESIZE 32

// Hardware I2C to the main computer
#define PIN_MASTER_I2C_SDA 18 // A4
#define PIN_MASTER_I2C_SCL 19 // A5

// Software I2C to the PCA9685 boards
#define PIN_PCA_I2C_SDA 2
#define PIN_PCA_I2C_SCL 3

#define PCA9685_1_ADDRESS 0x41
#define PCA9685_2_ADDRESS 0x42

#define PIN_OUTPUT_ENABLE 7 // ~OE, active low
#define PIN_POW_EN 15       // A1, buck converters enable
#define PIN_PWM_LIDAR 9 // D9 / OC1A, hardware PWM

#define PIN_SENSOR_1 12
#define PIN_SENSOR_2 11
#define PIN_SENSOR_3 10
#define PIN_SENSOR_4 6
#define PIN_SENSOR_5 8
#define PIN_SENSOR_6 5
#define PIN_SENSOR_7 4
#define PIN_SENSOR_8 16 // A2

// PCA9685 #1
#define PCA_CH_LED_R 0
#define PCA_CH_LED_G 1
#define PCA_CH_LED_B 2
#define PCA_CH_SERVO_1 3
#define PCA_CH_SERVO_2 4
#define PCA_CH_SERVO_3 5
#define PCA_CH_SERVO_4 6
#define PCA_CH_SERVO_5 7
#define PCA_CH_SERVO_6 8
#define PCA_CH_SERVO_7 9
#define PCA_CH_EXT_PWM_10 10
#define PCA_CH_EXT_PWM_11 11

// PCA9685 #2
#define PCA_CH_PUMP_1 0
#define PCA_CH_PUMP_2 1
#define PCA_CH_PUMP_3 2
#define PCA_CH_PUMP_4 3
#define PCA_CH_PUMP_5 4
#define PCA_CH_PUMP_6 5
#define PCA_CH_EV_1 6
#define PCA_CH_EV_2 7
#define PCA_CH_EV_3 8
#define PCA_CH_EV_4 9
#define PCA_CH_EV_5 10
#define PCA_CH_EV_6 11
#define PCA_CH_EV_7 12
#define PCA_CH_EV_8 13
#define PCA_CH_EV_9 14
#define PCA_CH_EV_10 15

#define SERVO_COUNT 7
#define PUMP_COUNT 6
#define EV_COUNT 10
#define SENSOR_COUNT 8
