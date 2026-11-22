#pragma once

#define API_VERSION 0x04
#define API_NAME "Arduino API"

#define I2C_ADDRESS 0x64

#define CMD_GET_VERSION 0x01

#define CMD_POWER_SERVOS 0x11
#define CMD_MOVE_SERVO 0x12
#define CMD_GET_SERVO 0x13

#define CMD_READ_SENSOR 0x21

// Unused on this board (no stepper motors)
#define CMD_ENABLE_STEPPER 0x31
#define CMD_DISABLE_STEPPER 0x32
#define CMD_MOVE_STEPPER 0x33
#define CMD_SET_STEPPER 0x34
#define CMD_GET_STEPPER 0x35
#define CMD_SET_STEPPER_SPEED 0x36

#define CMD_RGB_LED 0x41

#define CMD_SET_PWM_LIDAR 0x51

// Unused on this board (no DC motor)
#define CMD_MOVE_DC_MOTOR 0x61
#define CMD_STOP_DC_MOTOR 0x62

#define CMD_SET_PUMP 0x71
#define CMD_SET_EV 0x72
#define CMD_SET_PWM_EXT 0x73
