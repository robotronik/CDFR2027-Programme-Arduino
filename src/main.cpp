#include <Arduino.h>
#include <SoftwareWire.h>
#include <Wire.h>
#include "config.h"
#include "utils.h"
#include "pca9685.h"
#include "servo.h"
#include "rgb_led.h"
#include "lidar.h"
#include "common/protocol.h"

// #define SERIAL_DEBUG

static SoftwareWire pca_bus(PIN_PCA_I2C_SDA, PIN_PCA_I2C_SCL);
static pca9685_t pca1;
static pca9685_t pca2;
static rgb_led_t led;
static servo_t servos[SERVO_COUNT];

static const int sensor_pins[SENSOR_COUNT] = {
    PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4,
    PIN_SENSOR_5, PIN_SENSOR_6, PIN_SENSOR_7, PIN_SENSOR_8};

static const uint8_t pump_channels[PUMP_COUNT] = {
    PCA_CH_PUMP_1, PCA_CH_PUMP_2, PCA_CH_PUMP_3,
    PCA_CH_PUMP_4, PCA_CH_PUMP_5, PCA_CH_PUMP_6};

static const uint8_t ev_channels[EV_COUNT] = {
    PCA_CH_EV_1, PCA_CH_EV_2, PCA_CH_EV_3, PCA_CH_EV_4, PCA_CH_EV_5,
    PCA_CH_EV_6, PCA_CH_EV_7, PCA_CH_EV_8, PCA_CH_EV_9, PCA_CH_EV_10};

static uint8_t receive_buffer[BUFFERONRECEIVESIZE];
static uint8_t response_buffer[BUFFERONRECEIVESIZE];
static uint8_t response_size = 0;

// The receive interrupt cannot use the software I2C bus, so it only records the
// newest wanted state of each output and loop() applies what changed.
// Unlike a queue, a burst of commands cannot overflow it.
static volatile uint16_t pca2_wanted_on; // bit n: PCA #2 channel n on
static volatile uint16_t pca2_changed;
static volatile uint16_t servo_wanted_target[SERVO_COUNT];
static volatile uint16_t servo_wanted_speed[SERVO_COUNT];
static volatile uint8_t servo_changed; // bit n: servo n + 1
static volatile uint8_t ext_pwm_wanted[2]; // EXTPWM10, EXTPWM11
static volatile uint8_t ext_pwm_changed;

static void on_receive(int num_bytes);
static void on_request(void);
static void apply_changes(void);
static void want_pca2(uint8_t channel, bool on);

void setup(void)
{
#ifdef SERIAL_DEBUG
  Serial.begin(115200);
  Serial.println("Starting !");
#endif

  pinMode(PIN_OUTPUT_ENABLE, OUTPUT);
  digitalWrite(PIN_OUTPUT_ENABLE, HIGH);
  pinMode(PIN_POW_EN, OUTPUT);
  digitalWrite(PIN_POW_EN, LOW);
  lidar_init();

  for (uint8_t i = 0; i < SENSOR_COUNT; i++)
    pinMode(sensor_pins[i], INPUT_PULLUP);

  pca_bus.begin();
  pca_bus.setTimeout(10);
  bool pca1_ok = pca9685_init(&pca1, &pca_bus, PCA9685_1_ADDRESS, 50);
  bool pca2_ok = pca9685_init(&pca2, &pca_bus, PCA9685_2_ADDRESS, 50);

#ifdef SERIAL_DEBUG
  Serial.print("PCA9685 #1: ");
  Serial.println(pca1_ok ? "ok" : "not found");
  Serial.print("PCA9685 #2: ");
  Serial.println(pca2_ok ? "ok" : "not found");
#else
  (void)pca1_ok;
  (void)pca2_ok;
#endif

  for (uint8_t channel = 0; channel < PCA9685_CHANNEL_COUNT; channel++)
  {
    pca9685_set_channel(&pca1, channel, 0);
    pca9685_set_channel(&pca2, channel, 0);
  }
  rgb_led_init(&led, &pca1, PCA_CH_LED_R, PCA_CH_LED_G, PCA_CH_LED_B);

  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    servo_init(&servos[i], &pca1, PCA_CH_SERVO_1 + i, 0, 180);
  for (uint8_t i = 0; i < SERVO_COUNT - 1; i++)
    servo_target(&servos[i], 90, 0);
  servo_target(&servos[SERVO_COUNT - 1], 150, 0);

  digitalWrite(PIN_OUTPUT_ENABLE, LOW);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(on_receive);
  Wire.onRequest(on_request);
}

void loop(void)
{
  apply_changes();

  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    servo_run(&servos[i]);
  rgb_led_run(&led);
}

static void on_receive(int num_bytes)
{
  if (num_bytes > BUFFERONRECEIVESIZE)
    num_bytes = BUFFERONRECEIVESIZE;
  // Read before any early return: Wire ignores every later message while bytes are left unread.
  Wire.readBytes(receive_buffer, num_bytes);
  if (num_bytes < 2)
    return;

  uint8_t *ptr = receive_buffer;
  uint8_t command = ReadUInt8(&ptr);
  uint8_t number = ReadUInt8(&ptr);
  uint8_t *resp = response_buffer;

  switch (command)
  {
  case CMD_POWER_SERVOS:
    digitalWrite(PIN_POW_EN, number == 1 ? HIGH : LOW);
    break;

  case CMD_MOVE_SERVO:
    if (number >= 1 && number <= SERVO_COUNT && num_bytes >= 6)
    {
      servo_wanted_target[number - 1] = ReadUInt16(&ptr);
      servo_wanted_speed[number - 1] = ReadUInt16(&ptr);
      bitSet(servo_changed, number - 1);
    }
    break;

  case CMD_RGB_LED:
    // Rainbow needs only the mode byte, the other modes also need r, g, b.
    if (number == 1 && num_bytes >= (receive_buffer[2] == LED_MODE_RAINBOW ? 3 : 6))
      rgb_led_receive(&led, &ptr);
    break;

  case CMD_SET_PWM_LIDAR:
    lidar_set_duty(number);
    break;

  case CMD_SET_PUMP:
    if (number >= 1 && number <= PUMP_COUNT && num_bytes >= 3)
      want_pca2(pump_channels[number - 1], ReadUInt8(&ptr));
    break;

  case CMD_SET_EV:
    if (number >= 1 && number <= EV_COUNT && num_bytes >= 3)
      want_pca2(ev_channels[number - 1], ReadUInt8(&ptr));
    break;

  case CMD_SET_PWM_EXT:
    if ((number == 10 || number == 11) && num_bytes >= 3)
    {
      ext_pwm_wanted[number - 10] = ReadUInt8(&ptr);
      bitSet(ext_pwm_changed, number - 10);
    }
    break;

  case CMD_GET_VERSION:
    WriteUInt8(&resp, API_VERSION);
    break;

  case CMD_GET_SERVO:
    if (number < 1 || number > SERVO_COUNT)
      break;
    WriteInt16(&resp, servos[number - 1].current_angle);
    break;

  case CMD_GET_STEPPER:
    WriteInt32(&resp, 0);
    break;

  case CMD_READ_SENSOR:
    if (number < 1 || number > SENSOR_COUNT)
      break;
    WriteUInt8(&resp, !digitalRead(sensor_pins[number - 1]));
    break;

  // Stepper and DC motor commands are kept for API compatibility.
  case CMD_ENABLE_STEPPER:
  case CMD_DISABLE_STEPPER:
  case CMD_MOVE_STEPPER:
  case CMD_SET_STEPPER:
  case CMD_SET_STEPPER_SPEED:
  case CMD_MOVE_DC_MOTOR:
  case CMD_STOP_DC_MOTOR:
    break;

  default:
    break;
  }

  // A command without a reply must not drop a reply the master has not read yet.
  if (resp != response_buffer)
    response_size = resp - response_buffer;
}

static void on_request(void)
{
  Wire.write(response_buffer, response_size);
  response_size = 0;
}

static void want_pca2(uint8_t channel, bool on)
{
  bitWrite(pca2_wanted_on, channel, on);
  bitSet(pca2_changed, channel);
}

static void apply_changes(void)
{
  noInterrupts();
  uint16_t pca2_on = pca2_wanted_on;
  uint16_t pca2_todo = pca2_changed;
  uint8_t servo_todo = servo_changed;
  uint8_t ext_pwm_todo = ext_pwm_changed;
  pca2_changed = 0;
  servo_changed = 0;
  ext_pwm_changed = 0;
  interrupts();

  for (uint8_t channel = 0; channel < PCA9685_CHANNEL_COUNT; channel++)
    if (bitRead(pca2_todo, channel))
      pca9685_set_on_off(&pca2, channel, bitRead(pca2_on, channel));

  for (uint8_t i = 0; i < SERVO_COUNT; i++)
  {
    if (!bitRead(servo_todo, i))
      continue;
    noInterrupts();
    uint16_t target = servo_wanted_target[i];
    uint16_t speed = servo_wanted_speed[i];
    interrupts();
    servo_target(&servos[i], target, speed);
  }

  if (bitRead(ext_pwm_todo, 0))
    pca9685_set_duty(&pca1, PCA_CH_EXT_PWM_10, ext_pwm_wanted[0]);
  if (bitRead(ext_pwm_todo, 1))
    pca9685_set_duty(&pca1, PCA_CH_EXT_PWM_11, ext_pwm_wanted[1]);
}
