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

#define JOB_QUEUE_SIZE 8

typedef struct
{
  uint8_t command;
  uint8_t number;
  uint8_t data[4];
} job_t;

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

// Commands that need the software I2C bus are queued by the receive
// interrupt and executed from loop().
static job_t job_queue[JOB_QUEUE_SIZE];
static volatile uint8_t job_head = 0;
static volatile uint8_t job_tail = 0;

static void on_receive(int num_bytes);
static void on_request(void);
static void process_jobs(void);
static void job_push(uint8_t command, uint8_t number, uint8_t **data, uint8_t length);
static void set_pump(uint8_t number, bool on);
static void set_ev(uint8_t number, bool on);

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
  process_jobs();

  for (uint8_t i = 0; i < SERVO_COUNT; i++)
    servo_run(&servos[i]);
  rgb_led_run(&led);
}

static void on_receive(int num_bytes)
{
  if (!Wire.available())
    return;

  if (num_bytes > BUFFERONRECEIVESIZE)
    num_bytes = BUFFERONRECEIVESIZE;
  if (num_bytes < 2)
    return;

  Wire.readBytes(receive_buffer, num_bytes);

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
    if (number >= 1 && number <= SERVO_COUNT)
      job_push(command, number, &ptr, 4);
    break;

  case CMD_RGB_LED:
    if (number == 1)
      rgb_led_receive(&led, &ptr);
    break;

  case CMD_SET_PWM_LIDAR:
    lidar_set_duty(number);
    break;

  case CMD_SET_PUMP:
  case CMD_SET_EV:
    job_push(command, number, &ptr, 1);
    break;

  case CMD_SET_PWM_EXT:
    if (number == 10 || number == 11)
      job_push(command, number, &ptr, 1);
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

  response_size += resp - response_buffer;
}

static void on_request(void)
{
  Wire.write(response_buffer, response_size);
  response_size = 0;
}

static void job_push(uint8_t command, uint8_t number, uint8_t **data, uint8_t length)
{
  uint8_t next = (job_head + 1) % JOB_QUEUE_SIZE;
  if (next == job_tail)
    return;

  job_queue[job_head].command = command;
  job_queue[job_head].number = number;
  for (uint8_t i = 0; i < length && i < 4; i++)
    job_queue[job_head].data[i] = ReadUInt8(data);
  job_head = next;
}

static void process_jobs(void)
{
  while (job_tail != job_head)
  {
    job_t job = job_queue[job_tail];
    job_tail = (job_tail + 1) % JOB_QUEUE_SIZE;

    switch (job.command)
    {
    case CMD_MOVE_SERVO:
    {
      uint16_t target = job.data[0] | (job.data[1] << 8);
      uint16_t speed = job.data[2] | (job.data[3] << 8);
      servo_target(&servos[job.number - 1], target, speed);
      break;
    }
    case CMD_SET_PUMP:
      set_pump(job.number, job.data[0] != 0);
      break;
    case CMD_SET_EV:
      set_ev(job.number, job.data[0] != 0);
      break;
    case CMD_SET_PWM_EXT:
    {
      uint8_t channel = job.number == 10 ? PCA_CH_EXT_PWM_10 : PCA_CH_EXT_PWM_11;
      pca9685_set_duty(&pca1, channel, job.data[0]);
      break;
    }
    default:
      break;
    }
  }
}

static void set_pump(uint8_t number, bool on)
{
  if (number < 1 || number > PUMP_COUNT)
    return;
  pca9685_set_on_off(&pca2, pump_channels[number - 1], on);
}

static void set_ev(uint8_t number, bool on)
{
  if (number < 1 || number > EV_COUNT)
    return;
  pca9685_set_on_off(&pca2, ev_channels[number - 1], on);
}
