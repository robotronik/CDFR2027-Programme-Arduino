#include "pca9685.h"

#define REG_MODE1 0x00
#define REG_MODE2 0x01
#define REG_LED0_ON_L 0x06
#define REG_PRESCALE 0xFE

#define MODE1_RESTART 0x80
#define MODE1_AI 0x20
#define MODE1_SLEEP 0x10
#define MODE2_OUTDRV 0x04
#define LED_FULL_ON 0x10

#define OSCILLATOR_HZ 25000000UL

static bool write_registers(pca9685_t *pca, uint8_t reg, const uint8_t *values, uint8_t count)
{
    pca->bus->beginTransmission(pca->address);
    pca->bus->write(reg);
    for (uint8_t i = 0; i < count; i++)
        pca->bus->write(values[i]);
    return pca->bus->endTransmission() == 0;
}

static bool write_register(pca9685_t *pca, uint8_t reg, uint8_t value)
{
    return write_registers(pca, reg, &value, 1);
}

bool pca9685_init(pca9685_t *pca, SoftwareWire *bus, uint8_t address, uint16_t frequency)
{
    pca->bus = bus;
    pca->address = address;

    if (frequency == 0)
        return false;
    if (!write_register(pca, REG_MODE2, MODE2_OUTDRV))
        return false;

    uint16_t prescale = OSCILLATOR_HZ / (PCA9685_PWM_STEPS * frequency);
    if (prescale > 0)
        prescale -= 1;
    if (prescale > 255)
        prescale = 255;

    // The prescaler can only be written while the oscillator is asleep.
    if (!write_register(pca, REG_MODE1, MODE1_SLEEP))
        return false;
    if (!write_register(pca, REG_PRESCALE, (uint8_t)prescale))
        return false;
    if (!write_register(pca, REG_MODE1, MODE1_AI | MODE1_RESTART))
        return false;

    delay(1);
    return true;
}

bool pca9685_set_channel(pca9685_t *pca, uint8_t channel, uint16_t value)
{
    if (channel >= PCA9685_CHANNEL_COUNT)
        return false;

    uint8_t regs[4] = {0, 0, 0, 0};
    if (value == 0)
        regs[3] = LED_FULL_ON;
    else if (value >= PCA9685_PWM_STEPS - 1)
        regs[1] = LED_FULL_ON;
    else
    {
        regs[2] = value & 0xFF;
        regs[3] = value >> 8;
    }

    return write_registers(pca, REG_LED0_ON_L + 4 * channel, regs, 4);
}

bool pca9685_set_duty(pca9685_t *pca, uint8_t channel, uint8_t duty)
{
    return pca9685_set_channel(pca, channel, (uint16_t)duty * (PCA9685_PWM_STEPS - 1) / 255);
}

bool pca9685_set_on_off(pca9685_t *pca, uint8_t channel, bool on)
{
    return pca9685_set_channel(pca, channel, on ? PCA9685_PWM_STEPS - 1 : 0);
}
