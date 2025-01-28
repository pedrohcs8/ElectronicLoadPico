#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "ads1015_lib.h"

int ads_addr_write;
int selected_gain;

int conversion_reg = 0x0;
int config_reg = 0x1;
int lo_tresh_reg = 0x2;
int high_tresh_reg = 0x3;

// MUX Configs
int comparate0_1 = 0x0;
int comparate0_3 = 0x10;
int comparate1_3 = 0x20;
int comparate2_3 = 0x30;
int comparate0_GND = 0x40;
int comparate1_GND = 0x50;
int comparate2_GND = 0x60;
int comparate3_GND = 0x70;

//PGA Configs
int gain6v = 0x0;
int gain4v = 0x2;
int gain2v = 0x4;
int gain1v = 0x6;
int gain05v = 0x8;
int gain0256v = 0xA;

void initADS(int addr, int sdaPin, int sdlPin) {
  i2c_init(i2c0, 400000);

  ads_addr_write = addr;

  gpio_set_function(sdaPin, GPIO_FUNC_I2C);
  gpio_set_function(sdlPin, GPIO_FUNC_I2C);

  gpio_pull_up(sdaPin);
  gpio_pull_up(sdlPin);
}

void writeConfigReg(uint8_t config[3]) {
  i2c_write_blocking(i2c0, 0x48, config, 3, false);
}

int16_t readConversionReg() {
  uint8_t input[2];
  i2c_write_blocking(i2c0, 0x48, &conversion_reg, 1, true);
  i2c_read_blocking(i2c0, 0x48, &input, 2, false);

  uint16_t result = input[0] << 8 | input[1];
  return result;
}

double computeVolts(int raw, bool miliVolts, uint8_t pga_config) {
  if (pga_config == gain6v) {
    if (miliVolts) {
      return (raw * 0.125);
    } else {
      return (raw * 0.125) / 100;
    }
  } else if (pga_config == gain0256v) {
    if (miliVolts) {
      return (raw * 0.125);
    } else {
      return (raw * 0.125) / 1000;
    }
  }
}

void setCompareMode(uint8_t mux_config, uint8_t pga_config) {
  uint8_t data[3];
  data[0] = config_reg;
  data[1] = 0x81 | mux_config | pga_config;
  data[2] = 0xE0;

  writeConfigReg(data);

  selected_gain = pga_config;
}

bool conversionReady() {
  uint8_t input[2];
  i2c_write_blocking(i2c0, 0x48, &config_reg, 1, true);
  i2c_read_blocking(i2c0, 0x48, &input, 2, false);

  if ((input[0] & 0x80) == 0x80) {
    return true;
  } else {
    return false;
  }
}
