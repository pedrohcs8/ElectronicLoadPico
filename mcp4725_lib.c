#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

#include "mcp4725_lib.h"

// addresses
int mcp_addr_write;
int mcp_addr_read;

// commands
int fastmode_cmd = 0x0;
int write_dac_reg_cmd = 0x40;
int write_dac_reg_eeprom_cmd = 0x60;

// power down
int normal_pd = 0x0;
int pd_2 = 0x40;
int pd_3 = 0x80;
int pd_4 = 0xC0;


void initMCP(int addr,int sdaPin, int sdlPin) {
  i2c_init(i2c0, 400000);

  mcp_addr_write = addr;
  mcp_addr_read = addr + 1;

  gpio_set_function(sdaPin, GPIO_FUNC_I2C);
  gpio_set_function(sdlPin, GPIO_FUNC_I2C);

  gpio_pull_up(sdaPin);
  gpio_pull_up(sdlPin);

  writeDec(0);
}

void writeDec(int dec) {
  if (dec >= 4096) {
    dec = 4095;
  }

  u_int8_t data[2];
  data[0] = fastmode_cmd | dec >> 8;
  data[1] = dec >> 4;

  i2c_write_blocking(i2c0, 0x60, &data, 2, false);
  /* sleep_ms(2); */
}
