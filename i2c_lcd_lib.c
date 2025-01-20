#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"

#include "i2c_lcd_lib.h"

uint8_t display_no_char = 0x10;
uint8_t right_pointer = 0x7E;

uint16_t clear_disp_cmd = 0x41C;
uint16_t cursor_1st_cmd = 0x8C0C;
uint16_t cursor_2nd_cmd = 0xCC0C;

bool backligth_on = true;

void initLCD(int addr,int sdaPin, int sdlPin) {
  i2c_init(i2c1, 400000);

  gpio_set_function(sdaPin, GPIO_FUNC_I2C);
  gpio_set_function(sdlPin, GPIO_FUNC_I2C);

  gpio_pull_up(sdaPin);
  gpio_pull_up(sdlPin);

  writeReg(0x2C); // Sets 4 Bit Operation

  writeReg(0x2C); // Config 4 Bit/Font, etc
  writeReg(0x8C);

  writeReg(0xC); // Display ON
  writeReg(0xEC);

  writeReg(0xC); // ENTRY MODE
  writeReg(0x6C);
}

void writeReg(uint8_t data) {
  i2c_write_blocking(i2c1, 0x27, &data, 1, false);

  sleep_ms(2);

  uint8_t data3;

  if (backligth_on) {
    data3 = 0x8;
  } else {
    data3 = 0x0;
  }

  i2c_write_blocking(i2c1, 0x27, &data3, 1, false);
}

void clear_display() {
  uint8_t bytes[2];
  bytes[0] = clear_disp_cmd >> 8;     // High Byte
  bytes[1] = clear_disp_cmd & 0x00FF; // Low Byte

  writeReg(bytes[0]);
  writeReg(bytes[1]);
  sleep_ms(2);
}

void writeText(char string[], int size) {
  for (int i = 0; i < size; i++) {
    uint8_t char_hex = string[i];

    uint8_t high_char_hex = char_hex >> 4 << 4;
    uint8_t low_char_hex = (char_hex << 4) & 0xff;

    uint8_t high_data = high_char_hex | 0xD;
    uint8_t low_data = low_char_hex | 0xD;

    writeReg(high_data);
    writeReg(low_data);
  }
}

void writeHex(uint8_t hex) {
  uint8_t high_char_hex = hex >> 4 << 4;
  uint8_t low_char_hex = (hex << 4) & 0xff;

  uint8_t high_data = high_char_hex | 0xD;
  uint8_t low_data = low_char_hex | 0xD;

  writeReg(high_data);
  writeReg(low_data);
}

void writeInt(uint8_t number) {
  uint8_t low_int_hex = (number << 4) & 0xff;

  uint8_t high_data = 0x3D;
  uint8_t low_data = low_int_hex | 0xD;

  writeReg(high_data);
  writeReg(low_data);
}

void moveCursor(uint8_t pos) {
  uint8_t high_pos_hex = pos >> 4 << 4;
  uint8_t low_pos_hex = (pos << 4) & 0xff;

  uint8_t high_data = high_pos_hex | 0x8C;     // High Byte
  uint8_t low_data = low_pos_hex | 0xC; // Low Byte

  writeReg(high_data);
  writeReg(low_data);
}

void moveCursorLine(int pos) {
  uint16_t cmd;

  switch (pos) {
    case 1: {
      cmd = cursor_1st_cmd;
      break;
    }

    case 2: {
      cmd = cursor_2nd_cmd;
      break;
    }
  }

  uint8_t bytes[2];

  bytes[0] = cmd >> 8;     // High Byte
  bytes[1] = cmd & 0x00FF; // Low Byte

  writeReg(bytes[0]);
  writeReg(bytes[1]);
}

void display2Int(int number) {
  char display_number[4];
  sprintf(display_number, "%d", number);

  if (number < 10) {
    writeInt(display_number[1] - '0');
    writeInt(display_number[0] - '0');
  } else {
    writeInt(display_number[0] - '0');
    writeInt(display_number[1] - '0');
  }
}

void display4Int(int number) {
  char display_number[4];
  sprintf(display_number, "%d", number);

  if (number >= 10 && number < 100) {
    writeInt(display_number[2] - '0');
    writeInt(display_number[3] - '0');
    writeInt(display_number[0] - '0');
    writeInt(display_number[1] - '0');
  }

  if (number >= 100 && number < 1000) {
    writeInt(display_number[3] - '0');
    writeInt(display_number[0] - '0');
    writeInt(display_number[1] - '0');
    writeInt(display_number[2] - '0');
  }

  if (number >= 1000) {
    writeInt(display_number[0] - '0');
    writeInt(display_number[1] - '0');
    writeInt(display_number[2] - '0');
    writeInt(display_number[3] - '0');
  }
}
