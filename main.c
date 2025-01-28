#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/rtc.h"
#include "hardware/clocks.h"

#include "i2c_lcd_lib.h"
#include "ads1015_lib.h"
#include "mcp4725_lib.h"

bool ccw_fall = 0;  //bool used when falling edge is triggered
bool cw_fall = 0;
volatile bool bttMenuMode = 0;
bool mutexCurrent = 0;

int selector_pos = 1;
int amp_setting = 0;
int dac_value = 0;

float amps_reading = 0;
int16_t volts_reading = 0;

//Mutex
uint32_t owner_out;

// RTC
datetime_t setTime = {
    .year = 2008,
    .month = 10,
    .day = 13,
    .dotw = 1,
    .hour = 00,
    .min = 00,
    .sec = 00,
};

static recursive_mutex_t writeDisplayMutex;
auto_init_recursive_mutex(writeDisplayMutex);

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void readEncoder(uint gpio, uint32_t event_mask) {
  bool astate = gpio_get(9);
  bool bstate = gpio_get(10);

  if (gpio == 9) {
    if (!ccw_fall && !astate && bstate) {
      ccw_fall = true;
    }

    if (cw_fall && !astate && !bstate) {
      cw_fall = false;
      ccw_fall = false;

      amp_setting--;
    }
  }

  if (gpio == 10) {
    if (!cw_fall && astate && !bstate) {
      cw_fall = true;
    }

    if (ccw_fall && !astate && !bstate) {
      cw_fall = false;
      ccw_fall = false;

      amp_setting++;
    }
  }

  if (gpio == 12) {
    bttMenuMode = !bttMenuMode;
    rtc_set_datetime(&setTime);
  }
}

void refreshDisplayCallback() {
  clear_display();

  writeHex(right_pointer);
  writeHex(display_no_char);

  writeInt(amp_setting);
  writeText("AMPS", 4);

  writeHex(display_no_char);

  float volts = computeVolts(volts_reading, false, gain6v);

  char display_volts[4];
  snprintf(display_volts, 4, "%f", volts + 0.4);

  if (volts > 10) {
    writeInt(display_volts[0] - '0');
    writeInt(display_volts[1] - '0');
    writeText(".", 1);
    writeInt(display_volts[3] - '0');
  } else {
    writeInt(display_volts[0] - '0');
    writeText(".", 1);
    writeInt(display_volts[2] - '0');
  }

  writeText("V", 1);

  moveCursorLine(2);
  display4Int(amps_reading);
  writeText("AMPS", 4);
}

void bttTesterRefresh() {
  clear_display();
  writeHex(right_pointer);
  writeHex(display_no_char);

  writeInt(amp_setting);
  writeText("AMPS", 4);

  writeHex(display_no_char);
  
  datetime_t elapsedTime;
  rtc_get_datetime(&elapsedTime);

  int minutes = elapsedTime.min;

  display2Int(minutes);
  writeText("MIN", 3);

  moveCursorLine(2);

  int mah = amps_reading * minutes / 60;

  display4Int(mah);
  writeText("MAH", 3);

  writeHex(display_no_char);

  float volts = computeVolts(volts_reading, false, gain6v);

  char display_volts[4];
  snprintf(display_volts, 4, "%f", volts / 0.81);

  if (volts > 10) {
    writeInt(display_volts[0] - '0');
    writeInt(display_volts[1] - '0');
    writeText(".", 1);
    writeInt(display_volts[3] - '0');
  } else {
    writeInt(display_volts[0] - '0');
    writeText(".", 1);
    writeInt(display_volts[2] - '0');
  }

  writeText("V", 1);
}

void core1_main() {
  while(true) {
    recursive_mutex_enter_blocking(&writeDisplayMutex);

    if (bttMenuMode) {
      bttTesterRefresh();
    } else {
      refreshDisplayCallback();
    }

    long pwm_value = map(amps_reading, 0, 2000, 0, 255);
    pwm_set_chan_level(0, PWM_CHAN_A, pwm_value);

    recursive_mutex_exit(&writeDisplayMutex);

    sleep_ms(350);
  }
}

int main() {

  gpio_set_irq_enabled_with_callback(9, GPIO_IRQ_EDGE_FALL, true, &readEncoder); // Set Interrupt on Falling Edge
  gpio_set_irq_enabled(10, GPIO_IRQ_EDGE_FALL, true);
  gpio_pull_up(12);
  gpio_set_irq_enabled(12, GPIO_IRQ_EDGE_RISE, true);

  gpio_set_function(0, GPIO_FUNC_PWM);

  float pwm_divider = (float)(clock_get_hz(clk_sys)) / (25000 * 255);     // Calculate the desired PWM divisor
  pwm_config config = pwm_get_default_config();
  pwm_config_set_clkdiv(&config, pwm_divider);
  pwm_config_set_wrap(&config, 255 - 1);      // Set the PWM to wrap at 254. This allows a PWM value of 255 to equal 100% duty cycle

  pwm_init(0, &config, false);
  pwm_set_enabled(0, true);

  initLCD(0x27, 6, 7);
  initADS(0x48, 4, 5);
  initMCP(0x60, 4, 5);

  rtc_init();
  rtc_set_datetime(&setTime);

  multicore_launch_core1(core1_main);

  while (true) {
    int current;

    if (recursive_mutex_try_enter(&writeDisplayMutex, &owner_out)) {
      setCompareMode(comparate0_1, gain0256v);
      while(!conversionReady());
      amps_reading = readConversionReg() * 2.2;
    
      setCompareMode(comparate2_GND, gain6v);
    
      while(!conversionReady());
      volts_reading = readConversionReg();
    
      current = amps_reading;
      mutexCurrent = true;
    
      recursive_mutex_exit(&writeDisplayMutex);
    }

    if (!mutexCurrent) {
      setCompareMode(comparate0_1, gain0256v);
      while(!conversionReady());
      current = readConversionReg() * 2.2;
    }

    int mA_setpoint = amp_setting * 1000;

    if (current < mA_setpoint) {
      if (dac_value < 4095) {
        dac_value++;
      }
    } else if (current > mA_setpoint) {
      if (dac_value > 0) {
        dac_value--;
      }
    }

    if (amp_setting == 0) {
      writeDec(0);
      dac_value = 0;
    }

    // with opamp
    /* writeDec(mosfet_voltage + 1520); */

    //3100
    writeDec(dac_value + 800); //600
    mutexCurrent = false;
  };
}
