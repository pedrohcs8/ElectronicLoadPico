#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include <math.h>

#include "i2c_lcd_lib.h"
#include "ads1115_lib.h"
#include "mcp4725_lib.h"

volatile bool runUpdate = false;

bool ccw_fall = 0;  //bool used when falling edge is triggered
bool cw_fall = 0;

int selector_pos = 1;
int amp_setting = 0;
int dac_value = 0;

float amps_reading = 0;
float volts_reading = 0;

// PID Amp Controller
double error;
double derivative;
double integral;
uint32_t lastProcess;
double lastAmp;

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
}

void refreshDisplayCallback() {
  clear_display();

  writeHex(right_pointer);
  writeHex(display_no_char);

  writeInt(amp_setting);
  writeText("AMPS", 4);

  writeHex(display_no_char);

  int volts_int, volts_frac, volts_tmp;

  volts_tmp = (volts_reading / 0.09) * 100;
  volts_int = volts_tmp / 100;
  volts_tmp -= volts_int * 100;
  volts_frac = volts_tmp / 10;

  writeInt(volts_int);
  writeText(".", 1);
  writeInt(volts_frac);
  writeText("VOLT", 4);

  char display_amp_numbers[4];

  sprintf(display_amp_numbers, "%d", (int) amps_reading);

  moveCursorLine(2);
  writeInt(display_amp_numbers[0] - '0');
  writeInt(display_amp_numbers[1] - '0');
  writeInt(display_amp_numbers[2] - '0');
  writeInt(display_amp_numbers[3] - '0');
  writeText("AMPS", 4);
}

bool repeating_timer_callback(__unused struct repeating_timer *t) {
  /* printf("Repeat at %lld\n", time_us_64()); */
  runUpdate = true;
  return true;
}

int main() {
  stdio_init_all();

  struct repeating_timer timer;
  add_repeating_timer_ms(300, repeating_timer_callback, NULL, &timer);

  gpio_set_irq_enabled_with_callback(9, GPIO_IRQ_EDGE_FALL, true, &readEncoder); // Set Interrupt on Falling Edge
  gpio_set_irq_enabled(10, GPIO_IRQ_EDGE_FALL, true);

  initLCD(0x27, 4, 5);
  initADS(0x48, 4, 5);
  initMCP(0x60, 4, 5);

  while (true) {
    if (runUpdate) {
      refreshDisplayCallback();
      runUpdate = false;
    }

    /* setCompareMode(comparate2_GND); */
    /* while(!conversionReady()); */
    /* volts_reading = computeVolts(readConversionReg(), false); */
    /* printf("%f\n", computeVolts(readConversionReg(), true)); */

    setCompareMode(comparate0_1);
    while(!conversionReady());
    amps_reading = readConversionReg() * 2.2;

    float voltage_on_load = amps_reading;
    float mA_setpoint = amp_setting * 1000;
    
    error = mA_setpoint - voltage_on_load; 

   if (error > (mA_setpoint*0.8))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 300;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 300;
      }
    }

    else if (error > (mA_setpoint*0.6))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 170;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 170;
      }
    }

    else if (error > (mA_setpoint*0.4))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 120;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 120;
      }
    }
    else if (error > (mA_setpoint*0.3))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 60;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 60;
      }
    }

    else if (error > (mA_setpoint*0.2))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 40;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 40;
      }
    }
    
    else if (error > (mA_setpoint*0.1))
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 30;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 30;
      }
    }
    else
    {
      if(mA_setpoint > voltage_on_load){
        dac_value = dac_value + 1;
      }

      if(mA_setpoint < voltage_on_load){
        dac_value = dac_value - 1;
      }
    } 

    if (amp_setting == 0) {
      writeDec(0);
      dac_value = 0;
      integral = 0;
      derivative = 0;
    }

    // with opamp
    /* writeDec(mosfet_voltage + 1520); */

    //3100
    writeDec(dac_value);
  };
}
