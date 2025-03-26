#include <stdio.h>
#include <stdbool.h>

extern int conversion_reg;
extern int config_reg;
extern int lo_tresh_reg;
extern int high_tresh_reg;

extern int comparate0_1;
extern int comparate0_3;
extern int comparate1_3;
extern int comparate2_3;
extern int comparate0_GND;
extern int comparate1_GND;
extern int comparate2_GND;
extern int comparate3_GND;

extern int gain6v;
extern int gain4v;
extern int gain2v;
extern int gain1v;
extern int gain05v;
extern int gain0256v;

void initADS(int addr, int sdaPin, int sdlPin);
void writeConfigReg(uint8_t config[3]);
int16_t readConversionReg();
void setCompareMode(uint8_t mux_config, uint8_t pga_config);
float computeVolts(int16_t raw);
bool conversionReady();
