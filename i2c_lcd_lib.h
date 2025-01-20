#include <stdint.h>

extern uint8_t display_no_char;
extern uint8_t right_pointer;

void initLCD(int addr,int sdaPin, int sdlPin);
void clear_display();
void writeText(char string[], int size);
void writeHex(uint8_t hex);
void writeReg(uint8_t data);
void writeInt(uint8_t number);
void moveCursorLine(int pos);
void moveCursor(uint8_t pos);
void display2Int(int number);
void display4Int(int number);
