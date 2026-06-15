#ifndef ILI9488_H
#define ILI9488_H

#include "main.h"
#include <stdint.h>

#define ILI9488_BLACK   0x0000
#define ILI9488_WHITE   0xFFFF
#define ILI9488_RED     0xF800
#define ILI9488_GREEN   0x07E0
#define ILI9488_BLUE    0x001F
#define ILI9488_YELLOW  0xFFE0
#define ILI9488_CYAN    0x07FF

void ILI9488_Init(void);
void ILI9488_FillScreen(uint16_t color);
void ILI9488_FillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color);
void ILI9488_Print(uint16_t x, uint16_t y, char *str, uint16_t color, uint16_t bg, uint8_t size);

#endif
