#ifndef NEOPIXEL_H
#define NEOPIXEL_H

typedef enum NeopixelColor_ {
    NEOPIXEL_COLOR_RED,
    NEOPIXEL_COLOR_GREEN,
    NEOPIXEL_COLOR_BLUE,
    NEOPIXEL_COLOR_WHITE,
    NEOPIXEL_COLOR_BLACK,
} NeopixelColor;

void initNeopixelDriver(void);
int8_t setNeopixelData(uint8_t* colors, uint8_t count);

void initTimer2_neo(void);
static void initDMA1_neo(void);

#endif
