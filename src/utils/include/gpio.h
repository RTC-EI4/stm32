#ifndef GPIO_H
#define GPIO_H

#include "stm32f10x.h"

// Input modes
#define GPIO_MODE_INPUT_ANALOG 0x0 // Analog input
#define GPIO_MODE_INPUT_FLOATING 0x4 // Floating input
#define GPIO_MODE_INPUT_PULL_UP_DOWN 0x8 // Input with pull-up / pull-down

// Output modes, push-pull
#define GPIO_MODE_OUTPUT_PP_10MHz 0x1 // Output push-pull 10 MHz
#define GPIO_MODE_OUTPUT_PP_2MHz 0x2 // Output push-pull 2 MHz
#define GPIO_MODE_OUTPUT_PP_50MHz 0x3 // Output push-pull 50 MHz

// Output modes, open-drain
#define GPIO_MODE_OUTPUT_OD_10MHz 0x5 // Output open-drain 10 MHz
#define GPIO_MODE_OUTPUT_OD_2MHz 0x6 // Output open-drain 2 MHz
#define GPIO_MODE_OUTPUT_OD_50MHz 0x7 // Output open-drain 50 MHz

// Outputs in alternate function push-pull
#define GPIO_MODE_AF_PP_10MHz 0x9 // Alternate function push-pull 10 MHz
#define GPIO_MODE_AF_PP_2MHz 0xA // Alternate function push-pull 2 MHz
#define GPIO_MODE_AF_PP_50MHz 0xB // Alternate function push-pull 50 MHz

// Outputs in alternate function open-drain
#define GPIO_MODE_AF_OD_10MHz 0xD // Alternate function open-drain 10 MHz
#define GPIO_MODE_AF_OD_2MHz 0xE // Alternate function open-drain 2 MHz
#define GPIO_MODE_AF_OD_50MHz 0xF // Alternate function open-drain 50 MHz

void initGpioX(GPIO_TypeDef* gpioX, uint8_t bitIndex, uint8_t quartet);

#endif
