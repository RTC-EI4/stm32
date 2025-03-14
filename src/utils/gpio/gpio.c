#include "stm32f10x.h"

#include "gpio.h"

void initGpioX(GPIO_TypeDef* gpioX, uint8_t bitIndex, uint8_t mode) {
    // Check pin number
    if(bitIndex > 15) return;

    // Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG
    RCC->APB2ENR |= (0b1111111 << 2);

    // Check if the pin is in CRL or CRH
    if(bitIndex < 8) {
        uint32_t mask = 0xF << (bitIndex * 4); // Mask to clear the 4 bits
        uint32_t value = (uint32_t)mode << (bitIndex * 4); // Value to set

        GPIOx->CRL = (GPIOx->CRL & ~mask) | value;
    } else {
        uint32_t mask = 0xF << ((bitIndex - 8) * 4); // Mask to clear the 4 bits
        uint32_t value = (uint32_t)mode << ((bitIndex - 8) * 4); // Value to set

        GPIOx->CRH = (GPIOx->CRH & ~mask) | value;
    }
}
