#include <stdlib.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"

#include "neopixel.h"
#include "buttons_task.h"

extern TaskHandle_t handle_NeopixelTask;

// Define the button task
void task_Buttons(void *pvParameters) {
    // Set the pin to input pull-up mode
    initGpioX(GPIOC, 10, GPIO_MODE_OUTPUT_PP_50MHz); // Set the pin to input to pull mode
    // GPIOC->BRR = GPIO_BRR_BR10; // Set the pin output to high (pull-up)

    while(1) {
        // Check if the button is pressed (active low)
        if(GPIOC->IDR & GPIO_IDR_IDR10 == 1) buttonPressedSubroutine();

        vTaskDelay(100); // Wait 10ms
    }
}

// Subroutine to be called when the button is pressed
void buttonPressedSubroutine(void) {
    // xTaskNotify(handle_NeopixelTask, NEOPIXEL_COLOR_RED, eSetValueWithOverwrite);

    // Set GPIOC pin 11 to output high
    initGpioX(GPIOC, 11, GPIO_MODE_OUTPUT_PP_50MHz); // Set the pin to output push-pull mode with 50MHz speed
    GPIOC->BSRR = GPIO_BSRR_BS11; // Set the pin to high
}
