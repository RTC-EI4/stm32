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
void task_Buttons(void *params_p) {
    // Set the pin to input pull-up mode
    initGpioX(GPIOC, 13, GPIO_MODE_INPUT_FLOATING); // Set the pin to input to pull mode

    while(1) {
        // Check if the button is pressed (active low)
        if((GPIOC->IDR & GPIO_IDR_IDR13) == 0) buttonPressedSubroutine();

        vTaskDelay(100); // Wait 10ms
    }
}

// Subroutine to be called when the button is pressed
void buttonPressedSubroutine(void) {
    xTaskNotify(handle_NeopixelTask, NEOPIXEL_COLOR_RED, eSetValueWithOverwrite);

    initGpioX(GPIOA, 5, GPIO_MODE_OUTPUT_PP_50MHz); // Set the pin to input to pull mode
    GPIOA->BSRR = GPIO_BSRR_BS5;
}
