#include <stdlib.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"

#include "buttons_task.h"

// Define the button task
void task_Buttons(void *pvParameters) {
    // Set the pin to input pull-up mode
    initGpioX(GPIOC, 10, GPIO_MODE_INPUT_PULL_UP_DOWN); // Set the pin to input to pull mode
    GPIOC->BSRR = GPIO_BSRR_BS10; // Set the pin output to high (pull-up)

    while(1) {
        // Check if the button is pressed (active low)
        if(GPIOC->IDR & GPIO_IDR_IDR10 == 0) buttonPressedSubroutine();

        vTaskDelay(100); // Wait 10ms
    }
}

// Subroutine to be called when the button is pressed
void buttonPressedSubroutine(void) {
    
}
