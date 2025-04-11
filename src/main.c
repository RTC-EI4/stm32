#include <stdint.h>
#include <stdlib.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"

#include "neopixel.h"
#include "buttons_task.h"

/*

    Create task

*/

TaskHandle_t handle_NeopixelTask;
TaskHandle_t handle_ButtonTask;

void createTasks() {
    // Neopixel task
    if (xTaskCreate(task_Neopixel, "Neopixel", NEOPIXEL_TASK_STACK_SIZE, (void *)1, tskIDLE_PRIORITY + 2, &handle_NeopixelTask) != pdPASS) {
        // Error
    }

    // Buttons reading task
    if (xTaskCreate(task_Buttons, "Buttons", NEOPIXEL_TASK_STACK_SIZE, (void *)1, tskIDLE_PRIORITY + 2, &handle_ButtonTask) != pdPASS) {
        // Handle task creation failure
    }
}

int main() {
    // Create all tasks
    createTasks();

    // Start all tasks
    vTaskStartScheduler();

    // Error, should never reach here
    while(1) {}
}
