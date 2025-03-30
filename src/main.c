#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "ibutton.h"


int main(void) {
  TaskHandle_t xOneWireTaskHandle = NULL;
    // Initialize hardware
    init_button();
    Timer1_Init();
    init_USART2();


    // Create tasks
    xTaskCreate(vOneWireTask, "OneWireTask", configMINIMAL_STACK_SIZE*4, NULL, tskIDLE_PRIORITY + 1, &xOneWireTaskHandle);
    xTaskCreate(vUartTask, "UartTask", configMINIMAL_STACK_SIZE*2, NULL, tskIDLE_PRIORITY + 1, NULL);
    

    vTaskStartScheduler();
    
    // Should never reach here
    while (1) {}
}
