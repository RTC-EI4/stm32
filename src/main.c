#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#include "sound.h"

TaskHandle_t handle_SoundTask;

int main(void) {
    // SoundConfig cfg = {SOUND_RESISTANCE, 0, 0};
    // SoundConfig cfg = {SOUND_RESISTANCE, 5, 10}; 
    SoundConfig cfg = {SOUND_E1, 0, 0};

    SystemInit();
    
    vInit_soundTasks();
    
    setSoundState(cfg);

    vTaskStartScheduler();
    
    while(1) {
        // Do nothing
    } 
}
