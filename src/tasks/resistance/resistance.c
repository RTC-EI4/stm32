#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"
#include "resistance.h"

const uint16_t SEUIL1[] = {
    4094, 4094, 4094, 4094, 4093, 4093, 4093, 4092, 4092, 4091, 
    4090, 4089, 4087, 4086, 4084, 4081, 4078, 4074, 4069, 4064, 
    4058, 4051, 4042, 4031, 4018, 4002, 3981, 3956, 3928, 3893, 
    3850, 3804, 3752, 3691, 3618, 3532, 3432, 3320, 3183, 3033, 
    2875, 2695, 2504, 2322, 2143, 1959, 1772, 1583, 1401, 1231, 
    1065, 913, 785, 664, 559, 476, 406, 345, 291, 244
};

const uint16_t SEUIL2[] = {
    3898, 3859, 3810, 3752, 3686, 3605, 3511, 3414, 3307, 3185,
    3048, 2893, 2723, 2545, 2346, 2141, 1946, 1741, 1542, 1369,
    1214, 1065, 927, 797, 681, 580, 487, 406, 341, 283,
    234, 197, 166, 140, 117, 97, 80, 67, 55, 45,
    37, 30, 25, 21, 18, 15, 12, 10, 8, 7,
    6, 5, 4, 3, 3, 2, 2, 2, 1, 1
};


static TaskHandle_t xResistanceTaskHandle = NULL;

static volatile uint16_t adcValue = 0;
static volatile uint8_t conversionComplete = 0;

void Resistance_Init(void) {


}


void Resistance_Measure(void){

}

//other functions to manage the resistance measurement ?

void vResistanceTask(void *pvParameters) {

    while (1) {

    }
}