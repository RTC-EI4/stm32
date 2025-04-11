#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "gpio.h"


#define ADC_CHANNEL          ADC_Channel_0 //TODO
#define RES1_VALUE           47000      
#define RES2_VALUE           180          
#define TRANSISTOR_PORT      GPIOB
#define TRANSISTOR_PIN       GPIO_Pin_0     //TODO
#define STABILIZATION_DELAY  10             


#define ADC_MAX_VALUE        4095


#define SEUIL1_SIZE (sizeof(SEUIL1) / sizeof(SEUIL1[0]))
#define SEUIL2_SIZE (sizeof(SEUIL2) / sizeof(SEUIL2[0]))