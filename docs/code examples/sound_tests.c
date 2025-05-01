
// To use this code, you need to replace the main.c file in your project with this one.

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"
#include "sound.h"
#include "spi.h"
#include "uart.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h>

#define TEST_MODE 0

//1 = PWM and LED test
//2 = SPI test
//3 = SPI test with sound buffer and DMA


#if TEST_MODE == 0
	int main(){
		return 0;
	}
#endif

#if TEST_MODE == 1 // PWM and LED test
// ---------------- GPIO Setup for LED ----------------
void init_led(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    GPIOA->CRL &= ~(0xF << 20);             // Clear MODE5[1:0] and CNF5[1:0] bits
    GPIOA->CRL |= (0x2 << 20);              // MODE5 = 0b10 (Output 2MHz), CNF5 = 0b00 (GP Push-Pull)
}

// ---------------- PWM Setup for PA0 (TIM2_CH1) ----------------
void init_pwm_pa0(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // PA0 = TIM2_CH1, set to AF Push-Pull output
    GPIOA->CRL &= ~(0xF << 0);              // Clear MODE0[1:0] and CNF0[1:0]
    GPIOA->CRL |= (0x2 << 0) | (0x2 << 2);  // MODE0 = 0b10 (2MHz), CNF0 = 0b10 (AF Push-Pull)

    // Timer config
    TIM2->PSC = 71;       // 72 MHz / (71+1) = 1 MHz
    TIM2->ARR = 227;      // 1 MHz / 227 = ~440 Hz
    TIM2->CCR1 = 0;

    TIM2->CCMR1 |= (6 << 4);    // OC1M = 110 (PWM Mode 1), Bits 6:4
    TIM2->CCMR1 |= (1 << 3);    // OC1PE = 1 (Preload enable)
    TIM2->CCER  |= 1;           // CC1E = 1 (Enable output)
    TIM2->CR1   |= (1 << 7);    // ARPE = 1 (Auto-reload preload)
    TIM2->CR1   |= 1;           // CEN = 1 (Counter enable)
}

// ---------------- Tasks ----------------
void vBlinkTask(void *pvParameters) {
    while (1) {
        GPIOA->ODR ^= (1 << 5);    // Toggle PA5 (LED LD2)
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void vSpeakerTask(void *pvParameters) {
    uint16_t duty = 0;
    while (1) {
        TIM2->CCR1 = duty;
        duty += 5;
        if (duty >= TIM2->ARR) duty = 0;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ---------------- Main ----------------
int main(void) {
    SystemInit();
    init_led();
    init_pwm_pa0();

    xTaskCreate(vBlinkTask, "Blink", 128, NULL, 1, NULL);
    xTaskCreate(vSpeakerTask, "Speaker", 128, NULL, 1, NULL);

    vTaskStartScheduler();

    while (1);
}
#endif
#if TEST_MODE == 2 // SPI test
#define SOUND_BASE_ADDR 0x021700
int main(void) {
    uint16_t audio_buffer[PAGE_SIZE];  // Buffer pour les donnees audio
    uint32_t addr = SOUND_BASE_ADDR;  // Adresse de depart pour la lecture
    char pg_ct[20];  // Chaine de caracteres pour le compteur de pages
    // Initialiser SPI1
    initSound_SPI1();  // Initialiser le SPI1 pour le son
    USART2_init(9600);
    SPI_ReadJEDEC_ID(SPI1, GPIOA, 4);  // Lire l'ID JEDEC
    USART2_SendString("\r\n[JEDEC ID OK]\r\n");  // Afficher l'ID JEDEC
    
    //sprintf(pg_ct, "\r\n[%d]\r\n", PAGE_COUNT);  // Formater la chaine de caracteres
    USART2_SendString(pg_ct);
    while (1) {
        delay_ms(1000);  // Attendre 1 seconde
        SPI_read_page(SPI1, GPIOA, 4, addr, audio_buffer);  // Lire une page de données
        addr += PAGE_SIZE;
    }
}
#endif
#if TEST_MODE == 3
 #define sample_size		200


 uint16_t lookUp1[sample_size] = {0,50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494 ,542 ,589 ,635 ,681
     ,726 ,771 ,814 ,857 ,899 ,940 ,981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233 ,1264
     ,1294 ,1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,1536 ,1550 ,1561
     ,1572 ,1580 ,1587 ,1593 ,1597 ,1599 ,1600 ,1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561
     ,1550 ,1536 ,1522 ,1505 ,1488 ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,1264
     ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981 ,940 ,899 ,857 ,814 ,771 ,726 ,681 ,635
     ,589 ,542 ,494 ,446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
     ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
     ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
     ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 };
 
 uint16_t lookUp2[sample_size] = {0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
     ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
     ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0
     ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,50 ,100 ,151 ,201 ,250 ,300 ,349 ,398 ,446 ,494
     ,542 ,589 ,635 ,681 ,726 ,771 ,814 ,857 ,899 ,940 ,981 ,1020 ,1058 ,1095 ,1131 ,1166 ,1200 ,1233
     ,1264 ,1294 ,1323 ,1351 ,1377 ,1402 ,1426 ,1448 ,1468 ,1488 ,1505 ,1522 ,1536 ,1550 ,1561 ,1572 ,1580
     ,1587 ,1593 ,1597 ,1599 ,1600 ,1599 ,1597 ,1593 ,1587 ,1580 ,1572 ,1561 ,1550 ,1536 ,1522 ,1505 ,1488
     ,1468 ,1448 ,1426 ,1402 ,1377 ,1351 ,1323 ,1294 ,1264 ,1233 ,1200 ,1166 ,1131 ,1095 ,1058 ,1020 ,981
     ,940 ,899 ,857 ,814 ,771 ,726 ,681 ,635 ,589 ,542 ,494 ,446 ,398 ,349 ,300 ,250 ,201 ,151 ,100 ,50 ,0};
  
 int main(void)
 {
     RCC->APB2ENR|=RCC_APB2ENR_IOPAEN;
 
     /*Configure PA0 as Output Alternate Push/Pull */
     GPIOA->CRL|=GPIO_CRL_MODE0;
     GPIOA->CRL|=(GPIO_CRL_CNF0_1);
     GPIOA->CRL&=~(GPIO_CRL_CNF0_0);
 
     /*Configure PA1 as Output Alternate Push/Pull */
     GPIOA->CRL|=GPIO_CRL_MODE1;
     GPIOA->CRL|=(GPIO_CRL_CNF1_1);
     GPIOA->CRL&=~(GPIO_CRL_CNF1_0);
 
     /*Don't remap the pin*/
     AFIO->MAPR&=~AFIO_MAPR_TIM2_REMAP;
 
 
     /*Enable clock access to timer2*/
     RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
 
     /*Configure timer2*/
     TIM2->PSC=0;
     TIM2->ARR=1600;
     TIM2->CCMR1|=TIM_CCMR1_OC1M_2|TIM_CCMR1_OC1M_1|TIM_CCMR1_OC2M_2|TIM_CCMR1_OC2M_1;
     TIM2->CCER|=TIM_CCER_CC1E|TIM_CCER_CC2E;
 
     TIM2->DIER|=TIM_DIER_CC1DE|TIM_DIER_CC2DE;
 
 
     /*DMA configuration*/
 
     RCC->AHBENR|=RCC_AHBENR_DMA1EN;
 
     DMA1_Channel5->CCR=DMA_CCR1_MSIZE_0|DMA_CCR1_PSIZE_0|
             DMA_CCR1_MINC|DMA_CCR1_CIRC|DMA_CCR1_DIR;
 
     DMA1_Channel5->CNDTR=(uint16_t)sample_size;
     DMA1_Channel5->CMAR=(uint32_t)lookUp1;
     DMA1_Channel5->CPAR=(uint32_t)(&TIM2->CCR1);
 
     DMA1_Channel7->CCR=DMA_CCR1_MSIZE_0|DMA_CCR1_PSIZE_0|
             DMA_CCR1_MINC|DMA_CCR1_CIRC|DMA_CCR1_DIR;
 
     DMA1_Channel7->CNDTR=(uint16_t)sample_size;
     DMA1_Channel7->CMAR=(uint32_t)lookUp2;
     DMA1_Channel7->CPAR=(uint32_t)(&TIM2->CCR2);
 
     DMA1_Channel5->CCR|=DMA_CCR1_EN;
     DMA1_Channel7->CCR|=DMA_CCR1_EN;
 
     TIM2->CR1|=TIM_CR1_CEN;
 
     while(1)
     {
 
 
     }
 }
#endif