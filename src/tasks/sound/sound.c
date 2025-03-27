#include "sound.h"
#include "stm32f10x.h"

#include "gpio.h"
#include "spi.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "spi.h"

#define TIM_CCMR1_OCIM_PWM1 6<<4
#define TIM_CCMR1_OCIM_PWM2 7<<4

//SPI
#define SPI1_CS_PIN GPIOA, 4 // CS pin for SPI1
void init_SPI1(void);
void initTimer_SPI1(void);

//DMA
#define DMA_SOUND_BUFFER_SIZE 256 // 256 * 2 bytes
#define DMA_SOUND_BUFFER_ 128 // 128 * 2 bytes
void initTimer_DMA2(void);
void config_DMA2(uint8_t *buffer, uint16_t size);

// Sound management
SoundState soundState = SOUND_MUTE; // Default state
uint16_t soundBuffer[DMA_SOUND_BUFFER_SIZE]; // Buffer for sound data
void vSoundTask(void *pvParameters);
void vInit_soundTasks(void);
void setSoundState(SoundState state) {soundState = state;} // Function to set sound state


/*******************************IMPLEMENTATION****************************************/
// Sound Management

void vInit_soundTasks(void){
    xTaskCreate(vSoundTask, "Sound Task", 128, NULL, 1, NULL);
}

void vSoundTask(void *pvParameters){
    RCC->APB2ENR |=  RCC_APB2ENR_IOPAEN; // Active l'horloge du GPIOA
    //...
}	


// DMA2

void initTimer_DMA2(void)
{
    RCC->APB1ENR |= (1 << 3); // Active l'horloge du TIM5

    // Configure GPIOA3 comme sortie alternative pour TIM5_CH4
    // Reference Manuel pg 178
    initGpioX(GPIOA, 3, GPIO_MODE_AF_PP_50MHz);

    // Configuration du Prescaler et Auto-Reload (Ajustez selon les besoins)
    TIM5->PSC = 0;      // Pas de division d'horloge
    TIM5->ARR = 98 - 1; // Période du PWM (définissez selon les besoins)
    
    // Configuration du PWM sur le Canal 2 (Mode PWM1)
    TIM5->CCMR1 &= ~TIM_CCMR1_OC2M; // Efface le mode de sortie
    //TIM5->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // Définit PWM1
    TIM5->CCMR1 |= TIM_CCMR1_OC2PE; // Active le preload

    // Définit le cycle de service initial (50%)
    TIM5->CCR2 = 49;

    // Active la sortie PWM sur le Canal 2
    TIM5->CCER |= TIM_CCER_CC2E;

    // Génère une mise à jour pour appliquer les configurations
    TIM5->EGR |= TIM_EGR_UG;

    // Active le DMA pour CCR2
    TIM5->DIER |= TIM_DIER_CC2DE;

    // Active le Timer 5
    TIM5->CR1 |= TIM_CR1_CEN;
}

void config_DMA2(uint8_t *buffer, uint16_t taille)
{   
    volatile uint8_t done = 0;
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;  // Active l'horloge du DMA2

    DMA2_Channel1->CCR &= ~DMA_CCR_EN;  // Désactive le DMA avant de configurer


    DMA2_Channel1->PAR = (uint32_t)&TIM5->CCR2;  // Destination : CCR2 du TIM5
    DMA2_Channel1->M0AR = (uint32_t)buffer;  // Source : Buffer audio
    DMA2_Channel1->NDTR = taille;  // Nombre de transferts

    // Configuration du DMA2_Channel1, Canal 4, Mémoire → Périphérique
    DMA2_Channel1->CR = (4 << DMA_SxCR_CHSEL_Pos) |  // Sélectionne le canal 4
                       DMA_SxCR_MINC |  // Incrémente l'adresse mémoire
                       DMA_SxCR_DIR_MEM_TO_PERIPH |  // Mémoire → Périphérique
                       DMA_SxCR_TCIE |  // Active l'interruption à la fin du transfert
                       DMA_SxCR_CIRC;  // Mode circulaire pour boucle continue

    NVIC_EnableIRQ(DMA2_Channel1_IRQn);  // Active l'interruption dans le NVIC

    DMA2_Channel1->CR |= DMA_SxCR_EN;  // Active le DMA
}

// SPI1