#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "gpio.h"


// communication par FIFO
#define TAILLE_FIFO 32
uint8_t fifo_env[TAILLE_FIFO];
uint8_t pw_fifo = 0;
uint8_t pr_fifo = 0;
uint16_t place_libre = TAILLE_FIFO;


unsigned char identifiant[8];
volatile unsigned char motif_onewire_fini = 0;
volatile unsigned char etat_onewire = 0;
#define BIT_INDIQUANT_ENVOI_POSSIBLE (1 << 7)
#define TIM1CLK_1US 1

TaskHandle_t xOneWireTaskHandle = NULL;

// machine a etat pour la communication 1-wire
typedef enum {
    OW_IDLE,
    OW_RESET,
    OW_SEND_ROM_CMD,
    OW_READ_ROM,
    OW_COMPLETE
} OneWireState_t;


typedef struct {
    OneWireState_t state;
    uint8_t byteIndex;
    uint8_t bitIndex;
    uint8_t currentByte;
} OneWireOperation_t;

volatile OneWireOperation_t owOperation = {OW_IDLE, 0, 0, 0};

void tentative_depile_fifo(void) {
    if ((USART2->SR & (BIT_INDIQUANT_ENVOI_POSSIBLE))) {
        place_libre++;
        USART2->DR = fifo_env[pr_fifo++];
        if (pr_fifo == TAILLE_FIFO) {
            pr_fifo = 0;
        }
    }
}



void init_button(){
    // initGpioX(GPIO?, ?, 0x04); //TODO a remplacer pas les bons arguments
}


// USART2 initialization
void init_USART2(void) {
    RCC->APB1ENR |= (1 << 17);
    initGpioX(GPIOA, 2, 0x09);
    initGpioX(GPIOA, 3, 0x08);
    // init_gpioA(2, 9);
    // init_gpioA(3, 8);
    USART2->BRR = 36000000 / 9600;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART2->CR2 = 0x0000;
    USART2->CR3 = 0x0000;
}


unsigned char quartet2hex(unsigned char in) {
    in &= 0x0F;
    return (unsigned char)(in > 9) ? ('A' + in - 10) : ('0' + in);
}

void fabrique_trame(void) {
    int i;
    fifo_env[pw_fifo++] = '0';
    if (pw_fifo == TAILLE_FIFO) {
        pw_fifo = 0;
    }
    fifo_env[pw_fifo++] = 'x';
    if (pw_fifo == TAILLE_FIFO) {
        pw_fifo = 0;
    }

    // Generate hex representation of the identifier (all 8 bytes)
    for (i = 0; i < 8; i++) {
        fifo_env[pw_fifo++] = (unsigned char)quartet2hex(identifiant[i] >> 4);
        if (pw_fifo == TAILLE_FIFO) {
            pw_fifo = 0;
        }
        fifo_env[pw_fifo++] = (unsigned char)quartet2hex(identifiant[i]);
        if (pw_fifo == TAILLE_FIFO) {
            pw_fifo = 0;
        }
    }

    fifo_env[pw_fifo++] = 0x0A;
    if (pw_fifo == TAILLE_FIFO) {
        pw_fifo = 0;
    }
    fifo_env[pw_fifo++] = 0x0D;
    if (pw_fifo == TAILLE_FIFO) {
        pw_fifo = 0;
    }

    place_libre -= 20;
}


void TIM1_CC_IRQHandler(void) { 
    if (TIM1->SR & TIM_SR_CC2IF) {
        TIM1->SR &= ~TIM_SR_CC2IF;  // Effacer le flag CH2
        TIM1->CR1 &= ~TIM_CR1_CEN; // Arreter le timer
        // Action a effectuer apres CH2 (arret du timer)
        GPIOA->BSRR = 0x01<< (9+16) ;//effacement PA9 
        motif_onewire_fini = 1;

        //ici on peut faire une notification car on a fini de lire le motif
        if (xOneWireTaskHandle != NULL) {
            vTaskNotifyGiveFromISR(xOneWireTaskHandle, NULL); //TODO : is the second parameter xHigherPriorityTaskWoken necessary?
        }
    }
    // Interruption CH3 (independante)
    if (TIM1->SR & TIM_SR_CC3IF) {
        TIM1->SR &= ~TIM_SR_CC3IF;  // Effacer le flag CH3
        // Action a effectuer apres CH3
        GPIOA->BSRR = 0x04<< (9+16) ;//effacement  PA11
        if(GPIOA->IDR & (1<<8)) {
            etat_onewire=1;
        } else {
            etat_onewire=0;
        }	
    }
}


void Timer1_Start(uint16_t pulse_duration, uint16_t ch2_duration, uint16_t ch3_duration) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
	// Configurer la duree du pulse sur CH1
    TIM1->CCR1 = pulse_duration; // En ticks (10 kHz)

    // Configurer la duree pour arreter le timer sur CH2
    TIM1->CCR2 = ch2_duration; // En ticks (10 kHz)
    TIM1->CCR3 = ch3_duration; // En ticks (10 kHz)
    // Configurer la duree pour l'IT sur CH3
    if (ch3_duration < ch2_duration) {
        TIM1->DIER |= TIM_DIER_CC3IE; // Desactiver IT CH3 si ch3_duration > ch2_duration
    } else {
        TIM1->DIER &= ~TIM_DIER_CC3IE; // Desactiver IT CH3 si ch3_duration > ch2_duration
    }

    // Reinitialiser le compteur
    TIM1->CNT = 0;
	GPIOA->BSRR = 0x05<< (9) ;//montee PA9 et PA11
    // Lancer le timer
    TIM1->CR1 |= TIM_CR1_CEN;
}


void Timer1_Init(void) {
    // Activer l'horloge pour Timer 1 et GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;

    // Configurer PA8 en collecteur ouver t
    initGpioX(GPIOA, 8, 0x0D); //fonction alternate  pour CH1 collecteur ouvert mettre pull up externe
    initGpioX(GPIOA, 9, 3); //sortie espion
    initGpioX(GPIOA, 10, 9); //quartet 9 pour config CRH en fonction alternate pour CH3  push pull
    initGpioX(GPIOA, 11, 3); //sortie espion
    GPIOA->BSRR = 0x05<< (9+16) ;//effacement PA9 et PA11
    
    // Configurer la frequence du timer
    TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->PSC = 71; // Divise l'horloge systeme a 1MHz (72 MHz / 72)
    TIM1->ARR = 2000;//0xFFFF;
    TIM1->CNT =0x0000;
    TIM1->EGR = TIM_EGR_UG;// permet de generer l evenement de reload
    
    // Configurer CH1 (Output Compare pour le pulse niveau bas)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M; // Clear mode bits pour CH1
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Mode PWM1
    TIM1->CCER  |= TIM_CCER_CC1E;   // Activer la sortie CH1
    TIM1->CCER  |= TIM_CCER_CC1P;   // Polarite active bas

    // Configurer CH2 (One Pulse, arrete le timer a un certain temps)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC2M; // Clear mode bits pour CH2
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // Mode PWM1
    TIM1->CCER |= TIM_CCER_CC2E;   // Activer la sortie CH2

    // Configurer CH3 (IT independante pour une duree definie)
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M; // Clear mode bits pour CH3
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Mode PWM1
    TIM1->CCER |= TIM_CCER_CC3E;   // Activer la sortie CH3

    // Activer One Pulse Mode (OPM) pour CH2
    TIM1->CR1 |= TIM_CR1_OPM;
        
    // Activer le Main Output Enable pour permettre la sortie sur les pins
    TIM1->BDTR |= TIM_BDTR_MOE;
        
    // Activer les interruptions sur CH2 et CH3
    TIM1->DIER |= TIM_DIER_CC2IE | TIM_DIER_CC3IE;

    // Activer les interruptions Timer 1
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}




