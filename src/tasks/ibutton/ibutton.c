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

// Structure to track the one-wire operation
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
    // initGpioX(GPIO?, ?, 0x04);
}


// USART2 initialization
void init_USART2(void) {
    RCC->APB1ENR |= (1 << 17);
    init_gpioA(2, 9);
    init_gpioA(3, 8);
    USART2->BRR = 36000000 / 9600;
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
    USART2->CR2 = 0x0000;
    USART2->CR3 = 0x0000;
}


unsigned char quartet2hex(unsigned char in) {
    in& = 0x0F;
    return (unsigned char)(in > 9) ? ('A' + in - 10) : ('0' + in);
}

void fabrique_trame(void) {
    fifo_env[pw_fifo++] = '0';
    if (pw_fifo == TAILLE_FIFO) {
        pw_fifo = 0;
    }
    fifo_env[pw_fifo++] = 'x';
    if (pw_fifo == TAILLE_FIFO) {
        pw_fifo = 0;
    }

    // Generate hex representation of the identifier (all 8 bytes)
    for (int i = 0; i < 8; i++) {
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


