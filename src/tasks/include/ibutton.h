#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "gpio.h"


// communication par FIFO
#define TAILLE_FIFO 32


#define BIT_INDIQUANT_ENVOI_POSSIBLE (1 << 7)
#define TIM1CLK_1US 1

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

void tentative_depile_fifo(void);
// void init_button();
void init_USART2(void);
unsigned char quartet2hex(unsigned char in);
void fabrique_trame(void);
void TIM1_CC_IRQHandler(void);
void Timer1_Start(uint16_t pulse_duration, uint16_t ch2_duration, uint16_t ch3_duration) ;
void Timer1_Init(void) ;
void ONEWIRE_RESET(void);
void ONEWIRE_WRITE_BIT(unsigned char x);
void ONEWIRE_READ_BIT(void);
