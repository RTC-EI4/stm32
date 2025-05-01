#include <stdio.h>
#include <math.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>

#include "sound.h"
#include "stm32f10x.h"

#include "gpio.h"
#include "spi.h"
#include "uart.h"

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

#define DEBUG 0 

extern TaskHandle_t xSoundTaskHandle; // Declare xSoundTaskHandle
//------------------------------Defines------------------------------

#define PI2 6.28318530718f // Define 2 * PI

#define NOTIFY_HALF        (1U << 0)
#define NOTIFY_FULL        (1U << 1)
#define NOTIFY_SPI_DONE    (1U << 2)
#define DMA_CCR_EN        (1U << 0) // DMA channel enable bit

//------------------------------Global variables------------------------------
static volatile uint32_t g_nextFlashAddr;
static volatile uint32_t g_pagesRemaining;
static volatile bool     g_mute = false;

uint16_t sound_buffer[PAGE_SIZE] = {0}; // Buffer for sound data = 256 words of 16 bits -> 2 pages of 128 words
static uint8_t spi_rx_tmp[PAGE_SIZE]; // Temporary buffer for SPI reception
static const uint16_t mute_buffer[SOUND_BUFFER_SIZE] = {0}; // Buffer for silence = 256 words of 16 bits -> 2 pages of 128 words
static const uint8_t dummy_tx = 0xFF;
TaskHandle_t xSoundTaskHandle = NULL;

volatile uint8_t  refill_half = 0;  
//--------------------------------Prototypes--------------------------------
// SPI
void init_SPI1(SPI_Time_Div time_div, uint8_t data_size);
uint8_t SPI_ReadAddr(SPI_TypeDef* SPIx, uint8_t data);
uint8_t SPI_ReadJEDEC_ID(SPI_TypeDef* SPIx, GPIO_TypeDef* port, uint16_t pin);
void SPI_read_page(SPI_TypeDef* SPIx, GPIO_TypeDef* port, uint16_t pin, uint32_t addr, uint16_t *buff);
void SPI_read_page_DMA(uint32_t addr, uint16_t *buff);

//DMA
void init_DMA1_SPI1_RX(void);
void DMA1_Channel2_IRQHandler(void);
void init_DMA1_SPI1_TX(void);

void init_DMA1_CH4_PWM(void);
void DMA1_Channel4_IRQHandler(void);

// PWM
void init_TIM4_CH2_PWM(void);

/*******************************IMPLEMENTATION****************************************/
//------------------------------Public functions------------------------------

void setSoundState(SoundConfig cfg)
{
    // Stop DMA and timer first
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
    TIM4->CR1 &= ~TIM_CR1_CEN;
    
    switch (cfg.state) {
        case SOUND_MUTE:
            g_mute = true;
            g_nextFlashAddr = 0;
            g_pagesRemaining = 0;
            break;
            
        case SOUND_RESISTANCE:
            g_mute = false;
            g_nextFlashAddr = BYTES_PER_SOUND * (cfg.rValue + cfg.expValue*5);
            g_pagesRemaining = PAGES_PER_SOUND-1;
            break;
        case SOUND_E1:
            g_mute = false;
            g_nextFlashAddr = BYTES_PER_SOUND * (60+1); // E1
            g_pagesRemaining = (2*PAGES_PER_SOUND)-1;
            break;
        case SOUND_E2:
            g_mute = false;
            g_nextFlashAddr = BYTES_PER_SOUND * (60+2); // E2
            g_pagesRemaining = (2*PAGES_PER_SOUND)-1;
            break;
        case SOUND_E3:
            g_mute = false;
            g_nextFlashAddr = BYTES_PER_SOUND * (60+3); // E3
            g_pagesRemaining = (2*PAGES_PER_SOUND)-1;
            break;
        case SOUND_E4:
            g_mute = false;
            g_nextFlashAddr = BYTES_PER_SOUND * (60+4); // E4
            g_pagesRemaining = (2*PAGES_PER_SOUND)-1;
            break;
    }
    if (g_mute)
        DMA1_Channel4->CMAR  = (uint32_t)mute_buffer;
    else
        DMA1_Channel4->CMAR  = (uint32_t)sound_buffer;
    DMA1_Channel4->CNDTR = SOUND_BUFFER_SIZE;
    // Start DMA and timer
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void initSound_SPI1(void){
    init_SPI1(SPI_CLK_DIV_256, 8);
    SPI_ConfigCS(SPI1_CS_CONFIG); // Configure CS pin for SPI1
}

void initSound_DMA(void){
    init_DMA1_SPI1_RX(); // Initialize DMA2 for SPI1
    init_DMA1_SPI1_TX(); // Initialize DMA2 for SPI1 TX
    init_DMA1_CH4_PWM(); // Initialize DMA1_CH4 for PWM
}

void initSound_PWM(void){
    init_TIM4_CH2_PWM(); // Initialize PWM on TIM1
}
void stop_audio() {
    DMA1_Channel4->CCR &= ~DMA_CCR4_EN; // Stop playback DMA
    TIM4->CR1 &= ~TIM_CR1_CEN;         // Stop timer
    TIM4->CCR2 = 0;                    // Silence PWM output
}
void vSoundTask(void *pvParameters)
{
    uint32_t notification;
    xSoundTaskHandle = xTaskGetCurrentTaskHandle();
    
    for (;;) {
        // Wait for notification from DMA interrupt
        if (xTaskNotifyWait(0, NOTIFY_HALF | NOTIFY_FULL, &notification, portMAX_DELAY) == pdTRUE) {
            #if DEBUG
                if (notification & NOTIFY_HALF)
                    GPIOA->ODR &= ~(1 << 1);
                    USART2_SendString("\r\n[Half buffer]\r\n");
                if (notification & NOTIFY_FULL){
                    GPIOA->ODR |= (1 << 1);
                    USART2_SendString("\r\n[Half buffer]\r\n");
                    
                }
                GPIOA->ODR ^= (1 << 1);
            #endif

            // Si le son est désactivé, ne pas lire la mémoire flash
            if (g_mute) 
                continue;

            // Si le nombre de pages restantes est 0, on arrête la lecture
            if (g_pagesRemaining == 0){
                // Stop DMA and timer first
                DMA1_Channel4->CCR &= ~DMA_CCR4_EN;
                TIM4->CR1 &= ~TIM_CR1_CEN;
                DMA1_Channel4->CMAR  = (uint32_t)mute_buffer;
                DMA1_Channel4->CNDTR = SOUND_BUFFER_SIZE;
                g_mute = true;
                DMA1_Channel4->CCR |= DMA_CCR4_EN;
								TIM4->CR1 |= TIM_CR1_CEN;
                #if DEBUG
                    USART2_SendString("FIN du SON!!\r\n");
                #endif
                continue;
            }
            #if DEBUG
                USART2_SendString("\r\nRemains:\r\n");
                USART2_SendHexString(g_pagesRemaining); // Send the string over UART
                USART2_SendString("\r\n"); // New line for readability
            #endif
            // Remplir la moitié du tampon audio avec la mémoire flash
            if (notification & NOTIFY_HALF) {
                SPI_read_page_DMA(g_nextFlashAddr, &sound_buffer[0]);
                g_nextFlashAddr += PAGE_SIZE;
                g_pagesRemaining--;
            }
            
            if (notification & NOTIFY_FULL) {
                SPI_read_page_DMA(g_nextFlashAddr, &sound_buffer[HALF_SOUND_BUFFER_SIZE]);
                g_nextFlashAddr += PAGE_SIZE;
                g_pagesRemaining--;
            }
        }
    }
}

void vInit_soundTasks(void){
    int i = 0;
    #if DEBUG
        USART2_init(9600);
        USART2_SendString("\r\n[System initialized]\r\n");
    #endif
    for ( i = 0; i < PAGE_SIZE; i++)
        sound_buffer[i] = 1125;
    initSound_SPI1();
    initSound_PWM();
    initSound_DMA();
    #if DEBUG
        if(SPI_ReadJEDEC_ID(SPI1, GPIOA, 4) == 1) {
            USART2_SendString("\r\n[JEDEC ID OK]\r\n");
        } else {
            USART2_SendString("\r\n[JEDEC ID ERROR]\r\n");
        }
    #endif
    DMA1_Channel4->CCR |= DMA_CCR4_EN;
    TIM4->CR1 |= TIM_CR1_CEN;
    #if DEBUG
        USART2_SendString("[Sound playback started]\r\n");
        fillAudioBuffer(); // ramp from PWM_MIN to PWM_MAX
        USART2_SendString("\r\n[System initialized]\r\n");
    #endif
    xTaskCreate(vSoundTask, "Sound Task", 128, NULL, 1, NULL);
}

//------------------------------Private functions------------------------------

//============================= SPI =============================
void init_SPI1(SPI_Time_Div time_div, uint8_t data_size) {
    // Activer l'horloge pour SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN |
                    RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_AFIOEN;

    // Configurer les GPIOs : SCK (PA5), MISO (PA6), MOSI (PA7)	
    initGpioX(GPIOA, 5, GPIO_MODE_AF_PP_50MHz);
    initGpioX(GPIOA, 6, GPIO_MODE_INPUT_PULL_UP_DOWN);
    GPIOA->ODR |= (1 << 6);  // Activer la pull-up sur PA6 (MISO)
    initGpioX(GPIOA, 7, GPIO_MODE_AF_PP_50MHz);

    // Configurer SPI1
    SPI1->CR1 = 0;
    // Configurer la fréquence d'horloge SPI : fpclk / time_div (BR[2:0])
    SPI1->CR1 = SPI_CR1_MSTR | // Mode maître
                (time_div << 3) |  // Contrôle du taux de transfert (BR[2:0]) -> 7<<3 = 256
                ((data_size == 16) ? SPI_CR1_DFF : 0) | // Format de trame de données -> 8 bits
                SPI_CR1_SSM | // SSM = 1 (gestion logicielle du CS)
                SPI_CR1_SSI; // SSI = 1 (CS interne activé)
    
    SPI1->CR2 = 0;
    SPI1->CR2 = SPI_CR2_RXDMAEN | // Activer le DMA pour la réception
               SPI_CR2_TXDMAEN; // Activer le DMA pour la transmission

    SPI1->CR1 |= SPI_CR1_SPE; // Activer SPI1
}

uint8_t SPI_ReadAddr(SPI_TypeDef* SPIx, uint8_t data) {
    // while (!(SPI1->SR & SPI_SR_TXE));
    SPIx->DR = data;
    while (!(SPIx->SR & SPI_SR_RXNE));
    return SPIx->DR;
}

uint8_t SPI_ReadJEDEC_ID(SPI_TypeDef* SPIx, GPIO_TypeDef* port, uint16_t pin) {
    uint8_t id[3];

    port->BSRR = (1 << (pin + 16)); // CS low
    SPI_ReadAddr(SPIx,0x9F); // JEDEC ID command
    id[0] = SPI_ReadAddr(SPIx,0xFF);
    id[1] = SPI_ReadAddr(SPIx,0xFF);
    id[2] = SPI_ReadAddr(SPIx,0xFF);
    port->BSRR = (1 << pin); // CS high

    // Reg W25Q64: 0xEF 0x40 0x17
    #if DEBUG
        USART2_SendString("\r\n[JEDEC ID: ");
        USART2_SendHexString(id[0]); USART2_SendString(" ");
        USART2_SendHexString(id[1]); USART2_SendString(" ");
        USART2_SendHexString(id[2]); USART2_SendString("]\r\n");
    #endif
    return (id[0] == 0xEF && id[1] == 0x40 && id[2] == 0x17);
}

void SPI_read_page(SPI_TypeDef* SPIx, GPIO_TypeDef* port, uint16_t pin, uint32_t addr, uint16_t *buff) {
    uint16_t i;
    uint8_t lo, hi;
    #if DEBUG == 1
        char buffer[20];
        USART2_SendString("\r\n[Read page]\r\n");
        USART2_SendString("\r[Addresse:0x\r");
        USART2_SendHexString((addr >> 16) & 0xFF);  // bits 23-16
        USART2_SendHexString((addr >> 8) & 0xFF);   // bits 15-8
        USART2_SendHexString(addr & 0xFF);        // bits 7-0
        USART2_SendString("]\r\n");
    #endif

    port->BSRR = (1 << (pin + 16)); // Set CS low

    SPI_ReadAddr(SPIx, 0x03); // Read
    SPI_ReadAddr(SPIx, (addr >> 16) & 0xFF); // Addresses bits 23-16
    SPI_ReadAddr(SPIx, (addr >> 8) & 0xFF); // Addresses bits 15-8
    SPI_ReadAddr(SPIx, addr & 0xFF); // Addresses bits 7-0

    for (i = 0; i < PAGE_SIZE/2; i++) {
        lo = SPI_ReadAddr(SPIx, 0xFF); // Lire la première partie
        hi = SPI_ReadAddr(SPIx, 0xFF); // Lire la seconde partie
        buff[i] = lo + 128*hi; // Combiner en un échantillon de 14 bits
        #if DEBUG == 1      
            sprintf(buffer, "%x v: %d",addr+i, buff[i]);
            USART2_SendString(buffer); 
            USART2_SendString("\r\n"); 
        #endif
    }
    port->BSRR = (1 << pin); // Set CS high
}

void SPI_read_page_DMA(uint32_t addr, uint16_t *buff){
    uint16_t i;
    uint8_t lo, hi;

    // 1) CS bas
    SPI_GPIO_PORT->BSRR = (1 << (SPI_CS_PIN + 16));

    (void)SPI1->SR;
    (void)SPI1->DR;

    SPI_ReadAddr(SPI1, 0x03);
    SPI_ReadAddr(SPI1, (addr >> 16) & 0xFF);
    SPI_ReadAddr(SPI1, (addr >> 8) & 0xFF);
    SPI_ReadAddr(SPI1, addr & 0xFF);

    DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR3_EN;

    DMA1_Channel2->CMAR = (uint32_t)spi_rx_tmp;  // Definir le buffer de réception
    DMA1_Channel2->CNDTR = PAGE_SIZE;
    DMA1_Channel3->CMAR = (uint32_t)&dummy_tx;  // Definir le dummy de transmission
    DMA1_Channel3->CNDTR = PAGE_SIZE;

    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2; // Nettoyer les flags de transfert
    DMA1->IFCR = DMA_IFCR_CTCIF3 | DMA_IFCR_CHTIF3; //  Nettoyer les flags de transfert

    // DMA1_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_TCIE | DMA_CCR_PL_1;
    // DMA1_Channel3->CCR = DMA_CCR_DIR | DMA_CCR_TCIE | DMA_CCR_PL_1;
    
    SPI1->CR2 |= SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN; // Enable RX and TX DMA
    DMA1_Channel2->CCR |= DMA_CCR2_EN; // Activer le canal DMA après la configuration
    DMA1_Channel3->CCR |= DMA_CCR3_EN; // Activer le canal DMA après la configuration

    // 5) Démarre la génération d'horloges (premier octet)
    SPI1->DR = 0xFF;
    
    // 6) Attend la fin du transfert via notification
    // Notification faite dans l'IRQ du DMA1_Channel2
    xTaskNotifyWait(0, NOTIFY_SPI_DONE, NULL, portMAX_DELAY);
    // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CTCIF3; // Nettoyer les flags de transfert
    
    // 7) Désactive SPI DMA et TXE IRQ
    DMA1_Channel2->CCR &= ~DMA_CCR2_EN;
    DMA1_Channel3->CCR &= ~DMA_CCR3_EN;
    SPI1->CR2     &= ~(SPI_CR2_RXDMAEN | SPI_CR2_TXDMAEN);

    // 8) CS haut
    SPI_GPIO_PORT->BSRR = (1 << SPI_CS_PIN);

    // 9) Traiter les données reçues
    for (i = 0; i < (PAGE_SIZE/2); i++) {
        lo = spi_rx_tmp[2*i];
        hi = spi_rx_tmp[2*i+1];
        buff[i] = lo + (uint16_t)(128U * hi);
    }
}

//============================= DMA =============================
    
void init_DMA1_SPI1_RX(void){
    // Activer les horloges
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    // Configuration du Canal 2 du DMA2 (mémoire SPI → buffer dans la RAM)
    DMA1_Channel2->CCR = 0;
    DMA1_Channel2->CNDTR = PAGE_SIZE;  // Nombre de données à transférer (256 octets)
    DMA1_Channel2->CPAR = (uint32_t)&SPI1->DR;  // Adresse périphérique (SPI1 DR)
    DMA1_Channel2->CMAR = (uint32_t)&spi_rx_tmp[0];  // Adresse mémoire (buffer dans la RAM)

    /* Configuration du registre CCR2 (configuration du canal DMA) :
        - MEM2MEM = 0 (transfert mémoire à mémoire)
        - PL = 01 (priorité basse)
        - MSIZE = 00 (8 bits) // Buffer Temporaire
        - PSIZE = 00 (8 bits)
        - MINC = 1 (mode incrément mémoire)
        - PINC = 0 (mode sans incrément périphérique)
        - CIRC = 0 (mode non circulaire)
        - DIR = 0 (lecture depuis le périphérique)
        - TEIE = 0 (désactiver l'interruption d'erreur de transfert)
        - HTIE = 0 (désactiver l'interruption de transfert à mi-parcours)
        - TCIE = 1 (activer l'interruption de transfert complet)
        - EN = 1 (activer le canal) (activé à la fin)
    */
    DMA1_Channel2->CCR |= DMA_CCR2_MINC |  // Incrément de l'adresse mémoire
                          DMA_CCR2_TCIE |   // Activer l'interruption à la fin du transfert
                          DMA_CCR3_PL; // Priorité haute

    // Configuration de l'IRQ pour le DMA1_Channel2
    NVIC_SetPriority(DMA1_Channel2_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel2_IRQn);

    //Enable sur page_read
    // DMA1_Channel2->CCR |= DMA_CCR2_EN;
}
void DMA1_Channel2_IRQHandler(void)
{
    BaseType_t xWoken = pdFALSE;
    if (DMA1->ISR & DMA_ISR_TCIF2) {
        DMA1->IFCR = DMA_IFCR_CTCIF2 | DMA_IFCR_CHTIF2; // Nettoyer les flags de transfert
        // vTaskNotifyGiveFromISR(xSoundTaskHandle, &xWoken); // NOTIFY_SPI_DONE
        xTaskNotifyFromISR(xSoundTaskHandle, NOTIFY_SPI_DONE, eSetBits, &xWoken);
    }
    portYIELD_FROM_ISR(xWoken);
}

void init_DMA1_SPI1_TX(void){
    RCC->AHBENR |= RCC_AHBENR_DMA1EN;

    DMA1_Channel3->CCR = 0;
    DMA1_Channel3->CNDTR = PAGE_SIZE;  // Nombre de données à transférer (256 octets)
    DMA1_Channel3->CPAR = (uint32_t)&SPI1->DR;  // Adresse périphérique (SPI1 DR)
    DMA1_Channel3->CMAR = (uint32_t)&dummy_tx;  // Adresse mémoire (buffer dans la RAM)

    /* Configuration du registre CCR3 (configuration du canal DMA) :
        - MEM2MEM = 0 (transfert mémoire à mémoire)
        - PL = 00 (priorité basse)
        - MSIZE = 00 (8 bits)
        - PSIZE = 00 (8 bits)
        - MINC = 0 (mode sans incrément mémoire)
        - PINC = 0 (mode sans incrément périphérique)
        - CIRC = 1 (mode circulaire)
        - DIR = 1 (lecture depuis la mémoire)
        - TEIE = 0 (désactiver l'interruption d'erreur de transfert)
        - HTIE = 0 (désactiver l'interruption de transfert à mi-parcours)
        - TCIE = 0 (désactiver l'interruption de transfert complet)
        - EN = 1 (activer le canal) (activé à la fin)
    */
    DMA1_Channel3->CCR |= DMA_CCR3_CIRC |  // Mode circulaire
                          DMA_CCR3_DIR; // Transfert de mémoire vers périphérique
    // Enable sur page_read
    // DMA1_Channel3->CCR |= DMA_CCR3_EN;

}

void init_DMA1_CH4_PWM(void) {
    // Activer les horloges
    RCC->AHBENR  |= RCC_AHBENR_DMA1EN;
    // RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // AF remap & SWJ config

    // Canal4 du DMA1  : mémoire -> périphérique (TIM4_CCR2)
    DMA1_Channel4->CCR = 0;
    DMA1_Channel4->CPAR = (uint32_t)&TIM4->CCR2; // Adresse périphérique (TIM4 CCR2)
    DMA1_Channel4->CMAR = (uint32_t)sound_buffer; // Adresse de mémoire (audio_buffer[0])
    DMA1_Channel4->CNDTR = SOUND_BUFFER_SIZE; // Nombre de données à transférer (128 échantillons de 16 bits)
    /* Configuration du registre CCR (registre de configuration du canal DMA) :
        - MEM2MEM = 0 (transfert mémoire à mémoire)
        - PL = 00 (priorité basse)
        - MSIZE = 01 (16 bits)
        - PSIZE = 01 (16 bits)
        - MINC = 1 (mode incrément mémoire)
        - PINC = 0 (mode sans incrément périphérique)
        - CIRC = 1 (mode circulaire)
        - DIR = 1 (lecture depuis la mémoire)
        - TEIE = 0 (désactiver l'interruption d'erreur de transfert)
        - HTIE = 1 (activer l'interruption de transfert à mi-parcours)
        - TCIE = 1 (activer l'interruption de transfert complet)
        - EN = 1 (activer le canal) (activé à la fin)
    */
    DMA1_Channel4->CCR |= DMA_CCR4_MINC |  // Incrément de l'adresse mémoire
                          DMA_CCR4_MSIZE_0 | DMA_CCR4_PSIZE_0 | // Taille de 16 bits pour mémoire et périphérique
                          DMA_CCR4_CIRC |  // Mode circulaire
                          DMA_CCR4_DIR |   // Transfert de mémoire vers périphérique
                          DMA_CCR4_HTIE |  // Activer l'interruption à mi-parcours
                          DMA_CCR4_TCIE;   // Activer l'interruption à la fin du transfert

    // DMA1_Channel4->CCR |= DMA_CCR4_EN; // Activer le canal DMA1_Channel4

    NVIC_SetPriority(DMA1_Channel4_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void DMA1_Channel4_IRQHandler(void)
{
    BaseType_t xWoken = pdFALSE;
    uint32_t isr = DMA1->ISR;

    if (isr & DMA_ISR_HTIF4) {
        DMA1->IFCR = DMA_IFCR_CHTIF4;
        // vTaskNotifyGiveFromISR(xSoundTaskHandle, &xWoken);
        if(xSoundTaskHandle != NULL)
            xTaskNotifyFromISR(xSoundTaskHandle, NOTIFY_HALF, eSetBits, &xWoken);
    }
    if (isr & DMA_ISR_TCIF4) {
        DMA1->IFCR = DMA_IFCR_CTCIF4;
        // vTaskNotifyGiveFromISR(xSoundTaskHandle, &xWoken);
        if(xSoundTaskHandle != NULL)
            xTaskNotifyFromISR(xSoundTaskHandle, NOTIFY_FULL, eSetBits, &xWoken);
    }

    portYIELD_FROM_ISR(xWoken);
}

//============================= PWM =============================
void init_TIM4_CH2_PWM(void) {
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN   // AF remap & SWJ config
                 | RCC_APB2ENR_IOPBEN; // GPIOB
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;  // Bit 2 = TIM4

    initGpioX(GPIOB, 7, GPIO_MODE_AF_PP_2MHz); // Configurer PB7 comme fonction alternative pour TIM4_CH2

    // Configurer TIM4 pour générer du PWM
    TIM4->PSC = 0; // Pas de prescaler
    TIM4->ARR = 2249U; // Fréquence de 32 kHz (72 MHz / 2249+1 = 32 kHz)
    TIM4->CR1 |= TIM_CR1_ARPE; // Activer le préchargement

    // Mode PWM sur CH2 (OC2PE + OC2M[6:4] = 110 → PWM1)
    //    OC2PE = bit 3 of CCMR1, OC2M bits [6:4]
    // TIM4->CCMR1 &= ~((7 << 12) | (1 << 11)); // Clear OC2M[2:0] and OC2PE
    TIM4->CCMR1 |= (6 << 12) | (1 << 11);       // Set OC2M = 110 (PWM1), OC2PE = 1

    TIM4->CCER |= TIM_CCER_CC2E; // Activer le canal 2 (PB7)
    TIM4->CCR2 = BASE_PWM_VALUE; // Valeur initiale du PWM (cycle de travail 50%)

    // Activer le DMA pour le canal 2 de TIM4
    TIM4->DIER = 0; // Clear all DMA interrupts
    // TIM4->DIER |= TIM_DIER_TDE; // Activer le DMA pour CC2 (DMA request on update event)
    TIM4->DIER = TIM_DIER_CC2DE |   // request at OC2 match
                TIM_DIER_UDE;    // request at ARR rollover

    // Activer le timer sur SetSoundState
    // TIM4->CR1 |= TIM_CR1_CEN;
}

//============================= END =============================
