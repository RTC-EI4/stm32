#include "uart.h"
#include <stdio.h>

// Variables Globales statiques pour la FIFO
static UART_FIFO_t uartFifo;
static uint8_t fifoBuffer[DEFAULT_FIFO_SIZE];

UART_Status_t USART2_init(uint32_t baudRate) {
    uint32_t brrValue;
    RCC->APB2ENR |= (1 << 2);  // Activer l'horloge GPIOA
    RCC->APB1ENR |= (1 << 17); // Activer l'horloge USART2
        
    initGpioX(GPIOA, 2, GPIO_MODE_AF_PP_50MHz); // Configurer PA2 en mode alternatif push-pull 50MHz
    
    initGpioX(GPIOA, 3, GPIO_MODE_INPUT_FLOATING); // Configurer PA3 en entrée flottante

    brrValue = 36000000 / baudRate;
    USART2->BRR = brrValue; // Baudrate 9600, horloge APB1 36 MHz
    USART2->CR1 |= (1 << 2) | (1 << 3); // Activer le récepteur et l'émetteur
    USART2->CR1 |= (1 << 13); // Activer USART2

    // Initialisation de la FIFO
    uartFifo.buffer = fifoBuffer;
    uartFifo.size = DEFAULT_FIFO_SIZE;
    uartFifo.writePtr = 0;
    uartFifo.readPtr = 0;
    uartFifo.available = DEFAULT_FIFO_SIZE;

    return UART_OK;
}

UART_Status_t USART2_SendChar(char c) {
    // Attendre que le registre d'émission soit vide
    while (!(USART2->SR & UART_TXE_FLAG));
    USART2->DR = c;
    return UART_OK;
}

UART_Status_t USART2_SendString(const char* str) {
    while (*str)
        if (USART2_SendChar(*str++) != UART_OK) 
            return UART_ERROR;
    return UART_OK;
}

UART_Status_t USART2_SendHexString(uint8_t data) {
    char hexStr[3];
    sprintf(hexStr, "%02X", data);
    return USART2_SendString(hexStr);
}

UART_Status_t USART2_ProcessCharFIFO(void) {
    // Vérifier s'il y a des données à envoyer
    if (uartFifo.readPtr != uartFifo.writePtr) {
        USART2_SendChar(uartFifo.buffer[uartFifo.readPtr]);
        uartFifo.readPtr = (uartFifo.readPtr + 1) % uartFifo.size;
        uartFifo.available++;
        return UART_OK;
    }
    return UART_BUFFER_EMPTY;
}

UART_Status_t USART2_ProcessHexFIFO(void) {
    // Vérifier s'il y a des données à envoyer
    if (uartFifo.readPtr != uartFifo.writePtr) {
        USART2_SendHexString(uartFifo.buffer[uartFifo.readPtr]);
        uartFifo.readPtr = (uartFifo.readPtr + 1) % uartFifo.size;
        uartFifo.available++;
        return UART_OK;
    }
    return UART_BUFFER_EMPTY;
}

UART_Status_t USART2_WriteFIFO(uint8_t data) {
    if (uartFifo.available) {
        uartFifo.buffer[uartFifo.writePtr] = data;
        uartFifo.writePtr = (uartFifo.writePtr + 1) % DEFAULT_FIFO_SIZE;  // Rebouclage du pointeur d'écriture
        uartFifo.available--;
        return UART_OK;
    }
    return UART_BUFFER_FULL;
}
