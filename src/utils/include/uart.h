#ifndef UART_H
#define UART_H

#include "gpio.h"

// Structure pour la gestion du buffer circulaire
typedef struct {
    uint8_t* buffer;           // Buffer de données
    uint16_t size;             // Taille totale du buffer
    uint16_t writePtr;         // Pointeur d'écriture
    uint16_t readPtr;          // Pointeur de lecture
    uint16_t available;        // Espace disponible
} UART_FIFO_t;

// Codes d'état pour les opérations UART
typedef enum {
    UART_OK = 0,
    UART_ERROR,
    UART_BUSY,
    UART_BUFFER_FULL,
    UART_BUFFER_EMPTY
} UART_Status_t;

// Prototypes des fonctions
UART_Status_t USART2_init(uint32_t baudRate);
UART_Status_t USART2_SendChar(char c);
UART_Status_t USART2_SendString(const char* str);
UART_Status_t USART2_SendHexString(uint8_t value);
UART_Status_t USART2_ProcessCharFIFO(void);
UART_Status_t USART2_ProcessHexFIFO(void);
UART_Status_t USART2_WriteFIFO(uint8_t data);

// Définitions privées
#define UART_TXE_FLAG     (1<<7)  // Bit 7 de SR (TXE)
#define DEFAULT_FIFO_SIZE 256      // Taille par défaut du buffer

#endif // UART_H
