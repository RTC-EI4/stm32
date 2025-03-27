#ifndef SPI_H
#define SPI_H

#include "stm32f10x.h"
#include <stdint.h>

// Définitions pour la configuration
#define SPI_ENABLE          ((uint16_t)0x0040)
#define SPI_MASTER_MODE     ((uint16_t)0x0004)
#define SPI_8BIT_MODE      ((uint16_t)0x0000)
#define SPI_16BIT_MODE     ((uint16_t)0x0800)
#define SPI_SSOE           ((uint16_t)0x0004)
#define SPI_SSM            ((uint16_t)0x0001)

// Enums pour la sélection des configurations
typedef enum {
    SPI_CLK_DIV_2   = 0,
    SPI_CLK_DIV_4   = 1,
    SPI_CLK_DIV_8   = 2,
    SPI_CLK_DIV_16  = 3,
    SPI_CLK_DIV_32  = 4,
    SPI_CLK_DIV_64  = 5,
    SPI_CLK_DIV_128 = 6,
    SPI_CLK_DIV_256 = 7
} SPI_Time_Div;

// Fonctions d'initialisation
void SPI1_Init(SPI_Time_Div time_div, uint8_t CPOL, uint8_t CPHA, uint8_t data_size);
void SPI2_Init(SPI_Time_Div time_div, uint8_t CPOL, uint8_t CPHA, uint8_t data_size);

// Configuration du CS
void SPI_ConfigCS(GPIO_TypeDef* port, uint16_t pin);

// Fonctions de lecture et d'écriture
uint8_t SPI_Read(SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg);
void SPI_ReadN(SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg, uint8_t* buffer, uint16_t length);
void SPI_Write(SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg, uint8_t data);

#endif // SPI_H
