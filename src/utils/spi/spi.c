#include "spi.h"
#include "gpio.h"

// Implémentation des fonctions
void SPI1_Init(SPI_Time_Div time_div, uint8_t CPOL, uint8_t CPHA, uint8_t data_size) {
    // Activer l'horloge pour SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    // Configurer les GPIOs : SCK (PA5), MISO (PA6), MOSI (PA7)	
    initGpioX(GPIOA, 5, GPIO_MODE_AF_PP_50MHz);
    initGpioX(GPIOA, 6, GPIO_MODE_INPUT_PULL_UP_DOWN);
    GPIOA->ODR |= (1 << 6);  // Activer la pull-up sur PA6 (MISO)
    initGpioX(GPIOA, 7, GPIO_MODE_AF_PP_50MHz);

    // Configurer SPI1
    SPI1->CR1 = 0;
    SPI1->CR1 = SPI_MASTER_MODE | (time_div << 3) | (CPOL << 1) | (CPHA << 0);
    SPI1->CR1 |= (data_size == 16) ? SPI_16BIT_MODE : SPI_8BIT_MODE;
    SPI1->CR1 |= (1 << 9); //SSM
    SPI1->CR1 |= (1 << 8); //SSI
    
    // SPI1->CR2 = 0;
    // SPI1->CR2 |= (1 << 2);//SSOE

    SPI1->CR1 |= (1 << 6);//SPI enable
}

void SPI2_Init(SPI_Time_Div time_div, uint8_t CPOL, uint8_t CPHA, uint8_t data_size) {
    // Activer l'horloge pour SPI2
    RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN | RCC_APB2ENR_AFIOEN;

    // Configurer les GPIOs : SCK (PB13), MISO (PB14), MOSI (PB15)
    initGpioX(GPIOB, 13, GPIO_MODE_AF_PP_50MHz);
    initGpioX(GPIOB, 14, GPIO_MODE_INPUT_PULL_UP_DOWN);
    GPIOB->ODR |= (1 << 14);  // Activer la pull-up sur PB14 (MISO)
    initGpioX(GPIOB, 15, GPIO_MODE_AF_PP_50MHz);

    // Configurer SPI2
    SPI2->CR1 = SPI_MASTER_MODE | (time_div << 3) | (CPOL << 1) | (CPHA << 0);
    SPI2->CR1 |= (data_size == 16) ? SPI_16BIT_MODE : SPI_8BIT_MODE;
    SPI2->CR1 |= (1 << 9); //SSM
    SPI2->CR1 |= (1 << 8); //SSI
    
    SPI2->CR2 = 0;
    SPI2->CR2 |= (1 << 2);//SSOE

    SPI2->CR1 |= (1 << 6);//SPI enable
}

void SPI_ConfigCS(GPIO_TypeDef* port, uint16_t pin) {
    initGpioX(port, pin, GPIO_MODE_OUTPUT_PP_50MHz);
    port->BSRR = (1 << pin);
    /*uint8_t pin_position = (pin < 8) ? (pin * 4) : ((pin - 8) * 4);
    volatile uint32_t* cr = (pin < 8) ? &(port->CRL) : &(port->CRH);

    *cr &= ~(0xF << pin_position);
    *cr |= (0x3 << pin_position);  // Sortie push-pull, max 50MHz
    port->BSRR = (1 << pin);       // CS haut (inactif)*/
}

uint8_t SPI_Read(SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg) {
    uint8_t data;

    // Sélectionner CS (actif bas)
    cs_port->BSRR = (1 << (cs_pin + 16));

    // Envoyer le registre pour lecture
    while (!(SPIx->SR & SPI_SR_TXE));
    SPIx->DR = reg | 0x80;
    while (!(SPIx->SR & SPI_SR_RXNE));
    (void)SPIx->DR;  // Lecture fictive

    // Recevoir les données
    while (!(SPIx->SR & SPI_SR_TXE));
    SPIx->DR = 0xFF;  // Écriture fictive
    while (!(SPIx->SR & SPI_SR_RXNE));
    data = SPIx->DR;

    // Désélectionner CS
    cs_port->BSRR = (1 << cs_pin);

    return data;
}

void SPI_ReadN(SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg, uint8_t* buffer, uint16_t length) {
		uint16_t i = 0;
    // Activer CS (CS bas)
    cs_port->BSRR = (1 << (cs_pin + 16));

    // Envoyer l'adresse avec le bit de lecture
    while (!(SPIx->SR & SPI_SR_TXE)); // Attendre que le buffer de transmission soit vide
    SPIx->DR = reg | 0x80;            // Bit 7 haut pour lecture
    while (!(SPIx->SR & SPI_SR_RXNE)); // Attendre la réception
    (void)SPIx->DR;                   // Lecture fictive (dummy read)

    // Lire N octets
    for (i=0; i < length; i++) {
        while (!(SPIx->SR & SPI_SR_TXE)); // Buffer de transmission vide
        SPIx->DR = 0xFF;                  // Écriture fictive pour générer l'horloge
        while (!(SPIx->SR & SPI_SR_RXNE)); // Attendre les données reçues
        buffer[i] = SPIx->DR;             // Lire les données du buffer
    }

    while (SPIx->SR & SPI_SR_BSY); // Attendre la fin de la transmission
    // Désactiver CS (CS haut)
    cs_port->BSRR = (1 << cs_pin);
}

void SPI_Write(SPI_TypeDef* SPIx, GPIO_TypeDef* cs_port, uint16_t cs_pin, uint8_t reg, uint8_t data) {
    // Sélectionner CS (actif bas)
    cs_port->BSRR = (1 << (cs_pin + 16));

    // Envoyer le registre pour écriture
    while (!(SPIx->SR & SPI_SR_TXE));
    SPIx->DR = reg & 0x7F;
    while (!(SPIx->SR & SPI_SR_RXNE));
    (void)SPIx->DR;  // Lecture fictive

    // Envoyer les données
    while (!(SPIx->SR & SPI_SR_TXE));
    SPIx->DR = data;
    while (!(SPIx->SR & SPI_SR_RXNE));
    (void)SPIx->DR;  // Lecture fictive

    // Désélectionner CS
    cs_port->BSRR = (1 << cs_pin);
}
