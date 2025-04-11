#include <stdlib.h>
#include <stdint.h>

#include "stm32f10x.h"

#include "FreeRTOS.h"
#include "task.h"

#include "gpio.h"

#include "neopixel.h"

/*

    Neopixel task

*/

char seq = 0;
uint8_t colorsNeo[24] = {NEOPIXEL_COLOR_RED, NEOPIXEL_COLOR_GREEN, NEOPIXEL_COLOR_BLUE, NEOPIXEL_COLOR_WHITE, NEOPIXEL_COLOR_BLACK, NEOPIXEL_COLOR_RED, NEOPIXEL_COLOR_GREEN, NEOPIXEL_COLOR_BLUE, NEOPIXEL_COLOR_WHITE, NEOPIXEL_COLOR_BLACK, NEOPIXEL_COLOR_RED, NEOPIXEL_COLOR_GREEN, NEOPIXEL_COLOR_BLUE, NEOPIXEL_COLOR_WHITE, NEOPIXEL_COLOR_BLACK, NEOPIXEL_COLOR_RED, NEOPIXEL_COLOR_GREEN, NEOPIXEL_COLOR_BLUE, NEOPIXEL_COLOR_WHITE, NEOPIXEL_COLOR_BLACK, NEOPIXEL_COLOR_RED, NEOPIXEL_COLOR_GREEN, NEOPIXEL_COLOR_BLUE, NEOPIXEL_COLOR_WHITE};

void task_Neopixel(void* params_p) {
    // Init resources
    initTimer2_neo(); // Init the timer 2 for the neopixel
    initDMA1_neo(); // Init the DMA 1 for the neopixel

    // Set the pin to output push-pull mode with 50MHz speed
    initGpioX(GPIOB, 10, GPIO_MODE_OUTPUT_PP_50MHz); // Set the pin to output push-pull mode with 50MHz speed
    GPIOB->BSRR = GPIO_BSRR_BS10; // Set the pin to high
    vTaskDelay(10); // Wait to let the LED initialize

    // Task main loop
    volatile uint8_t ledIndex = 1;
    while(1) {
        /*
        
            Reset sequence
        
        */

        // Set the pin to output push-pull mode with 50MHz speed
        initGpioX(GPIOB, 10, GPIO_MODE_OUTPUT_PP_50MHz); // Set the pin to output push-pull mode with 50MHz speed
        GPIOB->BSRR = GPIO_BSRR_BR10; // Set the pin to low

        vTaskDelay(1); // Wait 100us

        /*

            Program the neopixel

        */

        // Calc and send the neopixel data
        setNeopixelData(colorsNeo, ledIndex++);

        // Wait
        vTaskDelay(1000);

        // Loop the array
        if(ledIndex > 24) {
            // Fill colorsNeo with all black
            if(seq++ == 0) for(uint8_t i = 0; i < 24; i++) colorsNeo[i] = NEOPIXEL_COLOR_BLACK;
            else { // Fill colorsNeo as before
                for(uint8_t i = 0; i < 24; i++) colorsNeo[i] = NEOPIXEL_COLOR_RED + (i % 5);
                seq = 0;
            }

            ledIndex = 1; // Reset the index
        }
    }
}

/*

    Functions

*/

int8_t setNeopixelData(uint8_t* colors, uint8_t count) {
    // Malloc the neopixel data array
    uint8_t* neopixelData = (uint8_t*) malloc((count * 24 + 1) * sizeof(uint8_t)); // Allocate memory for the neopixel data (24 bits per pixel)
    if(neopixelData == NULL) return -1; // Error, not enough memory
    
    // Fill the neopixel data with the colors
    for(uint8_t i = 0; i < count; i++) {
        uint32_t color = 0;

        // Convert the color to the neopixel format (GRB)
        if(colors[i] == NEOPIXEL_COLOR_RED) color = 0x000500; // Red
        else if(colors[i] == NEOPIXEL_COLOR_GREEN) color = 0x050000; // Green
        else if(colors[i] == NEOPIXEL_COLOR_BLUE) color = 0x000005; // Blue
        else if(colors[i] == NEOPIXEL_COLOR_WHITE) color = 0x050505; // White
        else if(colors[i] == NEOPIXEL_COLOR_BLACK) color = 0x000000; // Black
        else color = 0x000000; // Default to black

        // Fill the neopixel data with the signal duration for each bit
        for(uint8_t j = 0; j < 24; j++) {
            if((color >> (23 - j)) & 0x1) { // If the bit is 1
                neopixelData[i * 24 + j] = 7; // Set the duration to 0.8us
            } else { // If the bit is 0
                neopixelData[i * 24 + j] = 3; // Set the duration to 0.4us
            }
        }
    }

    // Set the last byte to 0 (not used)
    neopixelData[count * 24] = 0; // Set the last byte to 0 (not used)

    // Set pin mode
    initGpioX(GPIOB, 10, GPIO_MODE_AF_PP_50MHz);

    // Set the DMA memory address to the neopixel data and the number of data to transfer
    DMA1_Channel1->CMAR = (uint32_t) neopixelData; // Set the memory address to the neopixel data
    DMA1_Channel1->CNDTR = count * 24 + 1; // Set the number of data to transfer (24 bits per pixel)

    // Enable the DMA channel
    DMA1_Channel1->CCR |= DMA_CCR1_EN; // Enable the channel

    // Enable the timer
    TIM2->EGR |= TIM_EGR_UG; // Generate an update event to preload the registers
    TIM2->CR1 |= TIM_CR1_CEN; // Enable the counter

    return 1;
}

/*

    Hidden functions

*/

// Init the timer 2 chanel 3 in PWM mode, it will be controled by DMA
static void initTimer2_neo(void) {
    // Init timer 2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; // Enable the timer 2 clock

    // Enable the AFIO clock
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN; // Enable the AFIO clock

    // Remap the timer 2 to the GPIOB pin 10 (PB10)
    AFIO->MAPR |= AFIO_MAPR_TIM2_REMAP_0 | AFIO_MAPR_TIM2_REMAP_1; // Set the timer 2 remap bits to 0b11 (remap to PB10)

    /* CR1 config:
        - CKD (clock division) = 0 (no division)
        - ARPE (ARR buffering) = 1 (buffered)
        - CMS = 0 (edge aligned mode)
        - DIR = 0 (up counting)
        - OPM = 0 (not one pulse mode)
        - URS = 1 (overflow event only) TODO verify
        - UDIS = 0 (update event enabled)
        - CEN = 1 (counter enabled) (set at the end)
    */

    TIM2->CR1 = 0; // Reset the timer control register 1
    TIM2->CR1 |= TIM_CR1_ARPE;
    TIM2->CR1 |= TIM_CR1_URS;

    /* CR2 config:
        - TI1S = 0 (not used)
        - MMS = 0 (reset on update event)
        - CCDS = 0 TODO verify
    */

    TIM2->CR2 = 0; // Reset the timer control register 2

    /* SMCR : not used */
    TIM2->SMCR = 0; // Reset the timer slave mode control register

    /* DIER (DMA enable register) config:
        - TDE = 1 (enable update event for DMA)
        - CC4DE = 0 (not used)
        - CC3DE = 1 (enable DMA request for chanel 3)
        - CC2DE = 0 (not used)
        - CC1DE = 0 (not used)
        - UDE = 0 (not used)
        - TIE = 1 (not used)
        - CC4IE = 0 (not used)
        - CC3IE = 1 (enable interrupt for chanel 3)
        - CC2IE = 0 (not used)
        - CC1IE = 0 (not used)
        - UIE = 0 (not used)
    */

    TIM2->DIER = 0; // Reset the timer DMA enable register
    TIM2->DIER |= TIM_DIER_TDE; // Enable update event for DMA
    TIM2->DIER |= TIM_DIER_CC3DE; // Enable DMA request for chanel 3
    TIM2->DIER |= TIM_DIER_TIE; // Enable event for interrupt
    TIM2->DIER |= TIM_DIER_CC3IE; // Enable interrupt for chanel 3

    /* SR (status register): (set by hardware) */
    TIM2->SR = 0; // Reset the timer status register

    /* EGR (configure the event generation register): (set by hardware) */
    TIM2->EGR = 0; // Reset the timer event generation register

    /* CCMR1 : (capture/compare mode register 1 and PWM activation) config: (not used) */
    TIM2->CCMR1 = 0; // Reset the capture/compare mode register 1

    /* CCMR2 : (capture/compare mode register 2 and PWM activation) config:
        - OC4CE = 0 (not used)
        - OC4M = 0 (not used)
        - OC4PE = 0 (not used)
        - OC4FE = 0 (not used)
        - OC4S = 0 (not used)
        - OC3CE = 0 (not used)
        - OC3M = 0b110 (PWM mode 1)
        - OC3PE = 1 (preload enable)
        - OC3FE = 0 (not used) TODO swtich to 1
        - OC3S = 0 (channel output 3 selection)
    */

    TIM2->CCMR2 = 0; // Reset the capture/compare mode register 2
    TIM2->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Set the OC3M bits to 0b110 (PWM mode 1)
    TIM2->CCMR2 |= TIM_CCMR2_OC3PE; // Set the OC3PE bit to 1
    TIM2->CCMR1 |= TIM_CCMR1_OC1FE; // Set the OC1FE bit to 1 (fast mode)
    
    /* CCER :
        - CC4P = 0 (not used)
        - CC4E = 0 (not used)
        - CC3P = 0 (active high)
        - CC3E = 1 (enable output compare 3)
        - CC2P = 0 (not used)
        - CC2E = 0 (not used)
        - CC1P = 0 (not used)
        - CC1E = 0 (not used)
    */

    TIM2->CCER = 0; // Reset the capture/compare enable register
    TIM2->CCER |= TIM_CCER_CC3E; // Set the CC3E bit to 1 (enable output compare 3)

    /* PSC (prescaler) */
    TIM2->PSC = 6; // Base clock id 72MHz, prescaler (6 + 1) ~= 10.2MHz or ~= 97.2ns period
    
    /* ARR (auto reload register) */
    TIM2->ARR = 11; // PWN cycle in us, (11 + 1) * 97.2ns = 1.166us ~= 1.2us

    /* CCRx (capture/compare register x) : not used */
    TIM2->CCR1 = 0; // Reset the capture/compare register 1
    TIM2->CCR2 = 0; // Reset the capture/compare register 2
    TIM2->CCR3 = 0; // Reset the capture/compare register 3, (default value, will be set by DMA)
    TIM2->CCR4 = 0; // Reset the capture/compare register 4

    /* DCR (DMA control register) config:
        - DBL = 0 (1 data transfer)
        - DBA = 15 (0b1111) (write to the CCR3 register)
    */

    TIM2->DCR = 0; // Reset the DMA control register
    TIM2->DCR |= TIM_DCR_DBA_0 | TIM_DCR_DBA_1 | TIM_DCR_DBA_2 | TIM_DCR_DBA_3; // Set the DBA bits to 0b1111 (write to the CCR3 register)

    // Preload data into the shadow registers
    //TIM2->EGR |= TIM_EGR_UG; // Generate an update event to preload the registers

    // Enable counter
    // TIM2->CR1 |= TIM_CR1_CEN; // Enable the counter
}

// Init the DMA 1 to chanel 1 to transfer the data to the timer 2 chanel 3 and control the PWM ratio
static void initDMA1_neo(void) {
    // Init DMA 1 clock
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; // Enable the DMA 1 clock

    /* CCR (DMA chanel config register) config:
        - MEM2MEM = 0 (memory to memory transfer)
        - PL = 11 (high priority)
        - MSIZE = 00 (8 bits)
        - PSIZE = 01 (16 bits)
        - MINC = 1 (memory increment mode)
        - PINC = 0 (peripheral increment mode)
        - CIRC = 0 (not circular mode)
        - DIR = 1 (memory to peripheral transfer)
        - TEIE = 0 (not used)
        - HTIE = 0 (not used)
        - TCIE = 1 (transfer complete interrupt enable)
        - EN = 1 (enable the channel) (set at the end)
    */

    DMA1_Channel1->CCR = 0; // Reset the DMA chanel config register
    DMA1_Channel1->CCR |= DMA_CCR1_PL_0 | DMA_CCR1_PL_1; // Set the PL bits to 11 (high priority)
    DMA1_Channel1->CCR |= DMA_CCR1_PSIZE_0; // Set the PSIZE bits to 01 (16 bits)
    DMA1_Channel1->CCR |= DMA_CCR1_MINC; // Set the MINC bit to 1 (memory increment mode)
    DMA1_Channel1->CCR |= DMA_CCR1_DIR; // Set the DIR bit to 1 (memory to peripheral transfer)
    DMA1_Channel1->CCR |= DMA_CCR1_TCIE; // Set the TCIE bit to 1 (transfer complete interrupt enable)

    /* CPAR (peripheral address) */
    DMA1_Channel1->CPAR = (uint32_t) &TIM2->DMAR; // Set the peripheral address to the TIM2 DMAR register

    // Enable DMA IRQ
    NVIC_EnableIRQ(DMA1_Channel1_IRQn); // Enable the DMA channel 1 interrupt
    NVIC_SetPriority(DMA1_Channel1_IRQn, 0); // Set the DMA channel 1 interrupt priority to 0

    // DMA1_Channel1->CMAR = &neopixelData[0]; // Set the memory address to the neopixelData array
    // DMA1_Channel1->CNDTR = 5; // Set the number of data to transfer (5 bytes)

    /* CCR : enable the channel */
    // DMA1_Channel1->CCR |= DMA_CCR1_EN; // Enable the channel
}

/*

    Interrupt handlers

*/

void DMA1_Channel1_IRQHandler() {
    // Stop the PWM timer
    TIM2->CR1 &= ~TIM_CR1_CEN; // Disable the counter

    // Change pin mod and set to high
    initGpioX(GPIOB, 10, GPIO_MODE_OUTPUT_PP_50MHz); // Set the pin to output push-pull mode with 50MHz speed
    GPIOB->BSRR = GPIO_BSRR_BS10; // Set the pin to high

    // Disable the DMA channel
    DMA1_Channel1->CCR &= ~DMA_CCR1_EN; // Disable the channel

    // Free the neopixel data array
    free((void*) DMA1_Channel1->CMAR); // Free the neopixel data array
    DMA1_Channel1->CMAR = 0; // Set the memory address to 0 (not used)
    DMA1_Channel1->CNDTR = 0; // Set the number of data to transfer to 0 (not used)

    // Clear the transfer complete interrupt flag
    DMA1->IFCR |= DMA_IFCR_CTCIF1; // Clear the transfer complete interrupt flag
}
