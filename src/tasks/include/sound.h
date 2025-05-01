#ifndef SOUND_H
#define SOUND_H

#include "stm32f10x.h"
#include "FreeRTOS.h"
#include "task.h"

#include <stdint.h>

#define SPI_GPIO_PORT GPIOA // GPIO port for SPI1
#define SPI_CS_PIN 4 // CS pin for SPI1
#define PAGE_SIZE 256 // Page size for W25Q64 256 bytes - 128 words of 16 bits

//Sound
#define SAMPLE_RATE          32000U            // 32 kHz
#define SOUND_DURATION_S     2U                // 2 seconds each
#define SAMPLES_PER_SOUND    (SAMPLE_RATE * SOUND_DURATION_S)
#define BYTES_PER_SOUND      (SAMPLES_PER_SOUND * sizeof(uint16_t)) // 2 bytes per sample
#define PAGES_PER_SOUND      (BYTES_PER_SOUND / (PAGE_SIZE)) // 500 pages of 256 bytes
#define TOTAL_PAGES          64 * PAGES_PER_SOUND // Total number of pages in the flash memory

//SPI
#define SPI1_CS_CONFIG SPI_GPIO_PORT, SPI_CS_PIN // CS pin for SPI1

//DMA
#define SOUND_BUFFER_SIZE 256 // Define audio buffer size
#define HALF_SOUND_BUFFER_SIZE (SOUND_BUFFER_SIZE / 2) // Size of the sound buffer for DMA transfer (128 words of 16 bits)

// PWM
#define BASE_PWM_VALUE 1125U // Base value for PWM (50% duty cycle)
#define MAX_PWM_VALUE 2249U // Maximum value for PWM (72MHz / 32000 - 1 = 2249)
typedef enum sound_state_t
{
    SOUND_MUTE=0,
    SOUND_RESISTANCE=1,
    SOUND_E1=2,
    SOUND_E2=3,
    SOUND_E3=4,
    SOUND_E4=5,
    SOUND_ANOMALY=-2
} SoundState;

static uint8_t rvalues[12] = {10, 12, 15, 18, 22, 27, 33, 39, 47, 56, 68, 82};
static uint8_t expvalues[5] = {1, 2, 3, 4, 5};
typedef struct sound_config_t
{
    SoundState state;
    uint8_t rValue;
    uint8_t expValue;
} SoundConfig;

extern uint16_t sound_buffer[PAGE_SIZE]; // Buffer for sound data = 256 words of 16 bits -> 2 pages of 128 words
extern TaskHandle_t xSoundTaskHandle; // Declare xSoundTaskHandle

void setSoundState(SoundConfig cfg);

void vSoundTask( void *pvParameters );
void vInit_soundTasks(void);

void initSound_SPI1(void);
void initSound_DMA(void);
void initSound_PWM(void);

#endif // SOUND_H
