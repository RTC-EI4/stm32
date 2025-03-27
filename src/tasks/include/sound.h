#ifndef SOUND_H
#define SOUND_H

typedef enum sound_state_t
{
    SOUND_MUTE,
    SOUND_TEST,
    SOUND_SUPPLY,
    SOUND_ANOMALY,
    SOUND_RESISTANCE
} SoundState;

void vInit_soundTasks( void );

void initSound_SPI1(void);
void initSound_DMA2(void);


#endif // SOUND_H
