#include "stm32f10x.h"

// communication par FIFO
#define TAILLE_FIFO 32
uint8_t fifo_env[TAILLE_FIFO] ;
uint8_t pw_fifo =0;
uint8_t pr_fifo =0;
uint16_t place_libre = TAILLE_FIFO; 

unsigned char identifiant[8];
volatile unsigned char motif_onewire_fini = 0;
volatile unsigned char etat_onewire = 0;
#define BIT_INDIQUANT_ENVOI_POSSIBLE  (1<< 7)
#define  TIM1CLK_1US  1	
	
void tentative_depile_fifo(void)
{
	if ((USART2->SR  & (BIT_INDIQUANT_ENVOI_POSSIBLE)))
	{ place_libre++;
		USART2->DR =fifo_env[pr_fifo++]; 
		if( pr_fifo== TAILLE_FIFO) {pr_fifo=0;}	
	}
}

 
// Fonction d'initialisation GPIOA
void init_gpioA(unsigned char num_bit, unsigned int quartet_config) {
    // Calculez la position du quartet de bits à configurer dans CRL ou CRH
    unsigned char bit_ref = (num_bit * 4) & 31;
    // Activer l'horloge pour GPIOA : Bit 2 dans RCC->APB2ENR doit être mis à 1
    RCC->APB2ENR |= (1 << 2);
    // Limiter quartet_config à 4 bits
    quartet_config &= 0xF;
    // Configurer le registre CRL si le numéro de bit est inférieur à 8
    if (num_bit < 8) {
        // Effacer les anciens bits du quartet correspondant dans CRL
        GPIOA->CRL &= ~(0xF << bit_ref);
        // Configurer les nouveaux bits du quartet dans CRL
        GPIOA->CRL |= (quartet_config << bit_ref);
    } else {
        // Effacer les anciens bits du quartet correspondant dans CRH
        GPIOA->CRH &= ~(0xF << bit_ref);
        // Configurer les nouveaux bits du quartet dans CRH
        GPIOA->CRH |= (quartet_config << bit_ref);
    }
}
// Fonction d'initialisation GPIOA
void init_gpioB(unsigned char num_bit, unsigned int quartet_config) {
    // Calculez la position du quartet de bits à configurer dans CRL ou CRH
    unsigned char bit_ref = (num_bit * 4) & 31;
    // Activer l'horloge pour GPIOA : Bit 2 dans RCC->APB2ENR doit être mis à 1
    RCC->APB2ENR |= (1 << 3);
    // Limiter quartet_config à 4 bits
    quartet_config &= 0xF;
    // Configurer le registre CRL si le numéro de bit est inférieur à 8
    if (num_bit < 8) {
        // Effacer les anciens bits du quartet correspondant dans CRL
        GPIOB->CRL &= ~(0xF << bit_ref);
        // Configurer les nouveaux bits du quartet dans CRL
        GPIOB->CRL |= (quartet_config << bit_ref);
    } else {
        // Effacer les anciens bits du quartet correspondant dans CRH
        GPIOB->CRH &= ~(0xF << bit_ref);
        // Configurer les nouveaux bits du quartet dans CRH
        GPIOB->CRH |= (quartet_config << bit_ref);
    }
}
// Fonction d'initialisation GPIOC
void init_gpioC(unsigned char num_bit, unsigned int quartet_config) {
    // Calculez la position du quartet de bits à configurer dans CRL ou CRH
    unsigned char bit_ref = (num_bit * 4) & 31;
    // Activer l'horloge pour GPIOA : Bit 2 dans RCC->APB2ENR doit être mis à 1
    RCC->APB2ENR |= (1 << 4);
    // Limiter quartet_config à 4 bits
    quartet_config &= 0xF;
    // Configurer le registre CRL si le numéro de bit est inférieur à 8
    if (num_bit < 8) {
        // Effacer les anciens bits du quartet correspondant dans CRL
        GPIOC->CRL &= ~(0xF << bit_ref);
        // Configurer les nouveaux bits du quartet dans CRL
        GPIOC->CRL |= (quartet_config << bit_ref);
    } else {
        // Effacer les anciens bits du quartet correspondant dans CRH
        GPIOC->CRH &= ~(0xF << bit_ref);
        // Configurer les nouveaux bits du quartet dans CRH
        GPIOC->CRH |= (quartet_config << bit_ref);
    }
}
void init_bouton(void)
{// initialiser la patte  portC.13 en entrée sans pull up ou pull down
     init_gpioC(13,0x04 ); 
}

//*************************************************************************************
//**************************************************************************************


// Fonction d'initialisation de l'USART2 (complétez les étapes de configuration)
void init_USART2(void) {
    // Activer l'horloge pour l'USART2
    RCC->APB1ENR |= (1 << 17);

    // Configurer PA2 (TX) et PA3 (RX) pour la communication série
    init_gpioA(2, 9);  // PA2 en mode alternate function output push-pull (USART2_TX)
    init_gpioA(3, 8);  // PA3 en mode input floating (USART2_RX)
     
    // Configurer le baudrate à 9600 bps (APB1 Clock = 36 MHz)
    USART2->BRR = 36000000 / 9600;

    // Configurer l'USART pour 8 bits de données, 1 bit de stop
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;  // Activer TX, RX et l'USART
    USART2->CR2 = 0x0000;  // 1 bit de stop
    USART2->CR3 = 0x0000;  // Pas de configuration additionnelle
}

unsigned char quartet2hex(unsigned char in)
{ in&=0x0F;
	return (unsigned char) (in>9)?('A'+in-10):('0'+in);
}
void fabrique_trame(void)
{     fifo_env[pw_fifo++] = '0'; 
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
	    fifo_env[pw_fifo++] = 'x'; 
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}	
			
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[0]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[0]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}

			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[1]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[1]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}

			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[2]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[2]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
	
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[3]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[3]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
	
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[4]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[4]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
				
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[5]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[5]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}

			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[6]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[6]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
	
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[7]>>4);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			fifo_env[pw_fifo++] =(unsigned char) quartet2hex(identifiant[7]);
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}

			fifo_env[pw_fifo++] = 0x0A; 
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
	    fifo_env[pw_fifo++] = 0x0D; 
			if( pw_fifo== TAILLE_FIFO) {pw_fifo=0;}
			
      place_libre-=20;
}

void TIM1_CC_IRQHandler(void) {
    // Interruption CH2 (arrêt du timer) IT fin motif
    if (TIM1->SR & TIM_SR_CC2IF) {
        TIM1->SR &= ~TIM_SR_CC2IF;  // Effacer le flag CH2
        TIM1->CR1 &= ~TIM_CR1_CEN; // Arrêter le timer
        // Action à effectuer après CH2 (arrêt du timer)
			 GPIOA->BSRR = 0x01<< (9+16) ;//effacement PA9 
			motif_onewire_fini = 1;
    }

    // Interruption CH3 (indépendante)
    if (TIM1->SR & TIM_SR_CC3IF) {
        TIM1->SR &= ~TIM_SR_CC3IF;  // Effacer le flag CH3
        // Action à effectuer après CH3
			 GPIOA->BSRR = 0x04<< (9+16) ;//effacement  PA11
			 if(GPIOA->IDR & (1<<8)) {etat_onewire=1;} else {etat_onewire=0;}
			
    }
}
void Timer1_Start(uint16_t pulse_duration, uint16_t ch2_duration, uint16_t ch3_duration) {
    TIM1->CR1 &= ~TIM_CR1_CEN;
	  // Configurer la durée du pulse sur CH1
    TIM1->CCR1 = pulse_duration; // En ticks (10 kHz)

    // Configurer la durée pour arrêter le timer sur CH2
    TIM1->CCR2 = ch2_duration; // En ticks (10 kHz)
    TIM1->CCR3 = ch3_duration; // En ticks (10 kHz)
    // Configurer la durée pour l'IT sur CH3
    if (ch3_duration < ch2_duration) {
        TIM1->DIER |= TIM_DIER_CC3IE; // Désactiver IT CH3 si ch3_duration > ch2_duration
    } else {
        TIM1->DIER &= ~TIM_DIER_CC3IE; // Désactiver IT CH3 si ch3_duration > ch2_duration
    }

    // Réinitialiser le compteur
    TIM1->CNT = 0;
	  GPIOA->BSRR = 0x05<< (9) ;//montee PA9 et PA11
    // Lancer le timer
    TIM1->CR1 |= TIM_CR1_CEN;
}



void Timer1_Init(void) {
    // Activer l'horloge pour Timer 1 et GPIOA
    RCC->APB2ENR |= RCC_APB2ENR_TIM1EN ;
    
    // Configurer PA8 en collecteur ouvert
	   init_gpioA(8, 0x0D); //fonction alternate  pour CH1 collecteur ouvert mettre pull up externe
     init_gpioA(9, 3); //sortie espion
     init_gpioA(10, 9); //quartet 9 pour config CRH en fonction alternate pour CH3  push pull
     init_gpioA(11, 3); //sortie espion
	   GPIOA->BSRR = 0x05<< (9+16) ;//effacement PA9 et PA11
    // Configurer la fréquence du timer
	  TIM1->CR1 &= ~TIM_CR1_CEN;
    TIM1->PSC = 71; // Divise l'horloge système à 1MHz (72 MHz / 72)
    TIM1->ARR = 2000;//0xFFFF;
	  TIM1->CNT =0x0000;
	  TIM1->EGR = TIM_EGR_UG;// permet de générer l evenement de reload
    // Configurer CH1 (Output Compare pour le pulse niveau bas)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC1M; // Clear mode bits pour CH1
    TIM1->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; // Mode PWM1
    TIM1->CCER  |= TIM_CCER_CC1E;   // Activer la sortie CH1
    TIM1->CCER  |= TIM_CCER_CC1P;   // Polarité active bas

    // Configurer CH2 (One Pulse, arrête le timer à un certain temps)
    TIM1->CCMR1 &= ~TIM_CCMR1_OC2M; // Clear mode bits pour CH2
    TIM1->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2; // Mode PWM1
    TIM1->CCER |= TIM_CCER_CC2E;   // Activer la sortie CH2

    // Configurer CH3 (IT indépendante pour une durée définie)
    TIM1->CCMR2 &= ~TIM_CCMR2_OC3M; // Clear mode bits pour CH3
    TIM1->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // Mode PWM1
    TIM1->CCER |= TIM_CCER_CC3E;   // Activer la sortie CH3

    // Activer One Pulse Mode (OPM) pour CH2
    TIM1->CR1 |= TIM_CR1_OPM;
		

    // Activer le Main Output Enable pour permettre la sortie sur les pins
    TIM1->BDTR |= TIM_BDTR_MOE;
		
   // Activer les interruptions sur CH2 et CH3
    TIM1->DIER |= TIM_DIER_CC2IE | TIM_DIER_CC3IE;

    // Activer les interruptions Timer 1
    NVIC_EnableIRQ(TIM1_CC_IRQn);
}

void ONEWIRE_RESET_BLOQUANT(void) 
	 { motif_onewire_fini=0;
		 Timer1_Start( 500*TIM1CLK_1US,1000*TIM1CLK_1US,565*TIM1CLK_1US); 
		 while (!motif_onewire_fini ) 
		 {GPIOA->BSRR =(unsigned int) 1<< 15;
		  GPIOA->BSRR =(unsigned int) 1<< 31;	
		 }
		 
	 }
	 
void ONEWIRE_WRITE_BIT_BLOQUANT(unsigned char x) 
   { motif_onewire_fini=0;
     if(x){Timer1_Start( 8 *TIM1CLK_1US,64*TIM1CLK_1US,100*TIM1CLK_1US);}
     else {Timer1_Start( 60*TIM1CLK_1US,64*TIM1CLK_1US,100*TIM1CLK_1US);}
		 while (!motif_onewire_fini ) 
		 {GPIOA->BSRR =(unsigned int) 1<< 15;
		  GPIOA->BSRR =(unsigned int) 1<< 31;	
		 }	 
	 }	 
unsigned char ONEWIRE_READ_BIT_BLOQUANT(void)
	  { motif_onewire_fini=0;
			Timer1_Start( 8*TIM1CLK_1US,64*TIM1CLK_1US,13*TIM1CLK_1US);
		  while (!motif_onewire_fini ) 
		  {GPIOA->BSRR =(unsigned int) 1<< 15;
		   GPIOA->BSRR =(unsigned int) 1<< 31;	
		  }
		return etat_onewire;
	  }
/*	
void gestion_lecture_TAG(void)
{static unsigned char mae_lect = 0;
	switch (mae_lect)
		{ case 0: //déclenchement reset
			ONEWIRE_RESET(); mae_lect=1;
			break;
		case 1:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(0);mae_lect++; } break;
		case 2:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(1);mae_lect++; } break;
		case 3:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(0);mae_lect++; } break;
		case 4:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(1);mae_lect++; } break;
		case 5:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(0);mae_lect++; } break;
		case 6:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(1);mae_lect++; } break;
		case 7:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(0);mae_lect++; } break;
		case 8:if(motif_onewire_fini == 1) {motif_onewire_fini =0;ONEWIRE_WRITE(1);mae_lect++; } break;
	}
	
}	
*/
void ONEWIRE_ENVOI_OCTET_BLOQUANT(unsigned char in)	
{ ONEWIRE_WRITE_BIT_BLOQUANT(in&0x01);
  ONEWIRE_WRITE_BIT_BLOQUANT(in&0x02);
  ONEWIRE_WRITE_BIT_BLOQUANT(in&0x04);
  ONEWIRE_WRITE_BIT_BLOQUANT(in&0x08);
	ONEWIRE_WRITE_BIT_BLOQUANT(in&0x10);
	ONEWIRE_WRITE_BIT_BLOQUANT(in&0x20);
	ONEWIRE_WRITE_BIT_BLOQUANT(in&0x40);
	ONEWIRE_WRITE_BIT_BLOQUANT(in&0x80);
}	
unsigned char ONEWIRE_READ_OCTET_BLOQUANT(void)
{unsigned char rep=0;
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x01;}	
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x02;}
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x08;}
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x04;}
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x10;}
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x20;}
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x40;}
	if(ONEWIRE_READ_BIT_BLOQUANT()) {rep|=0x80;}

	return rep;
}

void gestion_lecture_TAG_bloquante(void)
{ ONEWIRE_RESET_BLOQUANT();
	ONEWIRE_ENVOI_OCTET_BLOQUANT(0x33);	
	identifiant[0]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[1]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[2]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[3]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[4]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[5]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[6]=ONEWIRE_READ_OCTET_BLOQUANT();
	identifiant[7]=ONEWIRE_READ_OCTET_BLOQUANT();

}

int main(void)
{volatile uint32_t tempo;
init_bouton();
Timer1_Init();
init_USART2();
init_gpioA(15,3);init_gpioA(10,3);
  while(1)
	{ gestion_lecture_TAG_bloquante();
	 //	ONEWIRE_RESET_BLOQUANT();
   //  ONEWIRE_READ_BIT_BLOQUANT();
   // ONEWIRE_WRITE_BIT_BLOQUANT(1) ;
   // ONEWIRE_WRITE_BIT_BLOQUANT(0) ;
		
	  if(place_libre>20) {	  fabrique_trame();	 }
    while(place_libre<TAILLE_FIFO)	{tentative_depile_fifo();	}	
		
		for(tempo=10000;tempo>0;tempo--) 
		{//temoin de vie du main
		 GPIOA->BSRR =(unsigned int) 1<< 10;
		 GPIOA->BSRR =(unsigned int) 1<< 26;			
		} 
	
  }
}
