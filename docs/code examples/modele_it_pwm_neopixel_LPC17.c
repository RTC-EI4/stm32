void PWM1_IRQHandler(void) //gestion sans DMA et sans pr�calcul
{	LPC_PWM1->IR=1; //acquittement de l IT
	
  if(masque<0) // indicateur reset si masque <0
	{LPC_PWM1->MR1 =0;	//duree Ton =0
	 LPC_PWM1->LER = 0x1F;/ charger le nouveau rapport cyclique d�s que possible	
	 if(!(++masque))//test sur nul, on quitte le reset du WS si nul et on prepare le premier pixel
       {   masque|=1<<23; // positionner le masque sur 1<<23 (|= d un seul bit plus rapide que =nombre)
           pixel	=liste_pixel_aff[0]; //variable globale pour les 24 bits couleur � ecrire		 
	   }
	}
	else // on va creer les motifs pour envoyer les bits d un pixel, 24 motifs par pixel 
	{	LPC_PWM1->MR1 = (pixel&masque)? 75 : 40;//choix du rapport cyclique
		LPC_PWM1->LER = 0x1F;// charger le nouveau rapport cyclique d�s que possible	
	    masque>>=1;//preparation selection bit suivant
        if(masque == 0) //dernier bit a �t� envoy�, passer au pixel suivant
		  {	masque|=1<<23; //remettre le masque � jour
		    if (++index_pixel<dernierpixel) 
				  {  pixel	=liste_pixel_aff[index_pixel]; } // charger le pixel suivant
            else  {masque=-40;index_pixel=0;}  //mettre masque n�gatif pour g�nerer le reset   pendant 40 cycles pwm   
		  }			 		 	
	}

}