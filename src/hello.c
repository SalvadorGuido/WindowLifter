#include "clocks_and_modes.h"
#include "initialization_functions.h"
#include "output_functions.h"
#include "macros.c"
#include "S32K144.h"          /* include peripheral declarations S32K144 */

volatile unsigned int idle_counter = 0; /* main loop idle counter */
volatile unsigned int ui32_lpit0_ch0_flag_counter = 0;
volatile unsigned int ui32_lpit0_ch1_flag_counter = 0;
volatile unsigned int ui32_lpit0_ch2_flag_counter = 0;
volatile unsigned int ui8_Register;
volatile unsigned char uc8_Anti_Pinch=0;
volatile unsigned char uc8_up=0;
volatile unsigned char uc8_down=0;
volatile unsigned int  ui32_value=0;
volatile int exit_code = 0;


int main(void) {


	MICROCONTROLLER_init();

  for (;;) {
	  if((uc8_down==0) && (uc8_up==0)){
		  PTD->PSOR|=1<<0;
		  PTD->PSOR|=1<<16;
	  }
      else if(uc8_up && (ui32_value<1023)){
		  PTD->PCOR|=1<<0;
		  PTD->PSOR|=1<<16;
      }
      else if(uc8_down && (ui32_value>0)){
		  PTD->PSOR|=1<<0;
		  PTD->PCOR|=1<<16;
      }
      else{};


  }
}

void PORTC_IRQHandler(void)  /* Interrupts for pressed buttons  */
{
	ui8_Register=PORTC->ISFR;
	if (ui8_Register & 1<<15)                       /*If interrupt has been caused by sw1 (Antipinch) */
	  {

		if(uc8_up){                             /*If the interrupt is made while the window is moving up */
			PORTC->PCR[15] |= (1 << 24);       /*Clean Antipinch   Interrupt flag */
			PORTC->PCR[13] |= (1 << 24);       /*Clean Moving Down Interrupt flag */
			PORTC->PCR[12] |= (1 << 24);       /*Clean Moving Up   Interrupt flag */
			ui32_lpit0_ch0_flag_counter=0;          /*Initialize ch0 flag counter */
			LPIT0_chan0_init();                /*Initialize ch0 */
		}
		else {                                 /*Antipinch pressed while other condition*/
			PORTC->PCR[15] |= (1 << 24);       /*Clean Antipinch   Interrupt flag */
		}

	  }
	else if((ui8_Register & 1<<12)&&(uc8_Anti_Pinch==0) )                    /*If interrupt has been caused by sw 2 (Button Up) and the antipinch function is not being executed */
	  {
		PORTC->PCR[12] |= (1 << 24);                                /*Clean the interrupt flag*/
		if ((PTC->PDIR & (1<<12)&& ((PTC->PDIR&(1<<13))==0)))       /*If switch2 is pressed and switch 3 is not pressed*/
		{
			ui32_lpit0_ch0_flag_counter=0;                               /*Initialize ch0 flag counter */
			LPIT0_chan0_init();                                     /* Initialize ch0  */
		}
	   else if((PTC->PDIR & (1<<12))==0){                           /*If there is a falling edge in sw2*/
		   if((ui32_lpit0_ch0_flag_counter>=50)||(ui32_lpit0_ch0_flag_counter==0)){                          /*Check if the pulse width is greater or equal than 500ms*/
			   turn_off ();                                         /*If the pulse is greater or equal than 500ms turn off the moving up when the switch is released*/
		   }
	   }
	  }
	else if (ui8_Register & 1<<13&&(uc8_Anti_Pinch==0))                      /*If interrupt has been caused by sw3 (Button Down) and the antipinch function is not on execution */
	   {

		PORTC->PCR[13] |= (1 << 24);                                /*Clean the interrupt flag*/
		if (PTC->PDIR & (1<<13)&& ((PTC->PDIR&(1<<12))==0)){        /*If switch2 is pressed and switch 3 is not pressed*/
			ui32_lpit0_ch0_flag_counter=0;                               /* Start ch0 timeout counter  */
			LPIT0_chan0_init();                                     /* Initialize ch0  */
		}
		else if((PTC->PDIR & (1<<13))==0){                          /*If there is a falling edge in sw3*/
		    if((ui32_lpit0_ch0_flag_counter>=50)||(ui32_lpit0_ch0_flag_counter==0)){                         /*Check if the pulse width is greater or equal than 500ms*/
				turn_off ();                                        /*If the pulse is greater or equal than 500ms turn off the moving down when the switch is released*/
			}
		}
	   }
	else {                                                          /*Clean all the interrupt flags*/
	   PORTC->PCR[15] |= (1 << 24);
	   PORTC->PCR[13] |= (1 << 24);
	   PORTC->PCR[12] |= (1 << 24);
	   }
}
void LPIT0_Ch0_IRQHandler (void) {                                  /* This function measures if the pulse width is greater than 10ms and if it is true, outputs the corresponding signals */
	LPIT0->MSR |= LPIT_MSR_TIF0_MASK;                               /* Clear LPIT0 timer flag 0 */
	ui32_lpit0_ch0_flag_counter++;
  if (ui32_lpit0_ch0_flag_counter==1){                                   /* if 10ms had occur since one of the 3 switches were pressed */
	  if(PTC->PDIR & (1<<15)){                                      /* antipinch button is pressed and pulse width is already 10ms */
		  uc8_Anti_Pinch=1;                                             /* antipinch flag is set */
		  go_down();                                                /* The movement downward is started*/
	  }
	  else if ((PTC->PDIR & (1<<12))&& ((PTC->PDIR&(1<<13))==0)){   /*If SW2 is pressed (moving up) and SW3 (moving down) is not pressed */
	  if ((uc8_up==0) &&(uc8_down==0)){                               /*If the lifter is not on movement*/
		  go_up();                                                  /*If the lifter will start moving up*/
	  }
	  else if(uc8_down==1){                                          /*If the lifter is on downward movement*/
		  turn_off();                                               /*The lifter will cancel all the movements*/
	  }
  }
	else if ((PTC->PDIR & (1<<13))&& ((PTC->PDIR&(1<<12))==0)){     /*If SW3 is pressed (moving down)and SW2 (moving up) is not pressed */
	  if ((uc8_down==0)&&(uc8_up==0)){                                /*If the lifter is not on movement*/
			  go_down();                                            /*The lift will star moving downward*/
		  }
		  else if(uc8_up==1){                                        /*If the lifter is on downward movement*/
			  turn_off();                                           /*The lifter will cancel all the movements*/
		  }
	  }
  }
	                                       /* Increment LPIT0 timeout counter */
}
void LPIT0_Ch1_IRQHandler (void) {
  LPIT0->MSR |= LPIT_MSR_TIF1_MASK;                                 /* Clear LPIT0 timer flag 1 */

  if (uc8_up && (ui32_value<1023)){                                   /*Move the Window Up*/
  ui32_value <<= 1 ;
  ui32_value++;
  }
  else if (uc8_down && (ui32_value>0)){                               /*Move the Window Down*/
  ui32_value >>= 1 ;
  }
  else if ((uc8_up==0) && (uc8_down==0)){                             /*Finish the movement*/
  turn_off();
  }
  ui32_lpit0_ch1_flag_counter++;         /* Increment LPIT0 timeout counter */
  led_output(ui32_value);
  if ((ui32_value==0) || (ui32_value==1023)){
  turn_off();
  }
  if((ui32_value==0)&&(uc8_Anti_Pinch==1)){                              /*Finish the movement*/
	  turn_off();
	  LPIT0_chan2_init();

  }

}
void LPIT0_Ch2_IRQHandler (void) {
  LPIT0->MSR |= LPIT_MSR_TIF2_MASK; /* Clear LPIT0 timer flag 2 */
  ui32_lpit0_ch2_flag_counter++;         /* Increase ch2 timeout counter  */
  uc8_Anti_Pinch=0;                     /* Disable antipinch flag*/
  LPIT0_chan_end (2);
  LPIT0_chan_end (1);
  LPIT0_chan_end (0);
}

