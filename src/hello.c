#include "S32K144.h"          /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
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



void PORT_init (void) {

	 /*	Pin Description:(FROM MSB TO LSB)
	  * 10 LED BAR:  PORTC 17...16   PORTE 16..13 PORTB 17...14.
	  * LED INDICATORS ON EVB: PORTC16=GREEN LED, PORTC15=RED LED, PORTC0=BLUE LED.
	  * ANTIPINCH BUTTON = PORTC15; INTERNAL PULLDOWN RESISTOR.
	  * */
	PCC-> PCCn[PCC_PORTB_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock to PORT B */
	PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock to PORT C */
	PCC-> PCCn[PCC_PORTD_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock to PORT D */
	PCC-> PCCn[PCC_PORTE_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock to PORT E */

    /*Set Each Output pin as Output*/
    PTD->PDDR |= 0x0001800F;
    PTB->PDDR |= 0X0003C000;
    PTE->PDDR |= 0x0001E000;
    PTC->PDDR |= 0X00030000;

    /*Set Function of every pin, filters and special characteristics*/

    PORTD->PCR[0] = 0x00000100; /* Port D0: MUX = GPIO    BLUE LED  */
    PORTD->PCR[1] = 0x00000100; /* Port D1: MUX = GPIO              */
    PORTD->PCR[2] = 0x00000100; /* Port D2: MUX = GPIO              */
    PORTD->PCR[3] = 0x00000100; /* Port D3: MUX = GPIO              */
    PORTD->PCR[15] = 0x00000100; /* Port D15: MUX = GPIO  RED LED   */
    PORTD->PCR[16] = 0x00000100; /* Port D16: MUX = GPIO  GREEN LED */

    PORTB->PCR[14] = 0x00000100; /* Port B14: MUX = GPIO */
    PORTB->PCR[15] = 0x00000100; /* Port B15: MUX = GPIO */
    PORTB->PCR[16] = 0x00000100; /* Port B16: MUX = GPIO */
    PORTB->PCR[17] = 0x00000100; /* Port B17: MUX = GPIO */

    PORTE->PCR[13] = 0x00000100; /* Port D15: MUX = GPIO */
    PORTE->PCR[14] = 0x00000100; /* Port D16: MUX = GPIO */
    PORTE->PCR[15] = 0x00000100; /* Port E15: MUX = GPIO */
    PORTE->PCR[16] = 0x00000100; /* Port E16: MUX = GPIO */

    PORTC->PCR[16] = 0x00000100; /* Port C16: MUX = GPIO */
    PORTC->PCR[17] = 0x00000100; /* Port C17: MUX = GPIO */
    PORTC->PCR[13] = 0x000B0110; /* Port C13 MUX = ALT1, GPIO  (SW3 on EVB)  both edges interrupt*/
    PORTC->PCR[12] = 0x000B0110; /* Port C12: MUX = ALT1, GPIO (SW2 on EVB) both edges interrupt*/
    PORTC->PCR[15] = 0x00090112; /* Port C15 MUX = ALT1, GPIO, Antipinch,(SW1 On Protoboard), Rising Edge interrupt, pull down enabled */

    PTC->PDDR &= ~(1<<12);             /*Input Switch 2*/
    PTC->PDDR &= ~(1<<13);             /*Input Switch 3*/
    PTC->PDDR &= ~(1<<15);             /*Input Switch Antipinch*/
    PTD->PDOR |= 1<<16 | 1<<15 | 1<<0; /*Due to pull up in the board LEDs, outputs are set in order to turn off the leds.*/

}
void led_output(int ui32_value){
/*This section divides in 3 blocks the 10 bit output into 3 blocks*/
unsigned int i8_maskC=0x300; /*Filter 9 to 8 bits*/
unsigned int i8_maskE=0x0F0; /*Filter 7 to 4 bits*/
unsigned int i8_maskB=0x00F; /*Filter 3 to 0 bits*/

/*Apply the filters*/
i8_maskC &= ui32_value;
i8_maskE &= ui32_value;
i8_maskB &= ui32_value;

/*Bitshifting for keeping just the important bits*/
i8_maskC = i8_maskC>>8;
i8_maskE = i8_maskE>>4;

/*Set the output*/
PTC->PDOR =  (i8_maskC)<<16; /*From pin 16 to 17*/
PTE->PDOR =  (i8_maskE)<<13; /*From pin 13 to 16*/
PTB->PDOR =  (i8_maskB)<<14; /*From pin 14 to 17*/
}
void NVIC_init_IRQs (void) {

	/*This function start the Nested Interruption Vector
	 * Doing the next sequence:
	 */
  S32_NVIC->ICPR[1] = 1 << (48 % 32);  /* IRQ48-LPIT0 ch0: clr any pending IRQ*/
  S32_NVIC->ICPR[1] = 1 << (49 % 32);  /* IRQ49-LPIT0 ch1: clr any pending IRQ*/
  S32_NVIC->ICPR[1] = 1 << (50 % 32);  /* IRQ50-LPIT0 ch2: clr any pending IRQ*/
  S32_NVIC->ICPR[1] = 1 << (61 % 32);  /* IRQ61-PortC      clr any pending IRQ*/

  S32_NVIC->ISER[1] = 1 << (48 % 32);  /* IRQ48-LPIT0 ch0: enable IRQ */
  S32_NVIC->ISER[1] = 1 << (49 % 32);  /* IRQ49-LPIT0 ch1: enable IRQ */
  S32_NVIC->ISER[1] = 1 << (50 % 32);  /* IRQ50-LPIT0 ch2: enable IRQ */
  S32_NVIC->ISER[1] = 1 << (61 % 32);  /* IRQ61-PORTC ch0: enable IRQ */

  S32_NVIC->IP[48] = 0x0A;             /* IRQ48-LPIT0 ch1: priority 10 of 0-15*/
  S32_NVIC->IP[49] = 0x09;             /* IRQ49-LPIT0 ch2: priority  9 of 0-15*/
  S32_NVIC->IP[50] = 0x08;             /* IRQ50-LPIT0 ch3: priority  8 of 0-15*/
  S32_NVIC->IP[61] = 0x05;             /* IRQ61 PORTC      priority 5 of 0-15*/
}
void WDOG_disable (void) {
  WDOG->CNT=0xD928C520;     /* Unlock watchdog */
  WDOG->TOVAL=0x0000FFFF;   /* Maximum timeout value */
  WDOG->CS = 0x00002100;    /* Disable watchdog */
}
void LPIT0_chan0_init (void) {
  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);      /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK;   /* Enable clk to LPIT0 regs */
  LPIT0->MCR  |= 0x00000001;
  LPIT0->MIER |= 0x00000001;                        /* TIE 0 Enabled */

  LPIT0->TMR[0].TVAL =  400000;                	    /* Chan 0 Timeout period: 0.040M clocks */
  LPIT0->TMR[0].TCTRL = 0x0000001;                  /* T_EN=1: Timer channel is enabled */

}
void LPIT0_chan1_init (void) {
  LPIT0->MCR  |= 0x00000001;                        /* DBG_EN-0: Timer chans stop in Debug mode */
  LPIT0->MIER |= 0x00000002;                        /* TIE 1 Enabled */

  LPIT0->TMR[1].TVAL = 16000000;                   /* Chan 1 Timeout period: 0.0160M clocks (400ms) */
  LPIT0->TMR[1].TCTRL = 0x0000001;                 /* TRG_SEL=0: Timer chan 0 trigger source is selected*/
}
void LPIT0_chan2_init (void) {
	  PCC->PCCn[PCC_LPIT_INDEX] = PCC_PCCn_PCS(6);    /* Clock Src = 6 (SPLL2_DIV2_CLK)*/
	  PCC->PCCn[PCC_LPIT_INDEX] |= PCC_PCCn_CGC_MASK; /* Enable clk to LPIT0 regs */
  LPIT0->MCR = 0x00000001;                            /* DBG_EN-0: Timer chans stop in Debug mode */
  LPIT0->MIER |= 0x00000004;                          /* TIE 0,1,2 Enabled */

  LPIT0->TMR[2].TVAL = 200000000;                     /* Chan 2 Timeout period: 20M clocks (5s) */
  LPIT0->TMR[2].TCTRL = 0x0000001;                    /* T_EN=1: Timer channel is enabled */
}
void LPIT0_chan_end (unsigned char timer) {
  if ((timer<4)&&(timer>=0)){
  LPIT0->CLRTEN |= 1<<timer ;                         /*Finishes the chanel of the timer */
  LPIT0->MCR = 0x00000002;                            /*Reset the complete timer */
  LPIT0->MCR = 0x00000001;                            /*Timer ready to start*/
  }
}
void turn_off (void) {
  uc8_up=0;
  uc8_down=0;
  LPIT0_chan_end (1);
  LPIT0_chan_end (0);
}
void go_up (void){
  uc8_up=1;
  uc8_down=0;
  LPIT0_chan1_init ();
}
void go_down (void){
  uc8_up=0;
  uc8_down=1;
  LPIT0_chan1_init ();
}

int main(void) {
  WDOG_disable();        /* Disable WDOG*/
  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz SPLL & core, 40 MHz bus, 20 MHz flash */
  PORT_init();
  NVIC_init_IRQs ();        /* Enable desired interrupts and priorities */

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

