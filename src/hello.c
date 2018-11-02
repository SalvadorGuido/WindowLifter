#include "S32K144.h"          /* include peripheral declarations S32K144 */
#include "clocks_and_modes.h"
volatile unsigned int idle_counter = 0; /* main loop idle counter */
volatile unsigned int lpit0_ch0_flag_counter = 0;
volatile unsigned int lpit0_ch1_flag_counter = 0;
volatile unsigned int lpit0_ch2_flag_counter = 0;
volatile unsigned int Register;
volatile unsigned int anti_pinch=0;
volatile unsigned char c8_up=0;
volatile unsigned char c8_down=0;
volatile unsigned int  i32_value=0;
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
    PORTC->PCR[13] = 0x000B0110; /* Port C13 MUX = ALT1, GPIO  (SW3 LED on EVB) */
    PORTC->PCR[12] = 0x000B0110; /* Port C12: MUX = ALT1, GPIO (SW2 on EVB    ) */
    PORTC->PCR[15] = 0x00090112; /* Port C15 MUX = ALT1, GPIO, Antipinch  (SW1 On Protoboard) */

    PTC->PDDR &= ~(1<<12);             /*Input Switch 2*/
    PTC->PDDR &= ~(1<<13);             /*Input Switch 3*/
    PTC->PDDR &= ~(1<<15);             /*Input Switch Antipinch*/
    PTD->PDOR |= 1<<16 | 1<<15 | 1<<0; /*Due to pull up in the board LEDs, outputs are set in order to turn off the leds.*/

}
void led_output(int i32_value){
/*This section divides in 3 blocks the 10 bit output into 3 blocks*/
unsigned int i8_maskC=0x300; /*Filter 9 to 8 bits*/
unsigned int i8_maskE=0x0F0; /*Filter 7 to 4 bits*/
unsigned int i8_maskB=0x00F; /*Filter 3 to 0 bits*/

/*Apply the filters*/
i8_maskC &= i32_value;
i8_maskE &= i32_value;
i8_maskB &= i32_value;

/*Bitshifting for keeping just the important bits*/
i8_maskC = i8_maskC>>8;
i8_maskE = i8_maskE>>4;

/*Set the output*/
PTC->PDOR =  (i8_maskC)<<16;
PTE->PDOR =  (i8_maskE)<<13;
PTB->PDOR =  (i8_maskB)<<14;
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

  LPIT0->TMR[0].TVAL =  400000;                ;    /* Chan 0 Timeout period: 0.040M clocks */
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
void LPIT0_chan_end (char timer) {
  LPIT0->CLRTEN |= 1<<timer ;                         /*Finishes the chanel of the timer */
  LPIT0->MCR = 0x00000002;                            /*Reset the complete timer */
  LPIT0->MCR = 0x00000001;                            /*Timer ready to start*/
}
void turn_off (void) {
  c8_up=0;
  c8_down=0;
  LPIT0_chan_end (1);
  LPIT0_chan_end (0);
}
void go_up (void){
  c8_up=1;
  c8_down=0;
  LPIT0_chan1_init ();
}
void go_down (void){
  c8_up=0;
  c8_down=1;
  LPIT0_chan1_init ();
}
