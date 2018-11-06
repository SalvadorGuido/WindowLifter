/*
 * initialization_functions.c
 *
 *  Created on: Nov 5, 2018
 *      Author: maver
 */
#include "clocks_and_modes.h"
#include "initialization_functions.h"
#include "output_functions.h"
#include "macros.h"
#include "S32K144.h"

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
    PTD->PDDR |= PORTD_OUTPUT_PIN;
    PTB->PDDR |= PORTB_OUTPUT_PIN;
    PTE->PDDR |= PORTE_OUTPUT_PIN;
    PTC->PDDR |= PORTC_OUTPUT_PIN;

    /*Set Function of every pin, filters and special characteristics*/

    PORTD->PCR[0] = MUX_GPIO; /* Port D0: MUX = GPIO    BLUE LED  */
    PORTD->PCR[1] = MUX_GPIO; /* Port D1: MUX = GPIO              */
    PORTD->PCR[2] = MUX_GPIO; /* Port D2: MUX = GPIO              */
    PORTD->PCR[3] = MUX_GPIO; /* Port D3: MUX = GPIO              */
    PORTD->PCR[15] = MUX_GPIO; /* Port D15: MUX = GPIO  RED LED   */
    PORTD->PCR[16] = MUX_GPIO; /* Port D16: MUX = GPIO  GREEN LED */

    PORTB->PCR[14] = MUX_GPIO; /* Port B14: MUX = GPIO */
    PORTB->PCR[15] = MUX_GPIO; /* Port B15: MUX = GPIO */
    PORTB->PCR[16] = MUX_GPIO; /* Port B16: MUX = GPIO */
    PORTB->PCR[17] = MUX_GPIO; /* Port B17: MUX = GPIO */

    PORTE->PCR[13] = MUX_GPIO; /* Port D15: MUX = GPIO */
    PORTE->PCR[14] = MUX_GPIO; /* Port D16: MUX = GPIO */
    PORTE->PCR[15] = MUX_GPIO; /* Port E15: MUX = GPIO */
    PORTE->PCR[16] = MUX_GPIO; /* Port E16: MUX = GPIO */

    PORTC->PCR[16] = MUX_GPIO; /* Port C16: MUX = GPIO */
    PORTC->PCR[17] = MUX_GPIO; /* Port C17: MUX = GPIO */
    PORTC->PCR[13] = MUX_GPIO_UD_INT; /* Port C13 MUX = ALT1, GPIO  (SW3 on EVB)  both edges interrupt*/
    PORTC->PCR[12] = MUX_GPIO_UD_INT; /* Port C12: MUX = ALT1, GPIO (SW2 on EVB) both edges interrupt*/
    PORTC->PCR[15] = MUX_GPIO_U_ANTI; /* Port C15 MUX = ALT1, GPIO, Antipinch,(SW1 On Protoboard), Rising Edge interrupt, pull down enabled */

    PTC->PDDR &= ~(1<<12);             /*Input Switch 2*/
    PTC->PDDR &= ~(1<<13);             /*Input Switch 3*/
    PTC->PDDR &= ~(1<<15);             /*Input Switch Antipinch*/
    PTD->PDOR |= 1<<16 | 1<<15 | 1<<0; /*Due to pull up in the board LEDs, outputs are set in order to turn off the leds.*/

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

void FILTER_init (void){
 PCC-> PCCn[PCC_PORTC_INDEX] = PCC_PCCn_CGC_MASK; /* Enable clock for PORT C */
 PORTC->DFCR|= (1<<0);  /*0 in the lsb means bus clock for digital filter and 1 in the lsb means lpo clock for digital filter */
 PORTC->DFWR|=  0x1F;   /*This is the Filter length, which is a 5 bit number */
 PORTC->DFER|= ((1<<15)|(1<<12)|(1<<13));
}

void MICROCONTROLLER_init(void){
	  WDOG_disable();        /* Disable WDOG*/
	  SOSC_init_8MHz();      /* Initialize system oscillator for 8 MHz xtal */
	  SPLL_init_160MHz();    /* Initialize SPLL to 160 MHz with 8 MHz SOSC */
	  NormalRUNmode_80MHz(); /* Init clocks: 80 MHz SPLL & core, 40 MHz bus, 20 MHz flash */
	  FILTER_init();
	  PORT_init();
	  NVIC_init_IRQs ();     /* Enable desired interrupts and priorities */
}

