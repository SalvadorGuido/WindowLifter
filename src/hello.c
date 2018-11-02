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
