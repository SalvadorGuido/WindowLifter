/*
 * output_functions.c
 *
 *  Created on: Nov 5, 2018
 *      Author: maver
 */
#include "S32K144.h"

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
