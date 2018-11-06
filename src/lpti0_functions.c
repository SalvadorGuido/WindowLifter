/*
 * lpti0_functions.c

 *
 *  Created on: Nov 5, 2018
 *      Author: maver
 */

#include "lpti0_functions.h"
#include "S32K144.h"
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
