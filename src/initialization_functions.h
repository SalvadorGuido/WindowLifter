/*
 * initialization_functions.h
 *
 *  Created on: Nov 5, 2018
 *      Author: maver
 */

#ifndef INITIALIZATION_FUNCTIONS_H_
#define INITIALIZATION_FUNCTIONS_H_


void PORT_init (void);
void NVIC_init_IRQs (void);
void WDOG_disable (void);
void FILTER_init (void);

#endif /* INITIALIZATION_FUNCTIONS_H_ */
