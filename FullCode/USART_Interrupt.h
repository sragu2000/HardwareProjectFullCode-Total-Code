
/*
 * USART_Interrupt.h
 *
 * Created: 5/10/2022 11:18:12 AM
 *  Author: ishu
 */ 


#ifndef USART_INTERRUPT_H_
#define USART_INTERRUPT_H_
#define F_CPU 8000000UL						/* Define CPU clock Frequency e.g. here its 8MHz */
#include <avr/io.h>							/* Include AVR std. library file */
#include <avr/interrupt.h>
#include "USART_Interrupt.h"

#define BAUD_PRESCALE (((F_CPU / (9600 * 16UL))) - 1)	/* Define prescale value */

void USART_Init(unsigned long);				/* USART initialize function */
char USART_RxChar();						/* Data receiving function */
void USART_TxChar(char);					/* Data transmitting function */
void USART_SendString(char*);				/* Send string of USART data function */




#endif /* USART_INTERRUPT_H_ */