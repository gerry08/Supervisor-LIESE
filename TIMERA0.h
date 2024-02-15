/*
 * TIMERA0.h
 *
 *  Created on: 24 may 2023
 *      Author: maxim
 */



#ifndef TIMERA0_H_
#define TIMERA0_H_
#include <msp430.h>
#include <stdint.h>


void timer_Init(void);
void timer_setPeriod(uint16_t sec);
void timer_Stop(void);
void timer_Enable(void);
void timer_Wait_1_ms();
void timer_Wait_N_ms(int Num);

#endif /* TIMERA0_H_ */
