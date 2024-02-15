/*
 * eUSCIA0.c
 *
 *  Created on: 26 jun 2023
 *      Author: maxim
 */

#include <eUSCIA0_UART.h>

void eUSCIA0__UART_Init(){
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;
    UCA0BRW |= 6;
    UCA0MCTLW |= 0x20<<8 | UCOS16 | 8<<4;

    P2SEL0 &= ~(BIT0 | BIT1);                   //P2SEL0.x = 0
    P2SEL1 |= BIT0 | BIT1;                      //P2SEL1.x = 1; Selecciona la funci�n de UART en P2.1 y P2.0
    PM5CTL0 &= ~LOCKLPM5;

    UCA0CTLW0 &= ~UCSWRST;

    UCA0IE |= UCRXIE;                           //Habilita interrupci�n de recepci�n
                      //Habilita la las interrupciones enmascarables
    UCA0IFG &= ~UCRXIFG;
    _enable_interrupt();
}





