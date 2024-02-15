/*
 * LED.c
 *
 *  Created on: 24 may 2023
 *      Author: maxim
 */


#include <msp430.h>

void LED_Init(){
    P1DIR |= BIT0;
    //P4DIR |= BIT6;
    PM5CTL0 &= ~LOCKLPM5;
}

void LED_TurnOn(){
    P1OUT |= BIT0;
}

void LED_TurnOff(){
    P1OUT &= ~BIT0;
}

void LED_Toggle(){
    P1OUT ^= BIT0;
}
