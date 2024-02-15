
//#include "TIMERA0.h"

/*Drivers para establecer comunicacion UART con modulo UART1 manual del MSP430 slau367p cap30
https://www.ti.com.cn/lit/ug/slau367p/slau367p.pdf*/

//Comunicacion con el microcontrolador ST

#ifndef EUSCIA1_UART_H_
#define EUSCIA1_UART_H_

#include <msp430.h>
#include <stdint.h>

void eUSCIA1_UART_Init_Master_Reprog();
void eUSCIA1_UART_Init_MasterFunctionalStatus_Determination();
void eUSCIA1_UART_send(int data_Tx);
int eUSCIA1_UART_receiveACK_eerase();
int eUSCIA1_UART_receive();

#endif /* EUSCIA0_UART_H_ */
