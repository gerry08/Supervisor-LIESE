/*
 * FRAM_commands.h
 *
 *  Created on: 28 jun 2023
 *      Author: maxim y pablito xd
 */

#ifndef FRAM_COMMANDS_H_
#define FRAM_COMMANDS_H_


#include <stdint.h>
#include <msp430.h>

/*Drivers para la comunicacion con la memoria FRAM CY15B104Q
https://www.infineon.com/dgdl/Infineon-CY15B104Q_4-Mbit_(512_K_8)_Serial_(SPI)_F-RAM-DataSheet-v06_00-EN.pdf?fileId=8ac78c8c7d0d8da4017d0ecdc6684848*/
#define WREN   0x06
#define WRDI   0x04
#define RDSR   0x05
#define WRSR   0x01
#define READ   0x03
#define FSTRD  0x0B
#define WRITE  0x02
#define SLEEP  0xB9
#define RDID   0x9F

void divisor_byte();
void FRAM_write(int ADDRESS_1,int ADDRESS_2,int ADDRESS_3,int* arrayTx, int arrayTxSize);
void FRAM_read(int ADDRESS_1,int ADDRESS_2,int ADDRESS_3,uint16_t* arrayRx, int arrayRxSize);
void FRAM_erase(int ADDRESS_1,int ADDRESS_2,int ADDRESS_3,int Nbytes);


#endif /* FRAM_COMMANDS_H_ */
