/*
 * STMF407xx_bootloaderCommands.h
 *
 *  Created on: 28 jun 2023
 *      Author: maxim
 */

#ifndef STMF407XX_BOOTLOADERCOMMANDS_H_
#define STMF407XX_BOOTLOADERCOMMANDS_H_

#include <msp430.h>
#include <stdint.h>
//Drivers para comunicacion UART con la memoria Flash del UC maestro
//Notas de aplicacion AN3155 y AN2606
//https://stm32duinoforum.com/forum/resource/en/application_note/cd00264342.pdf
//https://www.st.com/resource/en/application_note/cd00167594-stm32-microcontroller-system-memory-boot-mode-stmicroelectronics.pdf

//Lista de comandos Nota AN3155 (p.7)
#define GET                 0x00
#define GET_V_RPS           0x01
#define GET_ID              0x02
#define READ_MEMORY         0x11
#define GO                  0x21
#define WRITE_MEMORY        0x31
#define E_ERASE             0x44
#define WRITE_PROTECT       0x63
#define WRITE_UNPROTECT     0x73
#define READOUT_PROTECT     0x82
#define READOUT_UNPROTECT   0x92
#define GET_CHEKSUM         0xA1

//Puerto encargado de establecer los niveles para los pines de acceso al bootloader
void P1_Init();
//Metodo de acceso a Bootloader
//Nota AN2606 (p.127)
//Patron 1 (p.29)
int BootloaderAccess(void);

static void sendCommand(int command);

static void receiveCommand_dataRx(uint32_t* arrayRx2, int arrayRxSize2);

static void send_startAddress(int ADDRESS_MSB,int ADDRESS_LSB);

static void send_4bytes_wChecksum(int WORD_MSB,int WORD_LSB);

static void writeData (uint32_t* arrayTx2, int arrayTxSize2);

void userSendCommand(int command,uint32_t* arrayRx, int arrayRxSize);

//Metodo de lectura de memoria Flash
//Nota AN3155 (p.13)
void readMemoryCommand(int ADDRESS_MSB,int ADDRESS_LSB,uint32_t* arrayRx, int arrayRxSize);

//Metodo de escritura a memoria Flash
//Nota AN3155 (p.18)
void writeMemoryCommand(int ADDRESS_MSB,int ADDRESS_LSB,uint32_t* arrayTx, int arrayTxSize);
//Metodo de salto a direccion para ejecucion de instrucciones
//Nota AN3155 (p.16)
void goCommand(int ADDRESS_MSB,int ADDRESS_LSB);
/**
 * @brief Esta funci�n borra un sector X de la memoria FLASH
 * @param FlashSectorCode codigo del sector de memoria a borrar
 */

//Metodo de borrado de memoria Flash
//Nota AN3155 (p.21)
void eeraseCommand(int FlashSectorCode);

void getChecksumCommand(int ADDRESS_MSB,int ADDRESS_LSB,
                        int WORD32b_MSB,int WORD32b_LSB,
                        int CRCpolynomial_MSB, int CRCpolynomial_LSB,
                        int CRCinitialValue_MSB, int CRCinitialValue_LSB,uint32_t* arrayRx, int arrayRxSize);



#endif /* STMF407XX_BOOTLOADERCOMMANDS_H_ */
