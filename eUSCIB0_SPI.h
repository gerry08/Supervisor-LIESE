/*
 * eUSCIB0_SPI.h
 *
 *  Created on: 28 jun 2023
 *      Author: maxim
 */

#ifndef EUSCIB0_SPI_H_
#define EUSCIB0_SPI_H_

#include <msp430.h>
#include <stdint.h>

void eUSCIB0_SPI_init();
void eUSCIB0_SPI_writeByte(int dato);
uint8_t eUSCIB0_SPI_readByte();
void eUSCIB0_CS1_set_state(uint8_t a);

#endif /* EUSCIB0_SPI_H_ */
