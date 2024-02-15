#include "eUSCIB0_SPI.h"

/*Drivers para establecer comunicacion SPI con modulo SPI0 manual del MSP430 slau367p cap 31
https://www.ti.com.cn/lit/ug/slau367p/slau367p.pdf*/

//Pruebaaaa Funciona bien


void eUSCIB0_SPI_init(){

    //Configuracion maestro de 4 pines (p.801)
    UCB0CTLW0 = UCSWRST; //resetea el modulo USCI
    UCB0CTLW0 |= UCSSEL__SMCLK; //Selecciona SMCLK como fuente de reloj (1 MHz)
    UCB0CTLW0 |= UCSYNC; //Configura modo sincrono.
    UCB0CTLW0 |=  UCMODE_0; //Se configura para 3 wire SPI.
    UCB0CTLW0 |= UCMST; //Se configura en modo maestro.
    UCB0CTLW0 |= UCMSB; //Configura MSB primero
    UCB0CTLW0 |= UCCKPH;  //(p.816)
    //Configuraci�n de la velocidad
    UCB0BRW = 180; //Velocidad de SMCLK, 1 Mbit/s (p.817)


    //configura puertos gral(p.371)
    //configura para SPI (p.800)
    PM5CTL0 &= ~LOCKLPM5;   
    //Configuraci�n de los GPIO
    P1SEL1 |= BIT6 | BIT7;             //Configura SOMI, MISO //La configuraci�n de puertos esta bien
    P2SEL1 |= BIT2;                    //UCB0CLK (p.369)

    //Configura un puerto IO para el pin CS de manera artificial
    P1DIR |= BIT0; //Configura P1.0 (LED) como salida;
    P1DIR |= BIT2; //Configura P1.2 como salida (CS)

    P1OUT &= ~BIT0; //Apaga LED.
    P1OUT |= BIT2; // (1.2) CS esta en alto, el esclavo esta desactivado.

    UCB0CTLW0 &= ~UCSWRST; //Pone a funcionar el modulo eUSCIB
}

void eUSCIB0_SPI_writeByte(int dato){       //El registro FIFO Tx del SPI es de 16-bit
    while((UCB0STATW & UCBUSY) == UCBUSY){} //esperar mientras el buffer de escritura no este vac�o.
        UCB0TXBUF = dato;                   //(p.811)
    while((UCB0STATW & UCBUSY) == UCBUSY){}
}

uint8_t eUSCIB0_SPI_readByte(){
    uint8_t dato = 0;
    _delay_cycles(100);
    while((UCB0STATW & UCBUSY) == UCBUSY){} //esperar mientras el buffer de escritura no este lleno.
    dato = UCB0RXBUF;                       //(p.810)
    while((UCB0STATW & UCBUSY) == UCBUSY){} //esperar mientras el buffer de escritura no este lleno.
    return dato;
}

//Se controla la longitud de la trama con la señal del CS
void eUSCIB0_CS1_set_state(uint8_t a){
    if (a==0) P1OUT &= ~BIT2; //CS = LOW
    if (a==1) P1OUT |= BIT2; //CS = HIGH
}
