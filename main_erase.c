#include <msp430.h>
#include <stdint.h>
#include "eUSCIA1_UART.h"
#include "STMF407xx_bootloaderCommands.h"
#include "eUSCIB0_SPI.h"
#include  "FRAM_commands.h"
#include "TIMERA0.h"


uint16_t REad_Bff[36]={0};
main(){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
        //PM5CTL0 &= ~LOCKLPM5;

        P1_Init();                  //Habilita pines GPIO para el patron de acceso al bootloader
        timer_Init();               //Habilita un timer para el patron de acceso al bootloader
        eUSCIA1_UART_Init();        //Habilita comunicacion UART para conectar con Flash
        eUSCIB0_SPI_init();         //Habilita comunicacion SPI para conectar con FRAM

        //ACK = BootloaderAccess();

        //eeraseCommand(7);

        //FRAM_erase(0x00,0x00,0x00,36);
        FRAM_read(0x00,0x00,0x00,REad_Bff,36);
        while(1);
}
