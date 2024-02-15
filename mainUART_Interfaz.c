#include <msp430.h>
#include <stdint.h>


#define NumDataRx 4
uint8_t RxByt;
uint8_t Tx_Qt;                                  //numero de transferencias Pasarlo como parametro para sacar el incremento
uint32_t command_dataRx[NumDataRx];              //Aqui antes estaban declarados como uint32_t
uint32_t dataW[]={0x10,0x17,0x22,0x33};
uint8_t FRAM_R_Buff[4];

#include <TIMERA0.h>
#include <eUSCIA1_UART.h>
#include <STMF407xx_bootloaderCommands.h>
#include <eUSCIA0_UART.h>
#include <eUSCIB0_SPI.h>
#include <FRAM_commands.h>
#include <RTCB.h>





void receivePrincipalComputerData(uint8_t *IncAdd){
    uint8_t dataCheck;
    uint8_t checksum;
    uint32_t FRAM_nextWAddress;
    uint8_t dataX[NumDataRx];
    //mientras la entrada de control sea 1 :
    while (P4IN == BIT2){
        dataX[0] = eUSCIA0_UART_receive();//Se recibe dato 1
        dataX[1] = eUSCIA0_UART_receive();//Se recibe dato 2
        dataX[2] = eUSCIA0_UART_receive();//Se recibe dato 3
        dataX[3] = eUSCIA0_UART_receive();//Se recibe dato 4
        checksum = eUSCIA0_UART_receive();//Se recibe checksum
        //comprobar checksum
        dataCheck = dataX[0] + dataX[1] + dataX[2] + dataX[3] + checksum;
        //Si los datos se recibieron correctamente data check tiene que ser 0xFF
        if(dataCheck == 0xFF){
            FRAM_nextWAddress=FRAM_startAddress+IncAdd;
            FRAM_write((FRAM_nextWAddress>>24)&0xFF,(FRAM_nextWAddress>>16)&0xFF,FRAM_nextWAddress&0xFF,dataX,NumDataRx);

            eUSCIA0_UART_send(0X79); //contesta bit de ACK
            //La computadora principal debera de enviar los cuatro bytes siguientes
        }else{
            eUSCIA0_UART_send(0X7F); //bit NACK
        }
    }

}

int main(void){
    WDTCTL = WDTPW | WDTHOLD;   // stop watchdog timer
        //PM5CTL0 &= ~LOCKLPM5;

    eUSCIA0_UART_Init();

    eUSCIB0_SPI_init();
    int i,ACK;
    //Receive Start Byte

    while(1){

        RxByt = eUSCIA0_UART_receive();
        if (RxByt == 0x0F){
            Tx_Qt=0x0;                     //Direccion inicial FRAM donde se guardara
            receivePrincipalComputerData(Tx_Qt);

        }
        else if(RxByt == 0xF0){
            //Aqui inicia la carga del programa al STM32
            //En proceso...(terminar)****
            for ( i=0; i<Tx_Qt; i++){
            //Primero leer 4 bytes de FRAM
            //Usar el mismo bufer de 4 bytes
           FRAM_read(0x00,0x90,4*i,FRAM_R_Buff,4);

           //PRUEBAS
           //Primero manda un areglo mucltiplo de 4 bytes para probar la funcionalidad apuntador

           //Despues que le entiendas al hexfile saca un programa de la memoria flash del st
           //pasalo a hexfile
           //transmitelo por uart a travez del bootloader para probar la funcionalidad
           //con datos reales y programas ejecutables como el blink.
           //https://www.fischl.de/hex_checksum_calculator/
           int genC;
           uint32_t FRAM_REACO[]={0x00,0x00,0x00,0x00};
           for (genC=0;genC<4;genC++)
               FRAM_REACO[genC] = (uint32_t)FRAM_R_Buff[genC];
            //Escribir el bootloader por uart
            int ACK = BootloaderAccess();
            while(!ACK);
                writeMemoryCommand(0x0806,0x01*i,FRAM_REACO,4);
            }
        }
        else{
            Tx_Qt++;
            receivePrincipalComputerData(Tx_Qt);
        }
    }
}

main(){
    while(StartByte=eUSCIA0_UART_receive();){
        //Inicia reprogramacion
    }
}