#include <msp430.h>
#include <stdint.h>
#include "eUSCIA1_UART.h"
#include "STMF407xx_bootloaderCommands.h"
#include "eUSCIB0_SPI.h"
#include "FRAM_commands.h"

/*
 * Recibe un archivo .hex completo,
 * Los datos de interes de cada linea
 * se guardan en los vectores:
 * frame_1 (Primera linea de datos del archivo hex)
 * frame_2 (Segunda linea de datos del archivo hex)
 * y asi consecutivamente para las 5 lineas del archivo hex de prueba
 * */

//Configura UART
//- 8 bits de datos.
//- Sin bit de paridad.
//- 1 bit de stop.
//- 9600 baudrate.

/*
 * Trama recibida desde el dispositivo externo
 * data[0] - Numero de bytes de datos
 * dato[1] - Address MSB (Sin offset)
 * dato[2] - Address LSB (Sin offset)
 * dato[3] - Primer dato
 * dato[3+data[0]] - dato n (dato final)
 * dato[4+data[0]] - dato n (checksum)
 * */

#define SIZE_BUFFER 100
#define START_BYTE 0xF
#define END_BYTE 0xF0
#define ACK_BYTE 0x79
#define NACK_BYTE 0x7F


uint8_t update_Code_Buffer[SIZE_BUFFER];
uint8_t update_Code_Byte_Count = 0;
uint8_t update_Code_first_Byte = 1;
uint8_t update_Code_enable = 0;
uint8_t update_Code_frame_ready = 0;
//Usar un puntero global movil para la escritura en FRAM
uint16_t FRAM_WRT_PTR = 0x0000;
//y un puntero gobal fijo para lectura de FRAM
//Y el orgen de los 2
uint16_t StartFRAM_ProgramAddress = 0x0000;
uint8_t ready_frames_count = 0;

uint8_t frame_1[25] = {0};
uint8_t frame_2[25] = {0};
uint8_t frame_3[25] = {0};
uint8_t frame_4[25] = {0};
uint8_t frame_5[25] = {0};

uint16_t Each_Frame_address[50]={};     //Maximo 50 frames pueden ser recibidas y guardar el offset de la direccion donde va cada una

uint16_t FRAM_tstBuff[25] = {0};

//Esta funcion escribe en FRAM un frame a la vez
void FRAM_REPROG(uint8_t* Global_Buffer){

    int nByt = *(Global_Buffer);

    // Saving 16-bit FRAM Address pointer

     Each_Frame_address[ready_frames_count-1] = *(Global_Buffer+1);
     Each_Frame_address[ready_frames_count-1] = Each_Frame_address[ready_frames_count-1]<<8;
     Each_Frame_address[ready_frames_count-1] = Each_Frame_address[ready_frames_count-1] | (*(Global_Buffer+2)&0xFF);

//    FRAM_WRT_PTR = FRAM_WRT_PTR + 2;
//    if(ready_frames_count == 1){
//        FRAM_WRT_PTR = FRAM_WRT_PTR-2;
//        StartFRAM_ProgramAddress = FRAM_WRT_PTR;//En la direccion mandada se empieza a escribir en FRAM
//
//    }




    uint8_t data_chksum = nByt;

    int MSP2FRAMvB[25];//La longitud inicial del buffer debe ser mayor al numero de datos
                        //Ya con eso no hay pdos

    int j;

    for (j=0; j<nByt; j++){
        MSP2FRAMvB[j] = *(Global_Buffer+j+3);      //salta el numero de datos y la direccion
        data_chksum = data_chksum ^ *(MSP2FRAMvB+j);
    }

    /*Estructura de datos en FRAM
    *numero de datos
    *datos
    *checksum de datos
    */
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&nByt,1);
    FRAM_WRT_PTR++;
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,MSP2FRAMvB,nByt);
    FRAM_WRT_PTR=FRAM_WRT_PTR +nByt;
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&data_chksum,1);
    FRAM_WRT_PTR++;
}

void eUSCIA0_UART_send(int data_Tx){
    while((UCA0STATW & UCBUSY) == UCBUSY){}
    UCA0TXBUF = data_Tx; //Dato a enviar (pag.791) manual slau367p.pdf
}

void masterReprogramationRutine(uint32_t FRAM_initialAddress, uint32_t Flash_initialAddress1, int Rx_ready_buffers){
    uint32_t FRAM_actualAddress = FRAM_initialAddress;
    uint32_t Flash_actualAddress = Flash_initialAddress1;

    //Esta funcion asume que el respaldo del programa del master ya ha sido cargado en la FRAM.

    uint16_t FRAM2MSP_VB[25]; //Vector de Buffer de FRAM ---> MSP
    uint8_t MSP2Master_VB[25];//Vector de Buffer de MSP ---> Master La longitud del vector receptor simpre es mayor que la cantidad de datos Rx

    //int FRAMvectorBufferSize = sizeof(FRAMvectorBuffer)/sizeof(FRAMvectorBuffer[0]);
    unsigned int i;


    ACK= BootloaderAccess();
    for (i=0; i<Rx_ready_buffers; i++){

        uint16_t BFFSZ_HX;
            FRAM_read(((FRAM_actualAddress)>>16)&0xFF,((FRAM_actualAddress)>>8)&0xFF, FRAM_actualAddress&0xFF,&BFFSZ_HX, 1);
            FRAM_actualAddress++;
            int BufferVectorSize = (int) BFFSZ_HX;

            //uint16_t FRAM2MSP_VB[BufferVectorSize]; //Vector de Buffer de FRAM ---> MSP
            //uint8_t MSP2Master_VB[BufferVectorSize];//Vector de Buffer de MSP ---> Master


        FRAM_read(((FRAM_actualAddress)>>16)&0xFF,((FRAM_actualAddress)>>8)&0xFF, FRAM_actualAddress & 0xFF, FRAM2MSP_VB, BufferVectorSize);
        //Ejecutar alguna rutina para verificar la integridad de los datos (No se tiene que desarrollar ahora).
        //Hacer la conversion de 16 a 8 bits para que se puedan enviar bien los datos
        //Para mas optimizazion modificar las funciones de escritura y lectura de la FRAM a 8 bits
        //Aunque es poco probable que esto suceda ya que se necesita vaciar o llenar el buffer SPI de 16 bits para que la funcion termine.

        unsigned int j;
        for (j=0; j<=BufferVectorSize; j++){
            MSP2Master_VB[j] = FRAM2MSP_VB[j]&0xFF;}


        writeMemoryCommand(((Flash_actualAddress)>>16)&0xFFFF,(Flash_actualAddress)&0xFFFF , MSP2Master_VB, BufferVectorSize);
        FRAM_actualAddress = FRAM_actualAddress + BufferVectorSize+1;       //El +1 es para saltar el checksum
        Flash_actualAddress = Flash_actualAddress + BufferVectorSize;
    }

}

void MSP430_Clk_Config(){
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0 | DCORSEL;
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}

void eUSCIA0__UART_Init(){
    UCA0CTLW0 |= UCSWRST;
    UCA0CTLW0 |= UCSSEL__SMCLK;
    UCA0BRW |= 6;
    UCA0MCTLW |= 0x20<<8 | UCOS16 | 8<<4;

    P2SEL0 &= ~(BIT0 | BIT1);                   //P2SEL0.x = 0
    P2SEL1 |= BIT0 | BIT1;                      //P2SEL1.x = 1; Selecciona la función de UART en P2.1 y P2.0
    PM5CTL0 &= ~LOCKLPM5;

    UCA0CTLW0 &= ~UCSWRST;

    UCA0IE |= UCRXIE;                           //Habilita interrupción de recepción
                      //Habilita la las interrupciones enmascarables
    UCA0IFG &= ~UCRXIFG;
    _enable_interrupt();
}

void LED_Init(){
    P1DIR |= BIT0;
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

uint8_t Frame_Verify_Checksum(uint8_t data[]){
    uint8_t i = 0;
    uint8_t local_checksum = 0;
    uint8_t received_checksum = 0;
    for(i = 0; i<= data[0]+2; i++){
        local_checksum ^=  data[i];
    }
    received_checksum = data[data[0]+3];
    if(local_checksum == received_checksum){
        return 1;
    }else{
        return 0;
    }
}

void Split_Vector(uint8_t ready_frames_count){
    uint8_t i = 0;
    switch (ready_frames_count) {
        case 1:
            for(i = 0;i <= 25; i++){
                frame_1[i] = update_Code_Buffer[i];
            }
            break;
        case 2:
            for(i = 0;i <= 25; i++){
                frame_2[i] = update_Code_Buffer[i];
            }
            break;
        case 3:
            for(i = 0;i <= 25; i++){
                frame_3[i] = update_Code_Buffer[i];
            }
            break;
        case 4:
            for(i = 0;i <= 25; i++){
                frame_4[i] = update_Code_Buffer[i];
            }
            break;
        case 5:
            for(i = 0;i <= 25; i++){
                frame_5[i] = update_Code_Buffer[i];
            }
            break;
        default:
            break;
    }
    for(i = 0;i <= 25; i++){
        update_Code_Buffer[i] = 0;
    }
}

int main(void)
{


    WDTCTL = WDTPW | WDTHOLD;
    MSP430_Clk_Config();
    eUSCIA0__UART_Init();
    P1_Init();                  //Habilita pines GPIO para el patron de acceso al bootloader
    timer_Init();               //Habilita un timer para el patron de acceso al bootloader
    eUSCIA1_UART_Init();        //Habilita comunicacion UART para conectar con Flash
    eUSCIB0_SPI_init();
    LED_Init();
    LED_TurnOn();

    while(1){
        if(update_Code_enable == 1){
            if(update_Code_frame_ready == 1){
                if(Frame_Verify_Checksum(update_Code_Buffer) == 1){
                    ready_frames_count++;
                    //Split_Vector(ready_frames_count);

                    /*
                     * Aca va la funcion para la escritura en la FRAM
                     * **********************************************
                     * **********************************************
                     * **********************************************
                     * El dispositivo externo envía los datos con el siguiente orden
                     * update_Code_Buffer[0] - Numero de bytes de datos
                     * update_Code_Buffer[1] - Address MSB (Sin offset)
                     * update_Code_Buffer[2] - Address LSB (Sin offset)
                     * update_Code_Buffer[3] - Primer dato
                     * update_Code_Buffer[3+data[0]] - dato n (dato final)
                     * update_Code_Buffer[4+data[0]] - dato n (checksum)
                     *
                     * */


//                    if (ready_frames_count == 2){         //Prueba de funcionamiento de escritura semiautomatica en FRAM
//                        FRAM_REPROG(frame_2);
//                        FRAM_read(0x00,frame_2[1],frame_2[2], FRAM_tstBuff, frame_2[0]+2);}

                    FRAM_REPROG(update_Code_Buffer);
//                    if (ready_frames_count == 1){
//                        FRAM_read(0x00,update_Code_Buffer[1],update_Code_Buffer[2],FRAM_tstBuff,update_Code_Buffer[0]+2);
//
//                    }
//                    else{
//                    FRAM_read(0x00,update_Code_Buffer[1],update_Code_Buffer[2]+2,FRAM_tstBuff,update_Code_Buffer[0]+2);}
                    for(i = 0;i <= 25; i++){
                            update_Code_Buffer[i] = 0;
                        }
                    if (ready_frames_count == 4){
                        //masterReprogramationRutine(0x00000000,Each_Frame_address[0],ready_frames_count);
                        masterReprogramationRutine(0x00000000,0x08060000,ready_frames_count);
                    }

                    eUSCIA0_UART_send(ACK_BYTE);
                }else{
                    eUSCIA0_UART_send(NACK_BYTE);
                }
                LED_Toggle();
                update_Code_frame_ready = 0;
            }
        }else{
            ready_frames_count = 0;
        }
    }

    return 0;
}


#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void){
    UCA0IFG = 0;
    update_Code_frame_ready = 0;
    uint8_t start_byte_received = 0;

    if(update_Code_first_Byte == 1){ //¿Es una nueva trama?
        start_byte_received = UCA0RXBUF;
        if(start_byte_received == START_BYTE){
           update_Code_first_Byte = 0;
           P1OUT ^= BIT0;
           UCA0TXBUF = ACK_BYTE; //Dato a enviar (pag.791) manual slau367p.pdf
           update_Code_enable = 1; //Habilita programacion
        }else if(start_byte_received == END_BYTE){
           update_Code_first_Byte = 1;
           UCA0TXBUF = ACK_BYTE;
           update_Code_enable = 0; //Deshabilita reprogramación
        }else{//Si el primer byte no es el byte de inicio o el byte de fin, entonces es un dato.
            update_Code_first_Byte = 0;
            update_Code_Buffer[update_Code_Byte_Count] = start_byte_received;
            update_Code_Byte_Count++;
        }
    }else{
        update_Code_Buffer[update_Code_Byte_Count] = UCA0RXBUF;
        update_Code_Byte_Count++;
        if(update_Code_Byte_Count == 4 + update_Code_Buffer[0]){
            update_Code_Byte_Count = 0;
            update_Code_frame_ready = 1;
            update_Code_first_Byte = 1;
        }
    }
}









#pragma vector = PORT4_VECTOR
__interrupt void PORT4_ISR(void){
    P4IFG = 0; //limpia bandera de interrupcion
    P1OUT ^= BIT0;
}
