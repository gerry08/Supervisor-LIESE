#include <FRAM_commands.h>
#include "eUSCIB0_SPI.h"
#include <msp430.h>
#include <stdint.h>
#include <stdbool.h>
#include "STMF407xx_bootloaderCommands.h"
#include "eUSCIA1_UART.h"
#include "eUSCIA0_UART.h"
#include "TIMERA0.h"
#include "LED.h"
#include <stdbool.h>

/*
 * Recibe un archivo .hex completo,
 * Los datos de interes de cada linea
 * se guardan en los vectores:
 * frame_1 (Primera linea de datos del archivo hex)
 * frame_2 (Segunda linea de datos del archivo hex)
 * y asi consecutivamente para las 5 lineas del archivo hex de prueba
 * */

/* Dos  comunicaciones tipo UART
 * Interfaz serial para recepcion de datos de programa MSP <--- (puerto COM)PC
 * Comunicacion de transmision del programa principal MSP ---> Maestro
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

//

bool data_Frame = false; //0
bool end_Of_File = false; //1
bool extended_Segment_Adress_Frame = false; //2
bool start_Segment_Adress_Frame = false; //3
bool extended_Linear_Address_Frame = false; //4
bool start_Linear_Address_Frame = false; //5
bool CRC_Frame = false; //6
bool master_NumBytes_Frame = false; //7
bool master_NumFrames_Frame = false; //8

//Variables de interes para pablo
//uint16_t Master_Program_Num_Bytes_MSB = 0;
//uint16_t Master_Program_Num_Bytes_LSB = 0;
//uint16_t Master_Program_CRC_MSB = 0;
//uint16_t Master_Program_CRC_LSB = 0;

uint8_t master_Program_CRC[] = {0,0,0,0};       //Arreglo de bytes que almacenan el crc del programa en FRAM
uint8_t master_Program_CRC_Checksum_Local = 0;  //Checksum aux
uint8_t master_Program_CRC_Checksum_Received = 0;//Checksum aux
uint8_t master_Program_Num_Bytes[] = {1,2,3,4}; //Arreglo de bytes que almacenan el numero total de bytes del programa en FRAM
uint8_t master_Program_Num_Bytes_Checksum_Local = 0;//Checksum aux
uint8_t master_Program_Num_Bytes_Checksum_Received = 0;//Checksum aux
uint8_t master_Program_NumFrames[] = {0,0,0,0};
uint8_t master_NumFrames_Checksum_Received = 0;
uint8_t master_NumFrames_Checksum_Local = 0;
uint8_t response = 0;


uint8_t masterMemoryCorrupted = 0;
uint8_t interrupt_timer_Counter = 0;
uint8_t first_time = 1;
#define NUM_FRAMES_MASTER_CODE 852

#define SIZE_BUFFER 100             //Tamaño maximo de un buffer de datos por cada trama
//Bytes recibidos de PC mediante interrupciones de UART
#define START_BYTE 0xF              //Byte para empezar la reporgramacion
#define END_BYTE 0xF0               //Byte para terminar la reprogramacion
//Bytes recibidos del UC maestro para continuar la comunicacion
#define ACK_BYTE 0x79               //Byte ACK para comunicacion MSP-->maestro
#define NACK_BYTE 0x7F              //Byte NACK para comunicacion con el maestro

uint8_t update_Code_Buffer[SIZE_BUFFER] = {0};    //Buffer Global de datos recibidos de PC
uint8_t update_Code_Byte_Count = 0;         //Indice de bytes para el llenado del buffer global
uint8_t update_Code_first_Byte = 1;         //
uint8_t update_Code_end_Byte = 0;           //Indica que la terminal completo la carga del software a la FRAM
uint8_t update_Code_enable = 0;             //Indica en la rutina main que se han empezado a recibir datos para reprogramacion
uint8_t update_Code_frame_ready = 0;        //Indica si un frame de datos termino de recibirse 

//LAS LIMITACIONES DEL MSP MAS QUE RENDIMIENTO ES DE EL TAMAÑO DEL BUS DE DATOS PARA MANEJAR UN MAYOR NUMERO DE BYTES SEGUN EL PROGRAMA XDz
uint16_t FRAM_WRT_PTR = 0x0000;             //Puntero global movil para la escritura en FRAM
uint16_t StartFRAM_ProgramAddress = 0x0000; //Puntero de la primera localidad en FRAM para lectura del program
#define StartFlash_ProgramAddress 0x0000 //0x08060000 Sector 6 (configurable) dividir la direccion
#define Flash_Begin 0x0800
                                            //Apuntador truncado a 16 bits
                                            //Eso dice que solo podemos escribir 64 kbytes de memoria en flash
                                            //Sector 0 al 3
uint16_t ready_frames_count = 0;             //Conteo de las tramas recibidas de PC
                                            //Sirve para saber cuando parar de leer el programa almacenado en FRAM
                                            //66535 frames
                                            //total de 2 MBytes
uint16_t Each_Frame_address[50]={};     //Buffer global para el manejo del offset de direccion para el guardado en FLASH
                                        //Truncado a 50 frames recibidas

uint16_t FRAM_tstBuff[25] = {0};        //Variable de pruenba (omitir)

/******ESCRITURA DEL PROGRAMA EN FRAM***********/
//Esta funcion escribe frame a la vez
//Param Global_Buffer = update_Code_Buffer para cuando haya una recepcion de datos de PC


void FRAM_REPROG(uint8_t* Global_Buffer){

if(Global_Buffer == update_Code_Buffer){
/* Estructura de datos recibidos de PC
 * Numero de datos [1 byte]
 * OFFSET para FLASH [2 bytes]
 * datos [n bytes]
 * checksum [1 byte]
*/
    int nByt = *(Global_Buffer);    // Numero de datos

    // Saving 16-bit Flash OFFSET pointer

     Each_Frame_address[ready_frames_count-1] = *(Global_Buffer+1);
     Each_Frame_address[ready_frames_count-1] = Each_Frame_address[ready_frames_count-1]<<8;
     Each_Frame_address[ready_frames_count-1] = Each_Frame_address[ready_frames_count-1] | (*(Global_Buffer+2)&0xFF);

//    FRAM_WRT_PTR = FRAM_WRT_PTR + 2;
//    if(ready_frames_count == 1){
//        FRAM_WRT_PTR = FRAM_WRT_PTR-2;
//        StartFRAM_ProgramAddress = FRAM_WRT_PTR;//En la direccion mandada se empieza a escribir en FRAM
//
//    }

    uint8_t data_chksum = nByt;     //Checksum calculado internamente para guardar

    //Conversion del tipo de dato para escritura en FRAM
    
    int MSP2FRAMvB[100]; //Buffer local de datos para escritura en FRAM
                        //La longitud inicial del buffer debe ser mayor al numero de datos
                        //Ya con eso no hay pdos
    int j;

    for (j=0; j<nByt; j++){
        MSP2FRAMvB[j] = *(Global_Buffer+j+3);      //salta el numero de datos y la direccion
        data_chksum = data_chksum ^ *(MSP2FRAMvB+j);
    }

    /*Estructura de datos en FRAM
    *numero de datos
    *datos
    *checksum de datos calculado
    */
    /*
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&nByt,1);
    FRAM_WRT_PTR++;
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,MSP2FRAMvB,nByt);
    FRAM_WRT_PTR=FRAM_WRT_PTR +nByt;
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&data_chksum,1);
    FRAM_WRT_PTR++;
    */

    if(ready_frames_count == 1){
        FRAM_WRT_PTR = 0x0C;
    }
    FRAM_write(0x00,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&nByt,1);
    FRAM_WRT_PTR++;
    FRAM_write(0x00,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,MSP2FRAMvB,nByt);
    FRAM_WRT_PTR=FRAM_WRT_PTR +nByt;
    FRAM_write(0x00,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&data_chksum,1);
    FRAM_WRT_PTR++;


}
else{
    unsigned int k;
    int master_Program[4]={0};
    for(k=0;k<4;k++){
        master_Program[k]= *(Global_Buffer+k);
    }
    if(Global_Buffer == master_Program_CRC){
        FRAM_write(0x00,0x00,0x00,master_Program,4);
    }
    if(Global_Buffer == master_Program_Num_Bytes){
            FRAM_write(0x00,0x00,0x04,master_Program,4);
        }
    if (Global_Buffer == master_Program_NumFrames){
        FRAM_write(0x00,0x00,0x08,master_Program,4);
    }
}
}

void eUSCIA0_UART_send(int data_Tx){
    while((UCA0STATW & UCBUSY) == UCBUSY){}
    UCA0TXBUF = data_Tx; //Dato a enviar (pag.791) manual slau367p.pdf
}



/*******REPROGRAMACION DEL UC MAESTRO****************/
//Esta funcion escribe todos los frames guardados en la FRAM segun el numero de frames recibidos de PC
//param FRAM_initalAddress puntero de inicio del FRAM donde se almacena el programa (fijado en 0x0)
//param Flash_initialAddress puntero de inicio donde el programa sera cargado al Flash UC maestro 
// Adaptar para usar segun cada offset de frame.****
//param Rx_ready_buffers numero de frames almacenados en FRAM para un programa completo (inicia en 0x0 en un reset)

void masterReprogramationRutine(uint32_t FRAM_initialAddress, uint32_t Flash_initialAddress1, int Rx_ready_buffers){
    uint32_t FRAM_actualAddress = FRAM_initialAddress+12;
    uint32_t Flash_actualAddress = Flash_initialAddress1;

    //Esta funcion asume que el respaldo del programa del master ya ha sido cargado en la FRAM.

    uint16_t FRAM2MSP_VB[100]={0}; //Vector de Buffer de FRAM ---> MSP
    uint8_t MSP2Master_VB_CHK[100];//Vector de Buffer de MSP ---> Master
                                //Truncados a 25 bytes
    uint32_t MSP2Master_VB[100]={0};//Vector de Buffer de MSP ---> Master

    //int FRAMvectorBufferSize = sizeof(FRAMvectorBuffer)/sizeof(FRAMvectorBuffer[0]);
    unsigned int i;
    timer_Stop();
    eUSCIA1_UART_Init_Master_Reprog();

    uint8_t ACK= BootloaderAccess();
    eeraseCommand(0);       //Region de Flash para hacer pruebas (sector 7 0x08060000) en la nucleo F446RE

    for (i=0; i<Rx_ready_buffers; i++){
        //Lectura de frames almacenados en FRAM
        LED_Toggle();
        uint16_t BFFSZ_HX=0;  //Numero datos formato 16-bit
            FRAM_read(0x00,((FRAM_actualAddress)>>8)&0xFF, FRAM_actualAddress&0xFF,&BFFSZ_HX, 1);
            //FRAM_actualAddress++;
            int BufferVectorSize = (int) BFFSZ_HX;  //Numero de datos formato int

        FRAM_read(0x00,((FRAM_actualAddress)>>8)&0xFF, FRAM_actualAddress & 0xFF, FRAM2MSP_VB, BufferVectorSize+2);
        //Ejecutar alguna rutina para verificar la integridad de los datos (No se tiene que desarrollar ahora).
        //Hacer la conversion de 16 a 8 bits para que se puedan enviar bien los datos
        //Para mas optimizazion modificar las funciones de escritura y lectura de la FRAM a 8 bits
        //Formato de 16-bit necesario para que funcionen los  Drivers de SPI. La FIFO de rececpion SPI es de 16 bits

        //Conversion de formato para compatibilidad con el UART del maestro 16-->8-bit
        unsigned int j;
        for (j=0; j<=BufferVectorSize+2; j++){
                    MSP2Master_VB_CHK[j] = FRAM2MSP_VB[j]&0xFF;}
        if (Frame_Verify_Checksum(MSP2Master_VB_CHK,false) == 1){
            for (j=0; j<=BufferVectorSize; j++){
                        MSP2Master_VB[j] = MSP2Master_VB_CHK[j+1];}
            //Escritura UART hacia la Flash
        writeMemoryCommand(0x0800,(Flash_actualAddress)&0xFFFF , MSP2Master_VB, BufferVectorSize);
        FRAM_actualAddress = FRAM_actualAddress + BufferVectorSize+2;       //El +2 es para saltar el checksum y el numero de datos
        Flash_actualAddress = Flash_actualAddress + BufferVectorSize;
        }
        else {i=i-1;}
    }
    goCommand(0x0800,0x0000);
    eUSCIA1_UART_Init_MasterFunctionalStatus_Determination();
    //timer_Enable();
}

void MSP430_Clk_Config(){
    CSCTL0_H = CSKEY >> 8;
    CSCTL1 = DCOFSEL_0 | DCORSEL;
    CSCTL2 = SELA__VLOCLK | SELS__DCOCLK | SELM__DCOCLK;
    CSCTL3 = DIVA__1 | DIVS__1 | DIVM__1;
    CSCTL0_H = 0;
}


uint8_t Frame_Verify_Checksum(uint8_t data[], bool in_nout){
    uint8_t i = 0;
    uint8_t local_checksum = 0;
    uint8_t received_checksum = 0;
    if(in_nout){
    for(i = 0; i<= data[0]+2; i++){
        local_checksum ^=  data[i];
    }
    received_checksum = data[data[0]+3];
    }
    else{
        for(i = 0; i<= data[0]; i++){//Asi si toma en cuenta el numero de datos para el calculo del checksum
                local_checksum ^=  data[i];
            }
            received_checksum = data[data[0]+1];
    }
    if(local_checksum == received_checksum){
        return 1;
    }else{
        return 0;
    }
}

void monitoring_MasterCode_Update_Request(void){
    uint16_t i = 0;
    if(update_Code_enable == 1){
        if(update_Code_frame_ready == 1){
            if(Frame_Verify_Checksum(update_Code_Buffer,true) == 1){
                ready_frames_count++; //Variable que tiene la cuenta del programa
                //Split_Vector(ready_frames_count);

                //Almacenar programa recibido en FRAM
                FRAM_REPROG(update_Code_Buffer);
                //Se vacia el buffer global para actualizar al siguiente frame
                for(i = 0;i <= 33; i++){
                        update_Code_Buffer[i] = 0;
                    }
               /* if (ready_frames_count == 4){       //Aqui se puede cambiar de bandera para reprogramar el UC maestro con otro evento
                    //Asi debe estar para reprogramar en la direccion indicada por el hexfile
                    //masterReprogramationRutine(0x00000000,Each_Frame_address[0],ready_frames_count); //<------------------------------
                    masterReprogramationRutine(0x00000000,StartFlash_ProgramAddress,ready_frames_count);
                }*/

                eUSCIA0_UART_send(ACK_BYTE);
            }else{
                eUSCIA0_UART_send(NACK_BYTE);
            }
            LED_Toggle();
            update_Code_frame_ready = 0;
        }
    }else{
        //Reset de las variables de control y apuntadores
        if(ready_frames_count > 0){
            if(update_Code_end_Byte==1){
                FRAM_REPROG(master_Program_CRC);
                FRAM_REPROG(master_Program_Num_Bytes);
                FRAM_REPROG(master_Program_NumFrames);
                masterReprogramationRutine(0x0000,StartFlash_ProgramAddress,ready_frames_count);
            }
        }
        ready_frames_count = 0;
        FRAM_WRT_PTR = 0x0000;
    }
}

void monitoting_Master_Functional_Status(){
    if (masterMemoryCorrupted == 1 && update_Code_enable == 0){
        //Reconfigura UART
        //eUSCIA1_UART_Init_MasterFunctionalStatus_Determination();
        //Reprograma
        timer_Stop();
        ready_frames_count = NUM_FRAMES_MASTER_CODE;
        masterReprogramationRutine(0x0000,StartFlash_ProgramAddress,ready_frames_count);
        //goCommand(0x0800,0x0000);
        masterMemoryCorrupted = 0;
        _delay_cycles(50000);
        _delay_cycles(50000);
        //UCA1TXBUF = 0xFF;
        //timer_Enable();
    }
}

bool monitoring_Function_Init(){
    bool initialization_Ok = false;
    uint8_t i = 0;
    uint8_t error = 0;

    master_Program_Num_Bytes_Checksum_Local = master_Program_Num_Bytes[3] ^ master_Program_Num_Bytes[2] ^ master_Program_Num_Bytes[1] ^ master_Program_Num_Bytes[0];

    eUSCIA1_UART_send(0xFF);
    __delay_cycles(20000);
    response = UCA1RXBUF;
    if (response == ACK_BYTE) {
        for (i = 0; i < 4; i++) {
            eUSCIA1_UART_send(master_Program_Num_Bytes[i]);
        }
        eUSCIA1_UART_send(master_Program_Num_Bytes_Checksum_Local);
        __delay_cycles(20000);
        response = UCA1RXBUF;
        if (response) {
            eUSCIA1_UART_send(0xFF);
            __delay_cycles(10000);
            response = UCA1RXBUF;
            timer_Enable();
            if (response == ACK_BYTE) {
                initialization_Ok = true;

            }else {
                error++;
            }
        }else{
            error++;
        }
    }else{
        error++;
    }

    return initialization_Ok;
}

int main(void)

{
    uint8_t response = 0;
    uint32_t data_To_Write[] = {12,11,10,9,8,7,6,5,4,3,2,1,0};
    int size = (sizeof(data_To_Write))/(sizeof(data_To_Write[0]));
    //Inicializacion basica del MSP
    WDTCTL = WDTPW | WDTHOLD;
    //MSP430_Clk_Config();
    eUSCIA0__UART_Init();       //Habilita comunicacion UART para conectar con la PC 
    P1_Init();                  //Habilita pines GPIO para el patron de acceso al bootloader
    eUSCIA1_UART_Init_MasterFunctionalStatus_Determination();        //Habilita comunicacion UART para conectar con Flash
    eUSCIB0_SPI_init();         //Habilita comunicacion SPI para conectar con FRAM
    LED_Init();                 //Habilita led indicador por cada frame recibida de PC
    LED_TurnOn();
    timer_Init();
    timer_setPeriod(2);
    //timer_Enable();
    //Tiene que dar un disparo al watchdog
    //Enviar dato por UARTA1
    //UCA1TXBUF = 0xFF;

    //monitoring_Function_Init();
    /* Rutina de reprogramacion en main
    * verifica si se inicio una transmision de PC
    * verifica si el frame recibido ya esta guardado en el buffer global
    * verifica la integridad de los datos recibidos con el checksum
    * en caso de corrupcion de datos manda NACK
    * caso contrario reprograma la FRAM con la frame recibida
    * cuando todos los frames esten guardados en FRAM inicia la reprogramacion del UC maestro
    */
    //response = BootloaderAccess();
    //eeraseCommand(0);
    //writeMemoryCommand(0x0800, 0x0000, data_To_Write, size);
    //if(response == ACK_BYTE) LED_TurnOn();
    //goCommand(0x0800, 0x0000);

    //ready_frames_count = NUM_FRAMES_MASTER_CODE;
    //masterReprogramationRutine(0x0000,StartFlash_ProgramAddress,ready_frames_count);
    //goCommand(0x0800,0x0000);

    while(1){
        monitoring_MasterCode_Update_Request();
        monitoting_Master_Functional_Status();
    }

    return 0;
}

/*Rutina de interrupcion
* Habilita la reprogramacion en la rutina main a travez de una bandera
* en el momento que se recibe el byte de inicio
* Guarda los frames recibidos byte por byte en el buffer global
* y la deshabilita la reprog en el momento que se recibe el byte de fin 
*/
#pragma vector = USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void){
    UCA0IFG = 0;
    update_Code_frame_ready = 0;
    uint8_t start_byte_received = 0;

    if(update_Code_first_Byte == 1){ //Es una nueva trama?
        start_byte_received = UCA0RXBUF;
        if(start_byte_received == START_BYTE){
           update_Code_first_Byte = 1;
           P1OUT ^= BIT0;
           UCA0TXBUF = ACK_BYTE; //Dato a enviar (pag.791) manual slau367p.pdf
           update_Code_enable = 1; //Habilita programacion
        }else if(start_byte_received == END_BYTE){
           update_Code_first_Byte = 1;
           UCA0TXBUF = ACK_BYTE;
           update_Code_enable = 0; //Deshabilita reprogramacion
           update_Code_end_Byte = 1;
        }else if(start_byte_received == 0x00){//Si el primer byte no es el byte de inicio o el byte de fin, entonces es un dato.
            update_Code_first_Byte = 0;
            data_Frame = true;
            ///update_Code_Buffer[update_Code_Byte_Count] = start_byte_received;
            //update_Code_Byte_Count++;
        }else if(start_byte_received == 0x06){
            //CRC frame
            update_Code_first_Byte = 0;
            CRC_Frame = true;
        }else if(start_byte_received == 0x07){
            //master num bytes frame
            master_NumBytes_Frame = true;
            update_Code_first_Byte = 0;
        }else if(start_byte_received == 0x08){
            //master num frames frame
            master_NumFrames_Frame = true;
            update_Code_first_Byte = 0;
        }
    }else{
        if (data_Frame == true) {
            update_Code_Buffer[update_Code_Byte_Count] = UCA0RXBUF; //Termina de llenar el buffer con los demas datos de la trama
            update_Code_Byte_Count++;
            if(update_Code_Byte_Count == 4 + update_Code_Buffer[0]){
                update_Code_Byte_Count = 0;
                update_Code_frame_ready = 1;
                update_Code_first_Byte = 1;
                data_Frame = false;
            }
        }else if (CRC_Frame == true){
            if(update_Code_Byte_Count == 4){
                //Se recibe checksum
                master_Program_CRC_Checksum_Local = master_Program_CRC[3] ^ master_Program_CRC[2] ^ master_Program_CRC[1] ^ master_Program_CRC[0];
                uint8_t master_Program_CRC_Checksum_Received = UCA0RXBUF;
                if(master_Program_CRC_Checksum_Received == master_Program_CRC_Checksum_Local){
                    UCA0TXBUF = 0x69;
                }else{
                    UCA0TXBUF = NACK_BYTE;
                }
                update_Code_Byte_Count = 0;
                update_Code_first_Byte = 1;
                CRC_Frame = false;
            }else{
                master_Program_CRC[update_Code_Byte_Count] = UCA0RXBUF;
                update_Code_Byte_Count++;
            }
        }else if (master_NumBytes_Frame == true){
            if(update_Code_Byte_Count == 4){
                //Se recibe checksum
                master_Program_Num_Bytes_Checksum_Local = master_Program_Num_Bytes[3] ^ master_Program_Num_Bytes[2] ^ master_Program_Num_Bytes[1] ^ master_Program_Num_Bytes[0];
                uint8_t master_Program_Num_Bytes_Checksum_Received = UCA0RXBUF;
                if(master_Program_Num_Bytes_Checksum_Received == master_Program_Num_Bytes_Checksum_Local){
                    UCA0TXBUF = 0x69;
                }else{
                    UCA0TXBUF = NACK_BYTE;
                }
                update_Code_Byte_Count = 0;
                update_Code_first_Byte = 1;
                master_NumBytes_Frame = false;
            }else{
                master_Program_Num_Bytes[update_Code_Byte_Count] = UCA0RXBUF;
                update_Code_Byte_Count++;
            }
        }else if (master_NumFrames_Frame == true) {
            if(update_Code_Byte_Count == 4){
                //Se recibe checksum
                master_NumFrames_Checksum_Local = master_Program_NumFrames[3] ^ master_Program_NumFrames[2] ^ master_Program_NumFrames[1] ^ master_Program_NumFrames[0];
                uint8_t master_NumFrames_Checksum_Received = UCA0RXBUF;
                if(master_NumFrames_Checksum_Received == master_NumFrames_Checksum_Local){
                    UCA0TXBUF = 0x77;
                }else{
                    UCA0TXBUF = NACK_BYTE;
                }
                update_Code_Byte_Count = 0;
                update_Code_first_Byte = 1;
                master_NumFrames_Frame = false;
            }else{
                master_Program_NumFrames[update_Code_Byte_Count] = UCA0RXBUF;
                update_Code_Byte_Count++;
            }
        }
    }
}

// Timer0_A0 interrupt service routine
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = TIMER0_A0_VECTOR
__interrupt void Timer0_A0_ISR (void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(TIMER0_A0_VECTOR))) Timer0_A0_ISR (void)
#else
#error Compiler not supported!
#endif
{
  //Limpiar bandera de interrupcion
  masterMemoryCorrupted = 1; //Si se ejecuta la subrutina del timer quiere decir que la memoria se ha corrupto
  P1OUT ^= BIT0;
  interrupt_timer_Counter++;
}

#pragma vector = PORT4_VECTOR
__interrupt void PORT4_ISR(void){
    P4IFG = 0; //limpia bandera de interrupcion
    P1OUT ^= BIT0;
}
