#include <msp430.h>
#include "eUSCIA0_UART.h"

static unsigned int i;
static int ACK;
static int StartByte;

//Bandera de recepcion exitosa
uint8_t Rx_done; 

    //Pendiente volver variables locales

//Checksum de datos propio que se escribe en FRAM
uint8_t data_chk;
int Program_NBytes;                     //pendiente

//uint32_t FRAM_Start_Write = 0x00000000;     //pendiente


//uint32_t Program_StoreAdd = 0x08000000;     //pendiente


/* Formatos de datos para los apuntadores de Buffers de Memoria
 * Para leer de FRAM                    uint16_t
 * Para escribir a FRAM                 int
 * Para leer de UART0 PCRX              int pero tambien sirve uint8_t
 * Para escribir a FLASH de la ST       int pero tambien sirve uint8_t
 */


//Max primero guarda los datos en crudo en una variable global y despues yo tengo que darles tratamiento
//pa guardarlos en FRAM

//funcion con la intension de vaciar el buffer vector donde van los datos de programa
//en caso de que se hallan corrompido y se tenga que volver a escribir.
//O en caso de recibir un nuevo vector de diferente tamaño

//Tratamiento de datos para escribir FRAM
//Formato para escribir en FRAM: int
//Esta funcion asume que la linea de programa ya esta guardada en un buffer global de 8 bytes
//Datos crudos recibidos de PC = n  add1 add2 data0 data..... chksumtotal
void FRAM_REPROG(uint8_t* Global_Buffer){

    int nByt = *(Global_Buffer);
    int MSP2FRAM_VBuff[nByt];
    uint8_t data_chksum = nByt;
    
    // Setting 16-bit FRAM Address pointer
    uint16_t FRAM_WRT_PTR = *(Global_Buffer+2);
    FRAM_WRT_PTR = (FRAM_WRT_PTR <<8)| *(Global_Buffer+1);
    
    int j;

    for (j=0; j<nByt; j++){
        MSP2FRAM_VBuff[j] = *(Global_Buffer+j+3);      //salta el numero de datos y la direccion
        data_chksum = data_chksum ^ *(MSP2FRAM_VBuff+j);
    }

    /*Estructura de datos en FRAM
    *numero de datos
    *datos
    *checksum de datos 
    */
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&nByt,1);
    FRAM_WRT_PTR++;
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,MSP2FRAM_VBuff,nByt);
    FRAM_WRT_PTR=FRAM_WRT_PTR +nByt;
    FRAM_write((FRAM_WRT_PTR>>16)&0xFF,(FRAM_WRT_PTR>>8)&0xFF,FRAM_WRT_PTR&0xFF,&data_chksum,1);
        
}


//void prereceive_reset(uint8_t* PC2MSP_VB, int DataSize){
//
//    for(i=0; i<=DataSize-1; i++){
//        *(PC2MSP_VB+i)=0;
//    }
//}
//
////Regresa el checksum de datos
////Recibir los datos que se guardan en FRAM
//uint8_t receive_ProgramdataRx(uint8_t* PC2MSP_VB, int DataSize, uint8_t CurrentChecksum){
//Rx_done=0;
//uint16_t checksum = CurrentChecksum;
//uint8_t Hx_chksum;
//uint8_t data_check=DataSize;
//    for (i=1;i<=DataSize;i++){                        //Aqui se puede cambiar la posicion en la que se va a leer el checsum -1,-2-3
//        *(PC2MSP_VB+i) = eUSCIA0_UART_receiveACK_eerase();   //*(array-1-i)
//    checksum= checksum ^ *(PC2MSP_VB+i);
//    data_chk=data_chk ^ *(PC2MSP_VB+i); }
//
//    Hx_chksum = eUSCIA0_UART_receiveACK_eerase();   //Checksum recibido
//    //Comprobacion del checksum total
//    if (checksum == Hx_chksum){                    //HAcer el propio checsum
//        Rx_done =1;
//        return data_check;
//    }
//    else{
//        return 0;       //Datos corrompidos
//    }
//
//    //La comprobación del checksum se puede hacer sumando todos los bytes de datos incluyendo el checksum
//    //El LSB del resultado debe ser 0x00
//}
//
////Recibir datos en crudo
////Regresa el chechsum de datos
////void receive_PC_Frame(uint16_t* Program_StoreAdd, int Program_NBytes, uint8_t data_chk){
//uint8_t receive_PC_Frame(){
//
//        //while(Rx_done==0){                  //Check
//            //Repetir la trasmision
//            //uint8_t Program_NBytes;
//
//
//            uint8_t data_check;
//            uint8_t Hx_address_1;
//            uint8_t Hx_address_2;
//            uint8_t CurrentChecksum;
//
//            //Recibe Dirección
//            Hx_address_1 = eUSCIA0_UART_receiveACK_eerase();
//            Hx_address_2 = eUSCIA0_UART_receiveACK_eerase();
//            //Recibe número de bytes
//            Program_NBytes = eUSCIA0_UART_receiveACK_eerase();
//            prereceive_reset(Program_StoreAdd,Program_NBytes);      //por si los datos estan sobre escritos en el buffer
//            //Calcula lo que llevamos recibido para el checkssum total
//            CurrentChecksum = Hx_address_1 ^ Hx_address_2 ^ Program_NBytes;
//            data_check = receive_ProgramdataRx(Program_StoreAdd,Program_NBytes,CurrentChecksum);
//            //Aqui ya debio de haber cambiado la bandera Rx_done
//    //}
//
//    //Datos corrompidos
//    while (Rx_done == 0){
//        prereceive_reset(Program_StoreAdd,Program_NBytes);      //por si los datos estan sobre escritos en el buffer
//        eUSCIA0_UART_send(0x7F);        //NACK
//        data_check = receive_ProgramdataRx(Program_StoreAdd,Program_NBytes,CurrentChecksum);
//    }
//
//    return data_check;
//    //Empieza a escribir en FRAM
//
//}
//
//
//
//
//
////Proceso de Reprogramacion
////Estructura Tx del PC
//// Add_MSB -- Add_LSB -- NumDatos -- Dato1 -- Daton -- Checksum
//
////En esta funcion se hace el manejo de direcciones de FRAM y de Flash
////Tambien hace el manejo de los formatos de buffers de memoria
////HAciendo diferentes buffers dependiendo de la funcion que se vaya a ocupar.
////Aqui van los nombres de MSP2FRAM *+++++****************************
////Trabaja por poleo cuando toca recibir byte de inicio y final
//
//void FRAM_Backup_Receive(){
//
//
//    if (eUSCIA0_UART_receiveACK_eerase() == 0x0F){      //Startbyte
//
//            data_chk = receive_PC_Frame();
//
//            //Empieza a escribir el codigo recibido en FRAM
//    FRAM_REPROG(FRAM_Start_Write, Program_StoreAdd);              //Probar la parte de FRAM**
//
//    }
//    //Recibido el primer paquete de datos
//
//    //Repetir el procedimiento hasta que se envie el byte de fin
//            while (eUSCIA0_UART_receiveACK_eerase()&&0xF0==0){
//                FRAM_Start_Write = FRAM_Start_Write + Program_NBytes;
//                //Listo para recibir la Siguiente linea de codigo
//                eUSCIA0_UART_send(0x79);      //ACK
//                data_chk = receive_PC_Frame();
//
//    //Empieza a escribir el codigo recibido en FRAM
//    FRAM_REPROG(FRAM_Start_Write, Program_StoreAdd);              //Probar la parte de FRAM**
//
//}
//}
//
//
//
//
//void masterReprogramationRutine(uint32_t FRAM_initialAddress, uint32_t Flash_initialAddress1){
//    uint16_t masterPSZ;
//    FRAM_read(((FRAM_initialAddress)>>16)&0xFF,((FRAM_initialAddress)>>8)&0xFF, FRAM_initialAddress&0xFF,&masterPSZ, 1);
//    FRAM_initialAddress++;
//    int BufferVectorSize = (int) masterPSZ;
//
//    //Esta funcion asume que el respaldo del programa del master ya ha sido cargado en la FRAM.
//    int i;
//    uint16_t FRAM2MSP_VB[BufferVectorSize]; //Vector de Buffer de FRAM ---> MSP
//    uint8_t MSP2Master_VB[BufferVectorSize];//Vector de Buffer de MSP ---> Master
//    //int FRAMvectorBufferSize = sizeof(FRAMvectorBuffer)/sizeof(FRAMvectorBuffer[0]);
//    unsigned int j;
//    uint32_t FRAM_actualAddress = FRAM_initialAddress;
//    uint32_t Flash_actualAddress = Flash_initialAddress1;
//    ACK= BootloaderAccess();
//    for (i = 0; i < BufferVectorSize-1; i++) {      //El menos 1 es para que se salte la lectura del checksum
//        FRAM_read(((FRAM_actualAddress)>>16)&0xFF,((FRAM_actualAddress)>>8)&0xFF, FRAM_actualAddress & 0xFF, FRAM2MSP_VB, (int)BufferVectorSize);
//        //Ejecutar alguna rutina para verificar la integridad de los datos (No se tiene que desarrollar ahora).
//        //Hacer la conversion de 16 a 8 bits para que se puedan enviar bien los datos
//        //Para mas optimizazion modificar las funciones de escritura y lectura de la FRAM a 8 bits
//        //Aunque es poco probable que esto suceda ya que se necesita vaciar o llenar el buffer SPI de 16 bits para que la funcion termine.
//        for (j=0; j<=BufferVectorSize; j++){
//            MSP2Master_VB[j] = FRAM2MSP_VB[j]&0xFF;
//        }
//
//        writeMemoryCommand(((Flash_actualAddress)>>16)&0xFFFF,(Flash_actualAddress)&0xFFFF , MSP2Master_VB, BufferVectorSize);
//        FRAM_actualAddress = FRAM_actualAddress + BufferVectorSize;
//        Flash_actualAddress = Flash_actualAddress + BufferVectorSize;
//    }
//}
//
//
//
//
//void receivePrincipalComputerData(uint8_t *IncAdd){
//    uint8_t dataCheck;
//    uint8_t checksum;
//    uint32_t FRAM_nextWAddress;
//    uint8_t dataX[NumDataRx];
//    //mientras la entrada de control sea 1 :
//    while (P4IN == BIT2){//??????
//        dataX[0] = eUSCIA0_UART_receive();//Se recibe dato 1
//        dataX[1] = eUSCIA0_UART_receive();//Se recibe dato 2
//        dataX[2] = eUSCIA0_UART_receive();//Se recibe dato 3
//        dataX[3] = eUSCIA0_UART_receive();//Se recibe dato 4
//        checksum = eUSCIA0_UART_receive();//Se recibe checksum
//        //comprobar checksum
//        dataCheck = dataX[0] + dataX[1] + dataX[2] + dataX[3] + checksum;
//        //Si los datos se recibieron correctamente data check tiene que ser 0xFF
//        if(dataCheck == 0xFF){
//            FRAM_nextWAddress=FRAM_startAddress+IncAdd;
//            FRAM_write((FRAM_nextWAddress>>24)&0xFF,(FRAM_nextWAddress>>16)&0xFF,FRAM_nextWAddress&0xFF,dataX,NumDataRx);
//
//            eUSCIA0_UART_send(0X79); //contesta bit de ACK
//            //La computadora principal debera de enviar los cuatro bytes siguientes
//        }else{
//            eUSCIA0_UART_send(0X7F); //bit NACK
//        }
//    }
//
//}

