#include "FRAM_commands.h"

//MSP SPI-MOSI P1.6 <-> 51 o ICSP-4
//MSP SPI-MISO P.1.7 <-> 50 o ICSP-1
//MSP SPI-CLK P2.2 <-> 52 o ICSP-3
//MSP SPI-SS P1.3 <-> 53 o
// Prueba funciona bien



void divisor_byte(){
    eUSCIB0_CS1_set_state(1);
    _delay_cycles(200);
    eUSCIB0_CS1_set_state(0);
}

//Recibe como 4to parametro (apuntador) el nombre del arreglo que se va a cargar
//Esto es la direccion del primer elemento
//A[]={0,1,2,3,4}
//A=&A[0]

//Operacion de escritura (p.8)
//Diagrama de tiempos (p.9)
void FRAM_write(int ADDRESS_1,int ADDRESS_2,int ADDRESS_3,int* arrayTx, int arrayTxSize){
    unsigned int i;
    eUSCIB0_CS1_set_state(0); //CS LOW
    eUSCIB0_SPI_writeByte(WREN);
    eUSCIB0_CS1_set_state(1);
    _delay_cycles(10);
    //Va un ciclo de CS aquí
    //divisor_byte();
    eUSCIB0_CS1_set_state(0);
    eUSCIB0_SPI_writeByte(WRITE);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_1);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_2);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_3);
    //divisor_byte();
    for(i=0;i<arrayTxSize;i++){
        eUSCIB0_SPI_writeByte(*(arrayTx+i));
//        eUSCIB0_SPI_writeByte(*(arrayTx));  //*(array+i)
//        arrayTx++;
        //divisor_byte();
    }
    eUSCIB0_CS1_set_state(1);
}
//Operacion de lectura (p.8)
//Diagrama de tiempos (p.9)
void FRAM_read(int ADDRESS_1,int ADDRESS_2,int ADDRESS_3,uint16_t* arrayRx, int arrayRxSize){
    unsigned int i;
    eUSCIB0_CS1_set_state(0);
    eUSCIB0_SPI_writeByte(READ);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_1);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_2);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_3);
    for(i=0;i<=arrayRxSize-1;i++){
        eUSCIB0_SPI_writeByte(0xFF); //Sigue escribiendo para mantener el reloj en funcionamiento
          *(arrayRx+i) = eUSCIB0_SPI_readByte();
//        *(arrayRx) = eUSCIB0_SPI_readByte();      //*(array+i)
//        arrayRx++;
    }
    eUSCIB0_CS1_set_state(1);
}
//Funcion de borrado es la funcion de escritura que escribe ceros
//las transmisiones son de 8 bits
void FRAM_erase(int ADDRESS_1,int ADDRESS_2,int ADDRESS_3,int Nbytes){
    unsigned int i;
    eUSCIB0_CS1_set_state(0); //CS LOW
    eUSCIB0_SPI_writeByte(WREN);
    eUSCIB0_CS1_set_state(1);
    _delay_cycles(10);
    //Va un ciclo de CS aquí
    //divisor_byte();
    eUSCIB0_CS1_set_state(0);
    eUSCIB0_SPI_writeByte(WRITE);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_1);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_2);
    //divisor_byte();
    eUSCIB0_SPI_writeByte(ADDRESS_3);
    //divisor_byte();
    for(i=0;i<Nbytes;i++){
        eUSCIB0_SPI_writeByte(0x00);
        //divisor_byte();
    }
    eUSCIB0_CS1_set_state(1);
}


