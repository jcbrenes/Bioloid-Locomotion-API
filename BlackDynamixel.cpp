//============================================================================
// Name        : BlackDynamixel.cpp
// Author      : David Sisternes Roses
// Version     : 1.0
// Copyright   : Free_Green_Team
// Description : Librería para el manejo y manupulación de la comuncación TTL
//               con servomotores Dynamixel y BeagleBone Black Rev C
//============================================================================

#include "BlackDynamixel.h"

/* Declaración de Variables necesarias */
serialib Uart;

/* Funcion Open_Connection */
bool Open_Connection(const char *Device,const unsigned int Bauds){

	int temp=0;

	printf("\n>>> Abriendo serial port...");

	temp = Uart.Open(Device,Bauds);

	if (temp!=1) {
	 printf ("\n>>> Error abriendo serial port %s!!! Error: %d",Device, temp);
	 return temp;
	}
	printf("\n>>> Serial port %s abierto correctamente!!!\n",Device);

	return true;
}

/* Función Close_Connection */
void Close_Connection(void){

   Uart.Close();         /* Se cierra el puerto especificado */
   printf("\n>>> Serial port cerrado correctamente!!!\n");

}

/* Función Send_Data */
void Send_Data(uint8_t *packetOut,uint16_t length){

	uint16_t m=0;

	printf("\nPacket_Out:"); //DEBUGG!!!
	for(m=0;m<length;m++){
	 printf(" %x",packetOut[m]);
	 }

	for(m=0;m<length;m++){
		 Uart.WriteChar((char)(packetOut[m]));
	 }

	 printf("\n>>> Packet enviado correctamente!\n");


}

/* Función Received_Data */
uint8_t Received_Data(uint8_t *destination, uint16_t max_data){

	char byte[2] = {0x00,0x00};
	uint8_t num_bytes=0,found=0,i=0;
    int16_t j=0,m=0,ms=5;

	j=-1;
   while((j<=max_data)&&(found==0)){

	 Uart.ReadChar(byte,ms);
	 j++;

	if(byte[0]==0xff){   //Se busca el primer byte de la cabezera
		 destination[0]=(char)byte[0];
    	 byte[0]=0x00;
    	 num_bytes++;
    	 Uart.ReadChar(byte,ms);
    	 j++;

    	 if(byte[0]==0xff){ //Se busca el segundo byte de la cabezera
    		destination[1]=(char)byte[0];
    		byte[0]=0x00;
    		num_bytes++;
    	    Uart.ReadChar(byte,ms);
    		j++;

    		if(byte[0]!=0xff){ //Se comprueba que no sea de nuevo 0xff y se busca el tamaño del mensaje

    			destination[2]=(char)byte[0];
    			byte[0]=0x00;
    			num_bytes++;
    			Uart.ReadChar(byte,ms);
    			j++;
    			destination[3]=(char)byte[0];
    			byte[0]=0x00;
    	        num_bytes++;
    	        for(i=0; i<(int)destination[3]; i++){
    	        	Uart.ReadChar(byte,ms);
    	        	j++;
    	        	destination[4+i]=(char)byte[0];
    	        	byte[0]=0x00;
    	        	num_bytes++;
    	        }
    			found=1;
    		}

    	 }else{
    		 byte[0]=0x00;
    		 byte[1]=0x00;
    		 num_bytes=0;
    		 found=0;
    		 destination[0]=0x00;
    		 destination[1]=0x00;
    	 }


     }else{
    	 byte[0]=0x00;
    	 byte[1]=0x00;
    	 num_bytes=0;
    	 found=0;
    	 destination[0]=0x00;
     }

   }
	//printf("\nfound: %d",found);
	//printf("\nnum_bytes: %d",num_bytes);
   if(num_bytes !=0){
	 printf("\nData_In: "); //DEBUGG!!!
	  for(m=0;m<num_bytes;m++){
	    printf(" %x",destination[m]);
	   }
   }
	return(num_bytes);
}
/* FIN_BLACKDYNAMIXEL */


