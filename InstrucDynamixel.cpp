//============================================================================
// Name        : InstrucDynamixel.cpp
// Author      : David Sisternes Roses
// Version     : 1.0
// Copyright   : Free_Green_Team
// Description : Librería para el manejo y manupulación de instruciones en los
//               servomotores Dynamixel
//============================================================================

#include "InstrucDynamixel.h"
//#include "DynamixelAXDef.h"

/* Declaración variables necesarias */

uint8_t  parameters[DATA_SIZE];
uint8_t  packet_Out[DATA_SIZE];
uint8_t  packet_In[BUFFER_SIZE];
extern uint8_t  Status_Return_Value = ALL;

/* Función Get_Packet */
void Get_Packet(uint8_t *param,uint8_t packetOut[DATA_SIZE],uint16_t *length){

  uint16_t i=0;

  packetOut[0]={0xff}; /* Se añade la cabecera de start de la trama */
  packetOut[1]={0xff};

   for (i=0; i<(*length);i++){    /* Se añaden los elementos de la trama junto con la cabecera */
	   packetOut[i+2]= param[i];
   }
   packetOut[(*length)+2]= Get_Checksum(param,*length); /* Se calcula el CheckSum y se añade a la trama */
   *length= (*length)+3;                                /* Se actualiza la longitud de los datos del mensaje */
}

/* Función Get_Checksum */
uint8_t Get_Checksum(uint8_t *p,uint16_t length){

  uint16_t i=0,sum=0;
  uint8_t  checksum=0;

   for (i=0; i<length;i++){  /* Suma los elementos que se van a enviar */
	   sum += p[i];
    }
   checksum = (sum & 0x00FF); /* Se obtiene el bit menos significativo */

  return ~(checksum); /* Retorna el bit negado */
}

/* Funcuión Paralen */
uint16_t Paralen(uint8_t *param){ /* Cuenta el numero de elementos de un vector hasta encontrarse con el carácter }*/

	uint16_t cnt=0;
    while(param[cnt]!='}'){
    	cnt++;
     }
	return cnt;
}

/* Función Ping_Servo */
uint8_t Ping_Servo(uint8_t id){

	uint16_t lng=2,num_bytes=0;

	memset(packet_Out,0,DATA_SIZE); /* Resetar los arrays globales */
	memset(parameters,0,DATA_SIZE);
	memset(packet_In,0,DATA_SIZE);

	parameters[0]= (char)id;
	parameters[1]= (char)lng;
	parameters[2]= INST_PING;

	lng=lng+1;                    /* Se suma uno para contemplar el parametro del Id en la longitud del cálculo de la trama */

	Get_Packet(parameters,packet_Out,&lng); /* Se monta la trama y se prepara para el envío, lng actualiza su valor */

	Send_Data(packet_Out,lng);  /* Se realiza el envío*/

	if(id== 0xfe || Status_Return_Value != ALL){
      return (0x00);
	}else
	  {
	    num_bytes = Received_Data(packet_In,BUFFER_SIZE-1); // Se lee lo que haya en el buffer de entrada.

        if(num_bytes !=0){
		    return (packet_In[4]);  //Error que envía el dynamixel.
         }else
          {
    	    return (0x80);          //Error No habido respuesta.
          }
	  }
}
/* Funcuión Reset_Servo */
uint8_t Reset_Servo(uint8_t id){

	uint16_t lng=2,num_bytes=0;

	memset(packet_Out,0,DATA_SIZE); /* Resetar los arrays globales */
	memset(parameters,0,DATA_SIZE);
	memset(packet_In,0,DATA_SIZE);

	parameters[0]= (char)id;
	parameters[1]= (char)2;
	parameters[2]= INST_RESET;

	lng=lng+1;                    /* Se suma uno para contemplar el parametro del Id en la longitud del cálculo de la trama */

	Get_Packet(parameters,packet_Out,&lng); /* Se monta la trama y se prepara para el envío, lng actualiza su valor */

	Send_Data(packet_Out,lng);  /* Se realiza el envío*/

	if(id== 0xfe || Status_Return_Value != ALL){
	      return (0x00);
	 }else
	   {
		 num_bytes = Received_Data(packet_In,BUFFER_SIZE-1); // Se lee lo que haya en el buffer de entrada.

	      if(num_bytes !=0){
			  return (packet_In[4]);  //Error que envía el dynamixel.
	       }else
	        {
	          return (0x80);          //Error No habido respuesta.
	        }
	   }
}

/* Función Write_Servo */
uint8_t Write_Servo(uint8_t id,uint16_t length,uint8_t instruction,uint8_t *param){

   uint16_t lng=0, i=0,num_bytes=0;
   memset(packet_Out,0,DATA_SIZE);  /* Resetar los arrays globales */
   memset(parameters,0,DATA_SIZE);
   memset(packet_In,0,DATA_SIZE);


   if(length == 0x00) {  /* Obtenemos  la longitud de los parametros a enviar */
	   lng = 2 + (Paralen(param));
   }else{
	   lng = 2 + length;
   }

   parameters[0] = (char)id;          /* Se añade la Id al params para calcular el CheckSum */
   parameters[1] = (char)lng;         /* Se añade la longitud al params para calcular el CheckSum */
   parameters[2] = (char)instruction; /* Se añade la Instrucción al params para calcular el CheckSum */

    for(i=0;i<=(lng-2);i++){       /* Se añaden los parametros que se quieren enviar */
    	parameters[3+i]= (char)param[i];
    }
    lng = lng + 1;   /* Se suma uno para contemplar el parametro del Id en la longitud del cálculo de la trama*/

    Get_Packet(parameters,packet_Out,&lng); /* Se monta la trama y se prepara para el envío, lng actualiza su valor */

    Send_Data(packet_Out,lng);     // Se realiza el envío

    if(id== 0xfe || Status_Return_Value != ALL){
    	      return (0x00);
    	 }else
    	   {
    		 num_bytes = Received_Data(packet_In,BUFFER_SIZE-1); // Se lee lo que haya en el buffer de entrada

    	      if(num_bytes !=0){
    			  return (packet_In[4]);  //Error que envía el dynamixel.
    	       }else
    	        {
    	          return (0x80);          //Error No habido respuesta.
    	        }
    	   }

}

/* Función Action_Servo */
void Action_Servo(void){

	memset(packet_Out,0,DATA_SIZE); /* Resetar array global */

	 packet_Out[0]=0xff;  /* Trama para que los servos actuen a la misma vez */
	 packet_Out[1]=0xff;
	 packet_Out[2]=0xfe;
	 packet_Out[3]=0x02;
	 packet_Out[4]=INST_ACTION;
	 packet_Out[5]=0xfa;

     Send_Data(packet_Out,6);  /* Se realiza el envío de los datos */
}

/* Función SyncWrite_Servo */
void SyncWrite_Servo(uint8_t *id, uint16_t length, uint8_t *param){

	 uint16_t lng=0, i=0,j=0,cnt=0,x=0;
	 uint8_t Num_dyna=0;

	 memset(packet_Out,0,DATA_SIZE);  /* Resetar los arrays globales */
	 memset(parameters,0,DATA_SIZE);

	 Num_dyna = Paralen(id); /* Cálculo numero de servos dinamixel */

	 lng= ((length+1)*Num_dyna)+4;     /* Cálculo de la longitud total de datos a envíar a los servos */

	 parameters[0]= (char) BROADCASTING_ID; /* Id broadcasting o 0xfe */
	 parameters[1]= (char) lng;             /* Longitud total de datos que se enviarán */
	 parameters[2]= (char) INST_SYNC_WRITE; /* Intruccion escritura sincronizada SYNC_WRITE */
	 parameters[3]= (char) param[0];    /* Dirección donde se empieza a escribir (Address) */
	 parameters[4]= (char) length;          /* longitud de los parámetros que se van a escribir */

	 cnt=5;
	 x=1;
	 for(i=0; i<Num_dyna;i++){   /* Se añaden los elementos de los parametros para el cálculo del CheckSum de la trama */
	 	parameters[cnt]= (char)id[i];
	    cnt++;
			for(j=0;j<length;j++){
				parameters[cnt]= (char)param[x];
				x++;
				cnt++;
			}
	  }

	 lng = lng + 1;   /* Se suma uno para contemplar el parametro del Id en la longitud del cálculo de la trama */
	 Get_Packet(parameters,packet_Out,&lng); /* Se monta la trama y se prepara para el envío, lng actualiza su valor */

	 Send_Data(packet_Out,lng); /* Se realiza el envío*/
}

/* Función Read_Servo */
uint8_t Read_Servo(uint8_t id,uint8_t Address,uint16_t lng_reg,uint8_t *value){

	uint16_t lng=4,num_bytes=0,x=0;

	memset(packet_Out,0,DATA_SIZE);  /* Resetar los arrays globales */
	memset(parameters,0,DATA_SIZE);
	memset(packet_In,0,DATA_SIZE);

	 parameters[0]= (char)id;    // Id del servo que se quiere envíar la trama
	 parameters[1]= 0x04;        // Longitud del numero parámetros
	 parameters[2]= INST_READ;
	 parameters[3]= (char)Address;
	 parameters[4]= (char)lng_reg;

    lng=lng+1; // Se suma uno para contemplar el parámetro del Id en la longitud del cálculo de la trama.

    Get_Packet(parameters,packet_Out,&lng); //Se monta la trama y se prepara para el envío, lng actualiza su valor.

	Send_Data(packet_Out,lng);     /* Se realiza el envío */


    num_bytes = Received_Data(packet_In,BUFFER_SIZE-1); /* Se lee lo que haya en el buffer de entrada */

    if(num_bytes !=0){

    	for(x=5;x<(num_bytes-1);x++){ /* Devuelve los valores deseados, en el puntero value */
    	           value[x-5]= packet_In[x];
    	       	  }
    	return (packet_In[4]);  /* Error que envía el dynamixel */
      }else
        {
          return (0x80);          /* Error No habido respuesta */
        }
}

/* FIN_INSTRUCDYNAMIXEL */
