//============================================================================
// Name        : InstrucDynamixel.h
// Author      : David Sisternes Roses
// Version     : 1.0
// Copyright   : Free_Green_Team
// Description : Librería para el manejo y manupulación de instruciones en los
//               servomotores Dynamixel
//============================================================================

#ifndef INSTRUCDYNAMIXEL_H_
#define INSTRUCDYNAMIXEL_H_


#include <cstdint>
#include <stdio.h>
#include <string.h>
#include "BlackDynamixel.h"
#include "DynamixelAXDef.h"

#define BUFFER_SIZE      200 /* Tamaño del Buffer de entrada */
#define DATA_SIZE        200 /* Tamaño del Buffer de salida */

/* Declaración de Funciones */

/* Función Get_Packet *//*
 Se añade al vector (uint8_t *param), la cabecera 0xff 0xff y se calcula el Get_CheckSum.
 uint8_t packetOut[DATA_SIZE], es el vector de salida donde quedarán guardados los datos.
 uint16_t *length es el numero de elementos que contiene char uint8_t *param.
 No retorna nada
 */
void Get_Packet(uint8_t *param,uint8_t packetOut[DATA_SIZE],uint16_t *length);

/* Función Get_Checksum *//*
  Se le pasa un vector uint8_t *param con la cantidad de elemenots (uint16_t length) que contiene
  y devuelve el byte de CheckSum necesario para el correcto envío de datos.
 */
uint8_t Get_Checksum(uint8_t *param,uint16_t length);

/* Función Paralen *//*
   Devuelve el numero de elementos contenidos en (uint8_t *param), cuenta bytes hasta hallar '}'.
*/
uint16_t Paralen(uint8_t *param);

/* Función Ping_Servo *//*
 Se realiza un PING al servo especificado para saber si está conectado y disponible.
 Devuelve 0 si no hay error, en caso contrario devuelve el error que se ha producido.
*/
uint8_t Ping_Servo(uint8_t id);

/* Función Reset_Servo *//*
   Resetea el nº de servo Dynamixel especificado en (uint8_t id), estableciendo los
   parámetros del servo con valores de fábrica.
   Devuelve 0 si no hay error, en caso contrario devuelve el error que se ha producido.
*/
uint8_t Reset_Servo(uint8_t id);

/* Función Write_Servo *//*
   Se envía la trama la cual provocará una escritura en los parámetros de cada servo.
   (uint8_t id) corresponde con el identificador del servo.
   (uint16_t length) corresponde con la longitud de parámetros que se enviarán.
   (uint8_t instruction) corresponde con la instrucción que se quiere realizar sobre el servo.
   (uint8_t *Param) contiene los parámetros que se le quieren enviar al servo.
   Se puede realizar un Write con Sincronización, instrucción (INST_REG_WRITE) y función Action_Servo();
   o realizar un Write directo con la instrucción (INST_WRITE).
   Devuelve 0 si no hay error, en caso contrario devuelve el error que se ha producido.
*/
uint8_t Write_Servo(uint8_t id,uint16_t length,uint8_t instruction,uint8_t *param);

/* Función Action_Servo *//*
  Provoca la ejecución de las instrucciones depositadas en el buffer de los servos (enviadas
  con la instrucción "INST_REG_WRITE") de manera sincronizada.
  No devuelve nada.
*/
void Action_Servo(void);

/* Función SyncWrite_Servo *//*
   Se envía la trama la cual provocará una escritura en los parámetros de cada servo.
   A diferencia de la Write_Servo, permite el envío simultáneo de datos a varios servos
   dynamixel, escribiendo en los mismos registros de todos los servos.
   (uint8_t *id) es el vector que contiene los diferentes "Id's" de los servomotores.
    ADVERTENCIA!!! el vector uint8_t *id de contener el caracter '}' como elemento final.
    por ejemplo: uint8_t id[]={0x01,0x03,'}'};
   (uint16_t length) es el numero de registros "Address" que se quieren escribir en dichos servos.
   (uint8_t *param) es el vector con los parámetros que se quieren cambiar en dichos servos.
   No devuelve nada.
*/
void SyncWrite_Servo(uint8_t *id, uint16_t length, uint8_t *param);

/* Función Read_Servo *//*
   Lee el número de registros (uint16_t lng_reg) del dynamixel (uint8_t id) y los guarda en (uint8_t *value).
   Devuelve 0 si no hay error, en caso contrario devuelve el error que se ha producido.
*/
uint8_t Read_Servo(uint8_t id,uint8_t Address,uint16_t lng_reg,uint8_t *value);

#endif /* INSTRUCDYNAMIXEL_H_ */
