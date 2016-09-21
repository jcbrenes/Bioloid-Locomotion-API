//============================================================================
// Name        : BlackDynamixel.h
// Author      : David Sisternes Roses
// Version     : 1.0
// Copyright   : Free_Green_Team
// Description : Cabezera para el manejo y manupulación de la comuncación TTL
//               con servomotores Dynamixel y BeagleBone Black Rev C
//============================================================================

#ifndef BLACKDYNAMIXEL_H_
#define BLACKDYNAMIXEL_H_

#include <cstdint>
#include <stdio.h>
#include "serialib.h"
#include "InstrucDynamixel.h"
//#include "DynamixelAXDef.h"

#define TIME_OUT                    10
#define TX_DELAY_TIME				400

/* Declaración de Funciones */

/* Función Open_Connection *//* 
  Abre y inicia la comunicación con el puerto especificado.
  Devuelve true si la conexión se ha realizado correctamente.
*/
bool Open_Connection(const char *Device,const uint32_t Bauds);

/* Función Close_Connection *//* 
   Cierra la comuniación con el puerto especificado.
   No devuelve nada.
*/
void Close_Connection(void);

/* Función Send_Data *//*
   Envía los datos de (uint8_t *packetOut), con la longitud (uint16_t length) indicada.
*/
void Send_Data(uint8_t *packetOut,uint16_t length);

/* Función Received_Data *//* 
   Lee el numero de datos especificado por (uint16_t max_data) y los guarda
   en (uint8_t *destination) en caso de ser validos.
   Devuelve el numero de bytes que se han leído.
*/
uint8_t Received_Data(uint8_t *destination, uint16_t max_data);

#endif /* BLACKDYNAMIXEL_H_ */
