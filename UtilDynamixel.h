//============================================================================
// Name        : UtilDynamixel.h
// Author      : David Sisternes Roses
// Version     : 1.0
// Copyright   : Free_Green_Team
// Description : Cabezera para el manejo y manupulación de utilidades en los
//               servomotores Dynamixel.
//============================================================================

#ifndef UTILDYNAMIXEL_H_
#define UTILDYNAMIXEL_H_

#include <cstdint>
#include <stdio.h>
#include <string.h>
#include "BlackDynamixel.h"
#include "InstrucDynamixel.h"
#include "DynamixelAXDef.h"

#define NUM_DYNAMIXELS  18
/* Declaración de Funciones */

//============================================================================
// FUNCIONES DE CONFIGURACIÓN O ESCRITURA EN DYNAMIXEL
//============================================================================
/*  Función set_Id *//*
  Establece el Id del dynamixel indicado en (uint8_t id_old), por el Id de (uint8_t id_new).
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
     set_Id(S_Dyn1,S_Dyn2); -> Establece el Identificador del servo de 1 a 2.
     ó
     set_Id(1,4); 			-> Establece el Identificador del servo de 1 a 4.

*/
uint8_t set_Id(uint8_t id_old,uint8_t id_new);

/*  Función set_BauRate *//*
  Establece la velocidad actual de trasmisión del dynamixel indicado en (uint8_t id) a la velocidad
  indicada en (uint32_t BaudRate).
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
     set_BaudRate(1,19200); -> Establece el BaudRate a la velocidad de 19200 en el dinamixel 1.
*/
uint8_t set_BaudRate(uint8_t id,uint32_t BaudRate);

/*  Función set_ReturnDelay *//*
  Establece el tiempo de retardo de la respuesta de los paquetes de los dynamixel.
  En (uint8_t id) se indica el dynamixel sobre el que se envía la configuración.
  En (uint16_t us) se indica el tiempo de retardo en usec. Rango de operación (0 - 508)usec.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_ReturnDelay(1,400); -> Establece un Return Delay de 400us en el dynamixel 1.
*/
uint8_t set_ReturnDelay(uint8_t id,uint16_t us);

/*  Función set_Mode *//*
  Establece el modo funcionamiento del dynamixel indicado en (uint8_t id).
  El primer modo puede ser en rotación continua (WHEEL), sin limites CW y CCW.
  El otro modo es con los limites CW y CCW, en el cual el servo no sobrepasará dichos limites.
  En (uint16_t Angle_Limit_CW) se indica el límite inferior. Rango (0 - 1023) -> (0 - 300)º
  En (uint16_t Angle_Limit_CCW) se indica el límite superior. Rango (0 - 1023) -> (0 - 300)º
  En (uint8_t type) se indica si los datos introducidos en (uint16_t Angle_Limit_CW) y
  (uint16_t Angle_Limit_CCW), son en grados o en hexadecimal.
  Nota:Para configurar el modo de rotación continua poner a O Angle_Limit_CW y Angle_Limit_CCW.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_Mode(2,WHEEL,0,0,DEGRESS); -> Establece el dynamixel 2 con modo rotación continua. Para poner en marcha
    ó                         	  	  el dynamixel ver función Mode_Wheel(); detallada más abajo.
    set_Mode(3,SERVO,0,1023,HEX);  -> Establece el dynamixel 3 en modo Servo, configura el límite CW a 0 (0º)
                                 	  y el límie CCW a 1023 (300º), como los datos que se introducen son hexadecimales,
                                 	  se usa la etiqueta HEX.
*/
uint8_t set_Mode(uint8_t id,uint8_t mode, uint16_t Angle_Limit_CW, uint16_t Angle_Limit_CCW, uint8_t type);

/*  Función set_HLimitTemp *//*
  Establece la temperatura máxima a la que puede trabajar el dynamixel.
  En caso de sobrepasar dicho límite, saltará una alarma y el dynamixel se desactivará.
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint8_t tmp) se indica el límite de temperatura al que saltará la alarma. Rango (0-150)ºC
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_HLimitTemp(5,85) -> Establece el límite de temperatura del dynamixel 5 a 85ºC.
*/
uint8_t set_HLimitTemp(uint8_t id,uint8_t tmp);

/*  Función set_Voltage *//*
  Establece los valores máximo y mínimo de tensión del dynamixel.
  Si no se llega al valor mínimo o se sobrepasa el valor máximo se generará una señal de alarma.
  En (uint8_t Low_Limit_Voltage) se indicará el límite inferior de tensión. Rango (50 - 250)
  En (uint8_T Highest_Limit_Voltage) se indicará el límite superior de tensión. Rango (50 -250)
  Nota: El valor de tensión indicado es 10 veces más del real. Por ejemplo un valor 80 -> 8V
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
     set_LimitVoltage(1,60,190); -> Establece en el dynamixel 1, el nivel mínimo de tensión a 6 V
                                    y el nivel máximo de tensión a 19 V.
*/
uint8_t set_LimitVoltage(uint8_t id,uint8_t Low_Limit_Voltage, uint8_t Highest_Limit_Voltage);

/*  Función set_MaxTorque *//*
  Establece el máximo torque del dynamixel. Si se establece el parámetro a 0, el dynamixel entra
  en modo (FreeRun).
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint16_t Mtorque) se indica el valor de torque. Rango (0 - 1023) -> (0 - 100)%
  En (uint8_t type) se indica el formato de valo que se introduce. Debe ser HEX si los datos que
  se introducen en (uint16_t Mtorque) son en hexadecimal o PERCENT si son datos de 0% a 100% de torque.
  Nota: Para modo FreeRun Mtorque = 0.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_MaxTorque(1,1023,HEX);	  -> Establece el torque del dynamixel 1 a 1023 (100%)
    ó
    set_MaxTorque(1,0, PERCENT);  -> Establece el torque del dynamixel 1 al 0%, modo FREERUN
*/
uint8_t set_MaxTorque(uint8_t id,uint16_t Mtorque,uint8_t type);

/*  Función set_StatusPacket *//*
  Establece el modo de respuesta de los dynamixel ante el envío de paquetes.
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint8_t set) se determina el modo de respuesta de los dynamixel.
   Establecer set = NONE para no recibir respuesta del dynamixel, sea cual sea el paquete que se envíe.
   Establecer set = READ para recibir respuesta Solo de los paquetes con la instrucción READ.
   Establecer set = ALL  para recibir respuesta siempre y cuando no se haga un BROADCAST_ID.
   Return 0 si no hay error, en caso de error devuelve el número de error.
   Ex:
     set_StatusPacket(1,ALL); -> Establece dynamixel 1, para que responda a todos los paquetes sin BROADCAST.
     ó
     set_StatusPacket(1,NONE); -> Establece dynamixel 1, para que no responda a ningún paquete.
*/
uint8_t set_StatusPacket(uint8_t id,uint8_t set);

/*  Función set_AlarmLED *//*
  Establece en que errores parpadeará el led del dynamixel.
  La función sigue la operación lógica "OR", permitiendo configurar varios errores a la vez.
  Por ejemplo: Input_Voltage_Error | Angle_Limit_Error, configura AlarmaLED para que se active
  cuando se produzca un error de voltage o de límite de águlo.
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint8_t errors) se indican los errores en los que saltará la alarmaLED.
  Tabla Errores:
     ->Input_Voltage_Error
     ->Angle_Limit_Error
     ->Overheating_Error
     ->Range_Error
     ->CheckSum_Error
     ->Overload_Error
     ->Instruction_Error
   Return 0 si no hay error, en caso de error devuelve el número de error.
   Ex:
     set_AlarmLED(1,Overheating_Error|Input_Voltage_Error); -> Establece en el dynamixel 1 los errores de Overheating y de Input_Voltage_Error
*/
uint8_t set_AlarmLED(uint8_t id,uint8_t errors);

/*  Función set_AlarmShutdown *//*
  Establece en que errores se desactivará el troque del dynamixel.
  La función sigue la operación lógica "OR", permitiendo configurar varios errores a la vez.
  Por ejemplo: Input_Voltage_Error | Angle_Limit_Error, configura AlarmShutdown para que desactive
  el torque del dynamixel cuando se produzca un error de voltage o de límite de ángulo.
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint8_t errors) se indican los errores en los que se desactivará el dynamixel.
  Tabla Errores:
     ->Input_Voltage_Error
     ->Angle_Limit_Error
     ->Overheating_Error
     ->Range_Error
     ->CheckSum_Error
     ->Overload_Error
     ->Instruction_Error
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_AlarmShutdown(1,Overheating_Error|Input_Voltage_Error); -> Establece en el dynamixel 1 los errores de Overheating y de Input_Voltage_Error
*/
uint8_t set_AlarmShutdown(uint8_t id,uint8_t errors);

/*  Función set_HoldingTorque *//*
  Establece la activación/desactivación del torque del dynamixel.
  En caso de desactivar torque el dynamixel entra en modo FreeRun (zero torque).
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint8_t set) se indica el estado que se quiere configurar en el dynamixel.
   set = ON para activar torque.
   set = OFF para desactivar torque.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_HoldingTorque(1,ON); -> Activa el torque del dynamixel 1.
    ó
    set_HoldingTorque(1,OFF); -> Desactiva el torque del dynamixel 1.
*/
uint8_t set_HoldingTorque(uint8_t id,uint8_t set);

/*  Función set_LED *//*
  Establece la activación/desactivación del led del servomotor.
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint8_t set) se indica el estado que se quiere configurar en el LED del dynamixel.
   set = ON para activar LED.
   set = OFF para desactivar LED.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_LED(3,OFF); -> Apaga el LED del dynamixel 3.
    ó
    set_LED(1,OFF); -> Enciende el LED del dynamixel 1.
*/
uint8_t set_LED(uint8_t id,uint8_t set);

/*  Función set_Compliance_M_S *//*
  Establece.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_Compliance_M_S(1,0,0,32,32); -> Establece en el dynamixel 1 el CW y CCW Margin a 0 y el CW y CCW Slope a 32.
*/
uint8_t set_Compliance_M_S(uint8_t id,uint8_t CW_Margin,uint8_t CCW_Margin,uint8_t CW_Slope, uint8_t CCW_Slope);

/*  Función set_Punch *//*
  Establece la corriente que se suministra al servo durante la operación.
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint16_t set) se indica el valor de corriente que se le suministra. Rango (0 - 1023) - (0-100)%
  En (uint8_t type) se indica el tipo de datos que se va a introducir. HEX para datos hexadecimales,
  PERCENT para datos en %.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_Punch(1,32, HEX); -> Establece el punch del dynamixel 1 a 32 (~3%)
    ó
    set_Punch(1,29, PERCENT); -> Establece el punch del dynamixel 1 a 20 %
*/
uint8_t set_Punch(uint8_t id,uint16_t punch, uint8_t type);

/*  Función set_Lock *//*
  Activa el modo de escritura reducido. Si se activa el bit Lock, solo podrán
  ser escritas las direcciones de la (0x18 a la 0x23), el resto no podrán ser modificadas.
  Nota: Para realizar el desbloqueo se deberá desconectar la alimentación del dynamixel.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_Lock(2); -> Activa el modo escritura reducido en el dynamixel 2.
*/
uint8_t set_Lock(uint8_t id);

/*  Función set_TorqueLimit *//*
  Establece el máximo torque del dynamixel. Si se establece el parámetro a 0, el dynamixel entra
  en modo (FreeRun).
  En (uint8_t id) se indica el dynamixel a configurar.
  En (uint16_t torque_limit) se indica el valor de torque. Rango (0 - 1023) - (0 - 100)%
  En (uint8_t type) se indica el formato de valo que se introduce. Debe ser HEX si los datos que
  se introducen en (uint16_t Mtorque) son en hexadecimal o PERCENT si son datos de 0% a 100% de torque.
  Nota: Para modo FreeRun Mtorque = 0. A diferencia de la función set_MaxTorque() ver más arriba, el valor del
  límite del torque es volátil, con lo que al quitar la alimentación se perderá la configuración de este parámetro.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
     set_TorqueLimit(1,1023,HEX); -> Establece el Torque Limit del dynamixel 1 al 1023 (100%)
     ó
     set_TorqueLimit(1,50,PERCENT); -> Establece el Torque Limit del dynamixel 1 al 50%

*/
uint8_t set_TorqueLimit(uint8_t id,uint16_t torque_limit, uint8_t type);

/*  Función set_Posl *//*
  Pone el dynamixel con Id(uint8_t id) en la posición indicada en (uint16_t pos). Rango (0 - 1023) -> (0º - 300º)
  En (uint8_t type) se indica el tipo de datos que se va a introducir. HEX para datos hexadecimales,
  DEGRESS para datos en grados.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_Pos(1,512,HEX); -> Establece el dynamixel 1 a la posición 512 (150º)
    ó
	set_Pos(1,200,DEGREES); -> Establece el dynamixel 1 a la posición 200º
*/
uint8_t set_Pos(uint8_t id,uint16_t pos, uint8_t type);

/*  Función set_PosVel *//*
  Pone el dynamixel con Id(uint8_t id) en la posición indicada en (uint16_t pos)
  a la velocidad indicadad en (uint16_t vel). Rangos: pos (0 - 1023) -> (0º - 300º)
   	   	   	   	   	   	   	   	   	   	   	   	      vel (0 - 1023) -> (0 - 114) RPM
  En (uint8_t type_pos) se indica el tipo de datos en la posición que se va a introducir. HEX para datos hexadecimales,
  DEGREES para datos en grados.
  En (uint8_t type_vel) se indica el tipo de datos en la velocidad que se va a introducir. HEX para datos hexadecimales,
  RPM para datos en revoluciones por minuto.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_PosVel(1,0,800,HEX,HEX); -> Establece la posición a 0 (0º) del dynamixel 1 con una velocidad de 800 (~88 RPM)
    ó
    set_PosVel(1,210,80,DEGREES,RPM); -> Establece la posición a 210º del dynamixel 1 con una velocidad de 80 RPM
*/
uint8_t set_PosVel(uint8_t id, uint16_t pos, uint16_t vel, uint8_t type_pos, uint8_t type_vel);

/*  Función set_PosVelPreload *//*
  Pone el dynamixel con Id(uint8_t id) en la posición indicada en (uint16_t pos)
  a la velocidad indicadad en (uint16_t vel).
  A diferencia de set_PosVel(), esta función carga los parámetros que se van a configurar
  en el dynamixel, pero no se harán válidos hasta que se ejecute la función Action_Servo();
  Rangos: pos (0 - 1023) -> (0º - 300º)
   	      vel (0 - 1023) -> (0 - 114) RPM
  En (uint8_t type_pos) se indica el tipo de datos en la posición que se va a introducir. HEX para datos hexadecimales,
  DEGREES para datos en grados.
  En (uint8_t type_vel) se indica el tipo de datos en la velocidad que se va a introducir. HEX para datos hexadecimales,
  RPM para datos en revoluciones por minuto.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_PosVelPreload(2,512,600,HEX,HEX); -> Precarga la posición 512 (150º) y velocidad 600 (~66RPM) al dynamixel 2
    sleep(5);                     		  -> espera de 5s para el ejemplo
    Action_Servo();               		  -> función para que se ejecuten los parámetros precargados.
*/
uint8_t set_PosVelPreload(uint8_t id, uint16_t pos, uint16_t vel,uint8_t type_pos, uint8_t type_vel);

/*  Función set_PosSync *//*
  Establece la posición de cada dynamixel contenida en una tabla o matriz.
  (uint8_t *id) debe contener los identificadores de los dynamixel.
  (const uint16_t pos[][NUM_DYNAMIXELS]) debe contener las posiciones de cada servo.
  (uint8_t column) establece que columna de posiciones se va a utilizar.
  En (uint8_t type) se indica el tipo de datos en la posición que se va a introducir. HEX para datos hexadecimales,
  DEGREES para datos en grados.
  Ex:
   set_PosSync(Dynamixels,Robot_ready_pos,0,HEX); -> Establece a cada dynamixel la posición contenida en la tabla Robot_ready_pos con valores en hexadecimal,
                                                 	 dicha tabla se encuentra en BioloidPosDef.h al igual que el vector Dynamixels.
                                                 	 Column indica la columna de la tabla que se quiere cargar, en este caso es la columna 0.
   ó
   set_PosSync(Dynamixels,Robot_ready_pos_deg,0,DEGREES); -> Establece a cada dynamixel la posición contenida en la tabla Robot_ready_pos_deg con valores en grados,
                                                 	 	 	 dicha tabla se encuentra en BioloidPosDef.h al igual que el vector Dynamixels.
                                                 	 	 	 Column indica la columna de la tabla que se quiere cargar, en este caso es la columna 0.
*/
void set_PosSync(uint8_t *id, uint16_t pos[][NUM_DYNAMIXELS],uint8_t column,uint8_t type);

/*  Función set_VelSync *//*
  Establece la velocidad de cada dynamixel contenida en una tabla o matriz.
  (uint8_t *id) debe contener los identificadores de los dynamixel.
  (const uint16_t vel[][NUM_DYNAMIXELS]) debe contener las velocidades de cada servo.
  (uint8_t column) establece que columna de velocidades se va a utilizar.
  En (uint8_t type) se indica el tipo de datos en la velocidad que se va a introducir. HEX para datos hexadecimales,
  RPM para datos en revoluciones por minuto.
  Ex:
   set_VelSync(Dynamixels,Robot_ready_vel_rpm,0,RPM); -> Establece a cada dynamixel la velocidad contenida en la tabla Robot_ready_vel con valores de revoluciones por minuto,
                                                 	 	 dicha tabla se encuentra en BioloidPosDef al igual que el vector Dynamixels.
                                                 	 	 Column indica la columna de la tabla que se quiere cargar, en este caso es la columna 0.
   ó
   set_VelSync(Dynamixels,Robot_ready_vel_hex,0,HEX); -> Establece a cada dynamixel la velocidad contenida en la tabla Robot_ready_vel_hex con valores hexadecimales,
                                                 	 	 dicha tabla se encuentra en BioloidPosDef al igual que el vector Dynamixels.
                                                 	 	 Column indica la columna de la tabla que se quiere cargar, en este caso es la columna 0.
*/
void set_VelSync(uint8_t *id,const uint16_t vel[][NUM_DYNAMIXELS],uint8_t column,uint8_t type);

/*  Función set_MovSpeed *//*
  Establece la velocidad (uint8_t vel) del dynamixel con Id (uint8_t id).
  Rango ( 1 - 1023 ) -> (0.111 - 114 RPM).
  Nota: en caso de establecer (uint8_t vel) = 0, el dynamixel configura su velocidad
  a la máxima que pueda ir (~114 RPM).
  En (uint8_t type) se indica el tipo de datos que se va a introducir. HEX para datos hexadecimales,
  RPM para datos en revoluciones por minuto.
  Return 0 si no hay error, en caso de error devuelve el número de error.
  Ex:
    set_MovSpeed(1,0,HEX); -> Establece la velocidad del dynamixel 1 a 0 , configurando la velociada a la máxima que pueda ir ~114 RPM.
    ó
    set_MovSpeed(1,60,RPM); -> Establece la velocidad del dynamixel 1 a 60 RPM.
*/
uint8_t set_MovSpeed(uint8_t id, uint16_t vel,uint8_t type);

/*  Función mode_Wheel *//*
   Establece en el modo rotación contínua, el sentido de giro (uint8_t rotation) en el dynamixel (uint8_t id), con la velocidad
   especificada en (uint16_t speed).
   En (uint8_t type) se indica el tipo de datos que se va a introducir. HEX para datos hexadecimales,
   RPM para datos en revoluciones por minuto.
   Return 0 si no hay error, en caso de error devuelve el número de error.
   Ex:
    set_Mode(2,WHEEL,0,0,DEGREES); -> Primero se establece el dynamixel 2 en modo rotación continua.

    mode_Wheel(1,LEFT,200,HEX);    -> Establece el sentido de giro a la izquierda del dynamixel 2 y la velocidad a 200 (~22 RPM).

*/
uint8_t mode_Wheel(uint8_t id, uint8_t rotation, uint16_t speed, uint8_t type);

/*  Función mode_WheelPreload *//*
   Establece en el modo rotación contínua, el sentido de giro (uint8_t rotation) en el dynamixel (uint8_t id), con la velocidad
   especificada en (uint16_t speed). A diferencia de la función anterior, está carga los parámetros para la rotación continua y
   no se ejecutarán hasta que se envíe la trama de Action_Servo().
   En (uint8_t type) se indica el tipo de datos que se va a introducir. HEX para datos hexadecimales,
   RPM para datos en revoluciones por minuto.
   Return 0 si no hay error, en caso de error devuelve el número de error.
   Ex:
    set_Mode(2,WHEEL,0,0,DEGREES); -> Primero se establece el dynamixel 2 en modo rotación continua.

    mode_WheelPreload(1,LEFT,200,HEX); -> Establece el sentido de giro a la izquierda del dynamixel 2 y la velocidad a 200 (~22 RPM).

    Action_Servo();                -> Provoca la ejecución de los parámetros que estaban a la espera de Action_Servo.
*/
uint8_t mode_WheelPreload(uint8_t id, uint8_t rotation, uint16_t speed, uint8_t type);

//============================================================================
// FUNCIONES DE LECTURA EN DYNAMIXEL
//============================================================================
/*  Función readPos *//*
   Lee la posición del dynamixel indicado en (uint8_t id) y lo devuelve en formato especificado en (uint8_t type).
   HEX para datos en hexadecimal, DEGREES para datos en grados.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nLa posición del dynamixel 1 es: %u",readPos(1,DEGREES));
    ó
    printf("\nLa posición del dynamixel 1 es: %u",readPos(1,HEX));
*/
int16_t readPos(uint8_t id, uint8_t type);

/*  Función readSpeed *//*
   Lee la velocidad actual que lleva el dynamixel indicado en (uint8_t id) y lo devuelve en formato especificado en (uint8_t type).
   HEX para datos en hexadecimal, RPM para datos en revoluciones por minuto.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nLa velocidad del dynamixel 1 es: %u",readSpeed(1,HEX));
    ó
    printf("\nLa velocidad del dynamixel 1 es: %u",readSpeed(1,RPM));
*/
int16_t readSpeed(uint8_t id, uint8_t type);

/*  Función readMovSpeed *//*
   Lee el parámetro de configuracion de la velocidad del dynamixel indicado en (uint8_t id) y lo devuelve en formato especificado en (uint8_t type).
   HEX para datos en hexadecimal, RPM para datos en revoluciones por minuto.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nParámetro velocidad del dynamixel 1 es: %u",readMovSpeed(1,HEX));
    ó
    printf("\nParámetro velocidad del dynamixel 1 es: %u",readMovSpeed(1,RPM));
*/
int16_t readMovSpeed(uint8_t id, uint8_t type);

/*  Función readTemperature *//*
   Lee la temperatura del dynamixel indicado en (uint8_t id). el valor de la temperatura se devuelve en ºC.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nLa temperatura del dynamixel 1 es: %u",readTemperature(1));
*/
int16_t readTemperature(uint8_t id);

/*  Función readVoltage *//*
   Lee el voltaje del dynamixel indicado en (uint8_t id). El valor será diez veces el valor real, es decir
   el valor 80 equivale a 8 V.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nVoltaje del dynamixel 1 es: %u",readVoltage(1));
*/
int16_t readVoltage(uint8_t id);

/*  Función readAngleLimit *//*
   Lee el límite del ángulo del dynamixel indicado en (uint8_t id) y lo devuelve en el formato indicado en (uint8_t type),
   HEX para datos en hexadecimal, DEGREES para datos en grados.
   En (uint8_t Limit_Select) se especifica que límite se quiere leer, (CW/CCW)
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nLímite CW del dynamixel 1 es: %u",readAngleLimit(1,CW,HEX));
    ó
    printf("\nLímite CCW del dynamixel 1 es: %u",readAngleLimit(1,CCW,DEGREES));
*/
int16_t readAngleLimit(uint8_t id, uint8_t Limit_Select, uint8_t type);

/*  Función readLoad *//*
   Lee la carga que realiza el dynamixel indicado en (uint8_t id) y lo devuelve en el formato indicado en (uint8_t type),
   HEX para datos en hexadecimal, PERCENT para datos en %.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nCarga dynamixel 1 es: %u",readLoad(1,PERCENT));
*/
int16_t readLoad(uint8_t id,uint8_t type);

/*  Función checkRegister *//*
   Lee el Register Instruccion del dynamixel(uint8_t id) y indica si hay o no una instrucción a la espera de Action_Servo().
   Devuelve 1 en caso que haya instrucción esperando y 0 en caso contrario.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    set_PosVelPreload(1,512,0,HEX,HEX);
    printf("\nCheck register dynamixel 1 es: %u",checkRegister(1));
    sleep(1);
    Action_Servo();
    printf("\nCheck register dynamixel 1 es: %u",checkRegister(1));
*/
int16_t checkRegister(uint8_t id);

/*  Función checkMovement *//*
   Lee el parámetro movement del dynamixel(uint8_t id) y indica si hay o no movimiento.
   Devuelve 1 en caso que el servo este moviendose, en caso de dynamixel parado devuelve 0.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
     set_Pos(1,0,DEGREES);
     sleep(2);
     set_Pos(1,300,DEGREES);
    if(checkMovement(1)){
        printf("\nDynamixel 1 en movimiento");
       }else
         {
           printf("\nDynamixel 1 Parado");
         }
*/
int16_t checkMovement(uint8_t id);

/*  Función checkLock *//*
   Lee el parámetro lock del dynamixel(uint8_t id) y indica si se ha activado el modo escritura reducida.
   El modo escritura reducida, límita la escritura en las "Adress" de la 0x18 a la 0x23, las demás quedan bloqueadas.
   Devuelve 1 en caso que el servo este moviendose, en caso de dynamixel parado devuelve 0.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    checkLock(1);
    set_Lock(1);
    checkLock(1);
*/
int16_t checkLock(uint8_t id);

/*  Función ledState *//*
   Lee el parámetros LED del dynamixel(uint8_t id) y devuelve si encuentra encendido o apagado.
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
     ledState(1);
      set_LED(1,ON);
     ledState(1);
      set_LED(1,OFF);
*/
int16_t ledState(uint8_t id);

//============================================================================
// FUNCIONES DE LECTURA PLANTA DEL PIE
//============================================================================
/*  Función Read_Sharp_Foot *//*
   Lee la distancia del sensor Sharp ubicado en la parte delantera de la planta del pie(uint8_t id).
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
     printf("\nSensor Sharp del pie %u: %u mm",Left_Foot,Read_Sharp_Foot(Left_Foot));
*/
int16_t Read_Sharp_Foot(uint8_t id);

/*  Función Read_Pressure *//*
   Lee la presión del sensor MF-01 indicado en(uint8_t sensor) y que se encuentra en la planta del pie (uint8_t id).
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   Ex:
    printf("\nPresión del sensor %u del pie %u: %u N",PRESSURE_SENSOR_1,Right_Foot,Read_Pressure(Right_Foot, PRESSURE_SENSOR_1));
*/
int16_t Read_Pressure(uint8_t id, uint8_t sensor);

/*  Función Read_CM *//*
   Lee el centro de masas(uint8_t axis) del pie indicado en(uint8_t id).
   En caso de error de lectura, devolverá el valor 4096 o 0xfff.
   En (uint8_t axis) se indica de que eje se quiere leer el CM. (Axis_X ó Axis_Y)
   Ex:
    printf("CM_X pie %u: %u",Left_Foot,Read_CM(Left_Foot,Axis_X));
*/
int16_t Read_CM(uint8_t id, uint8_t axis);

/*  Función Read_CM_Axes *//*
   Lee el centro de masas de los ejes X e Y del pie indicado en (uint8_t id) y los pasa por referencia.
   Ex:
*/
void Read_CM_Axes(uint8_t id, uint16_t *axis_x, uint16_t *axis_y);

#endif /* UTILDYNAMIXEL_H_ */
