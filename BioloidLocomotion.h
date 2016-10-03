/*
 * BioloidLocomotion.h
 *
 *  Librería para locomoción de Robot humanoide Bioloid usando BeagleBone Black
 *  Funciones de cinemática inversa, generaciób de trayectoria y movimientos básicos
 *
 *  Creada por Juan Carlos Brenes Torres
 *  Máster en Automática e Informática Industrial
 *  Universidad Politécnica de Valencia
 */

#ifndef BIOLOIDLOCOMOTION
#define BIOLOIDLOCOMOTION

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>
#include <math.h>
#include <iomanip>
#include <bitset>
#include <cstdint>
#include "GPIO.h"

//Librerías de movimiento de servos Dynamixel:
#include "InstrucDynamixel.h"
#include "DynamixelAXDef.h"
#include "UtilDynamixel.h"

#define sample  5

//Función que calcula la cinemática inversa del brazo. Recibe los puntos x, y, z del espacio.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IK3ServoArm (float xh, float yh, float zh, float armLen[5], int ang[3]);

//Función que recibe 3 ángulos calculados de la cinemática inversa y los ajusta a las especificaciones
//del brazo del robot Bioloid y los servos Dynamixel.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IKAdjust3ServoArm (int ang[3]);

//Función que calcula una trayectoria lineal para la punta del brazo. Recibe el punto de inicio y el punto final, así como las dimensiones del brazo.
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTrajArm (float pos1[3], float pos2[3], float armLen[5], int trajAng[3][sample]);

//Función que calcula una trayectoria libre entre 2 puntos. Recibe el punto de inicio y el punto final, así como las dimensiones del brazo.
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void freeTrajArm (float pos1[3], float pos2[3], float armLen[5], int trajAng[3][sample]);

//Función que calcula la cinemática inversa de la pierna del robot, toma como referencia el pie y ubica la cadera
//Recibe los puntos x, y, z de la ubicación deseada de la cadera. También los ángulos pitch, roll, yaw de la orientación deseada.
//Modifica por referencia un vector con el valor de los ángulos en grados. Estos ángulos toman el eje x del servo como cero
void IK6ServoLegtoHip (float xH, float yH, float zH, float pitchH, float rollH, float yawH, float legLen[5], int ang[6]);

//Función que calcula la cinemática inversa de la pierna del robot, toma como referencia la cadera y ubica el pie
//Recibe los puntos x, y, z de la ubicación deseada del pie. La cadera se orienta a 0 en pitch, roll y yaw.
//Modifica por referencia un vector con el valor de los ángulos en grados. Estos ángulos toman el eje x del servo como cero
void IK6ServoLegtoFoot (float xFi, float yFi, float zFi, float legLen[5], int ang[6]);

//Función que recibe 6 ángulos calculados de la cinemática inversa  de una pierna y los ajusta a las
//especificaciones del robot Bioloid y los servos Dynamixel. Diferencia entre pierna derecha o izquierda.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IKAdjust6ServoLeg (int ang[6], const bool isRightLeg);

//Función que calcula una trayectoria lineal para el desplazamiento de la pierna según la posición de la cadera.
//Recibe el punto de inicio y el punto final, también la orientación deseada durante la trayectoria
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTrajHip (float pos1[3], float pos2[3], float orient[3], int trajAng[][6], const bool isRightLeg, int divisionQty);

//Función que calcula una trayectoria lineal para el desplazamiento de la pierna según la posición del pie.
//Recibe el punto de inicio y el punto final. Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTrajFoot (float pos1[3], float pos2[3], int trajAng[][6], const bool isRightLeg, int divisionQty);

//Función que hace que los servos de la pierna vayan a su posicion de descanso
void legGoToRest (const bool isRightLeg);

//Función que convierte el vector de ángulos a la matriz con el tipo que necesitan las funciones de la libreria UtilDynamixel
//Recibe un vector de ints. Regresa por referencia una matriz de uint_8
void Angles2UtilDyn (int trajAng[sample][6], int row,  uint16_t legServoAngles [1][6]);

//Función que convierte el vector de ángulos de cada pierna (obtenido del generador de trayectorias) a la matriz
//con el tipo que necesitan las funciones de la libreria UtilDynamixel. Asigna por defecto 150 grados (512 HEX) en
//los ángulos de los brazos.
//Recibe 2 vectores de ints y el número de fila de la matriz de trayectoria.
//Regresa por referencia una matriz de uint_8
void LegAngles2UtilDyn (int trajAngRightLeg[sample][6], int trajAngLeftLeg[sample][6], int row,  uint16_t robotServoAngles [1][18]);

//Función que crea y ejecuta las trayectorias para dar un paso hacia adelante, empezando con el pie izquierdo
void StepForwardL();

//Función que crea y ejecuta las trayectorias para dar un paso hacia adelante, empezando con el pie izquierdo
void StepForwardR();

//Función que inicializa parámetros y posiciones del robot
void RobotReady();

//Función para caminar hacia adelante, empezando con el pie izquierdo
void WalkForwardL();

//Función para caminar hacia adelante, empezando con el pie derecho
void WalkForwardR();

//Función que da un paso, empezando con la pierna izquierda
void WalkShortL ();

//Función que da medio paso adelante. empezando con la pierna izquierda
void WalkSL1 ();
//Función complementaria que da el segundo medio paso luego de WalkSL1
void WalkSL2 ();

//Función que da un paso, empezando con la pierna derecha
void WalkShortR ();

//Función que da medio paso adelante. empezando con la pierna derecha
void WalkSR1();
//Función complementaria que da el segundo medio paso luego de WalkSR1
void WalkSR2();

//Función que devuelve para atrás la pierna izquierda (estando adelantada)
void BackSL ();

//Función que devuelve para atrás la pierna derecha (estando adelantada)
void BackSR ();

//Función para ubicar al robot en la posición correcta antes de subir/bajar un escalón
void StairReady();

//Función para subir un escalón
void UpStair();

//Función para bajar un escalón
void DownStair();

//Función para dar un giro corto a la izquierda
void LeftTurnMedium ();

//Función para dar un giro corto a la derecha
void RightTurnMedium ();

#endif /* BIOLOIDLOCOMOTION_H_ */

