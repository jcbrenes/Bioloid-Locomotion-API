//============================================================================
// Name        : Bioloid Inverse Kinematics
// Author      : Juan Carlos Brenes. Agosto 2016. UPV, Valencia
// Version     : 1
// Copyright   : Free para todos!
// Description : Control de Robot humanoide Bioloid con servos RX-12A mediante
//				 Beaglebone Black.
//				 Archivo principal con máquina de estados para el robot
//============================================================================



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
#include "BlackDynamixel.h"
#include "DynamixelAXDef.h"
#include "BioloidLocomotion.h"

using namespace std;
using namespace exploringBB;

/* Puerto de comunicación */
#define           DEVICE_PORT          "/dev/ttyUSB0"
/* Velocidad comunicación, bps. */
//#define           BAUD_RATE             19200
//#define           BAUD_RATE             38400
//#define         BAUD_RATE             57600
#define         BAUD_RATE             115200

uint8_t idRight_Foot= 51;
uint8_t idLeft_Foot= 52;

int main() {

	int stepQty= 100;
	int readDelay= 30000;
	int footDelay= 500000;
	int PSThreshold= 1000;
	int SharpProximityThreshold= 100;
	int SharpInFrontThreshold=120;
	int SharpDifferenceThreshold= 15;
	int SharpDifference= 0;
	bool SharpProximity= false;
	int stairCont= 4;
	int16_t sharpValue_RightFoot, sharpValue_LeftFoot;
	int16_t PS_LeftFoot1, PS_LeftFoot2, PS_LeftFoot3, PS_LeftFoot4;
	int16_t PS_RightFoot1, PS_RightFoot2, PS_RightFoot3, PS_RightFoot4;
	int16_t historicalSharp[2][stepQty];
	int16_t historicalPSLeft[4][stepQty];
	int16_t historicalPSRight[4][stepQty];
	int contHistSharp = 0;
	int contHistPSLeft = 0;
	int contHistPSRight = 0;

	cout << "Comunicando con el robot a través de la BBB..." << endl;
	Open_Connection(DEVICE_PORT,BAUD_RATE);
	sleep(1);

	GPIO outGPIO(60);
	GPIO inGPIO(48);
	outGPIO.setDirection(GPIO::OUTPUT);
	inGPIO.setDirection(GPIO::INPUT);
	inGPIO.setActiveHigh();

	//Se ubica el robot en la posición base
	RobotReady();
	sleep(4);


	//Se ejecuta la máquina de estados una cantidad determinada de pasos
	for (int j=0;j<stepQty;j++){

		//Se utiliza un botón en el robot, el programa sólo se ejecuta cuando este botón está presionado
		GPIO::VALUE goProgram = inGPIO.getValue();
		usleep(100000);
		outGPIO.setValue(GPIO::HIGH);
		usleep(10000);

		if (goProgram){

			//Lectura de sensores SHARP
			sharpValue_LeftFoot= Read_Sharp_Foot(idLeft_Foot);
				usleep(readDelay);
			sharpValue_RightFoot= Read_Sharp_Foot(idRight_Foot);
				usleep(readDelay);

			//Datalog histórico de los Sharp
			historicalSharp[1][contHistSharp]= sharpValue_LeftFoot;
			historicalSharp[2][contHistSharp]= sharpValue_RightFoot;
			contHistSharp++;

			//Revisión de los valores de los sensores SHARP
			//Revisa si el robot está en cercanias de una grada de subir
			if ((sharpValue_LeftFoot<SharpProximityThreshold) && (sharpValue_RightFoot<SharpProximityThreshold)){
				SharpProximity= true;
			}

			//Revisa si el sensor ya está en distancia de subir, en cuyo caso sube la grada
			if (((sharpValue_LeftFoot>SharpInFrontThreshold)||(sharpValue_RightFoot>SharpInFrontThreshold)) && (SharpProximity||stairCont>0)  && stairCont<3){

				//Revisa 3 veces si los valores de distancias son mayores en un sensor que en otro
				for (int i=0; i<3; i++){
					sharpValue_LeftFoot= Read_Sharp_Foot(idLeft_Foot);
						usleep(readDelay);
					sharpValue_RightFoot= Read_Sharp_Foot(idRight_Foot);
						usleep(readDelay);

					//Datalog histórico de los Sharp
					historicalSharp[1][contHistSharp]= sharpValue_LeftFoot;
					historicalSharp[2][contHistSharp]= sharpValue_RightFoot;
					contHistSharp++;

					SharpDifference= (int)sharpValue_RightFoot - (int)sharpValue_LeftFoot;

					//Revisa si el robot está torcido, para girar y corregir posición
					if (SharpDifference > SharpDifferenceThreshold){
						RightTurnMedium();
						usleep(200000);
					}
					if (SharpDifference < -SharpDifferenceThreshold){
						LeftTurnMedium();
						usleep(200000);
					}
				}

				//Movimiento de subir grada
				StairReady();
				sleep(1);
				stairCont++;
				UpStair();
				sleep(1);
				SharpProximity= false;
			}

			//******Movimiento de caminar y lectura de los sensores de Presión
			//En valores pares empieza con el pie izquierdo, en valores impares empieza con el pie derecho
			//PASO PAR
			if (j%2==0){
				//Pie izquierdo para adelante y lee sensores
				WalkSL1();
				usleep(footDelay);

				PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
				usleep(readDelay);
				PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
				usleep(readDelay);

				PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
				usleep(readDelay);
				PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
				usleep(readDelay);

				//Datalog histórico
				historicalPSLeft[1][contHistPSLeft]= PS_LeftFoot1;
				historicalPSLeft[2][contHistPSLeft]= PS_LeftFoot2;
				historicalPSLeft[3][contHistPSLeft]= PS_LeftFoot3;
				historicalPSLeft[4][contHistPSLeft]= PS_LeftFoot4;
				contHistPSLeft++;

				//Si detecta que hay baja presion en los sensores de adelante realiza otra lectura
				if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold) {

					//Lee de nuevo los sensores por is acaso
					PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
					usleep(readDelay);
					PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
					usleep(readDelay);

					PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
					usleep(readDelay);
					PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
					usleep(readDelay);

					//Datalog histórico
					historicalPSLeft[1][contHistPSLeft]= PS_LeftFoot1;
					historicalPSLeft[2][contHistPSLeft]= PS_LeftFoot2;
					historicalPSLeft[3][contHistPSLeft]= PS_LeftFoot3;
					historicalPSLeft[4][contHistPSLeft]= PS_LeftFoot4;
					contHistPSLeft++;

					//Si la presion nuevamente es baja recoge la pierna y baja
					if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold && stairCont>=3) {
						//Recoge la pierna hacia atrás
						BackSL();
						//Y baja la grada
						StairReady();
						sleep(1);
						DownStair();
						sleep(1);
					}

				}else{
					//si no hay presión baja en los sensores de presión, solo termina el movimiento de caminar
					WalkSL2();
					usleep(footDelay);

					PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
					usleep(readDelay);
					PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
					usleep(readDelay);

					PS_RightFoot1= Read_Pressure(idRight_Foot, 1);
					usleep(readDelay);
					PS_RightFoot2= Read_Pressure(idRight_Foot, 2);
					usleep(readDelay);

					//Datalog histórico
					historicalPSRight[1][contHistPSRight]= PS_RightFoot1;
					historicalPSRight[2][contHistPSRight]= PS_RightFoot2;
					historicalPSRight[3][contHistPSRight]= PS_RightFoot3;
					historicalPSRight[4][contHistPSRight]= PS_RightFoot4;
					contHistPSRight++;

					//Si al terminar el paso detecta presión baja en los sensores de adelante, mide de nuevo
					if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold && stairCont>=3) {

						PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
						usleep(readDelay);
						PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
						usleep(readDelay);

						PS_RightFoot1= Read_Pressure(idRight_Foot, 1);
						usleep(readDelay);
						PS_RightFoot2= Read_Pressure(idRight_Foot, 2);
						usleep(readDelay);

						//Datalog histórico
						historicalPSRight[1][contHistPSRight]= PS_RightFoot1;
						historicalPSRight[2][contHistPSRight]= PS_RightFoot2;
						historicalPSRight[3][contHistPSRight]= PS_RightFoot3;
						historicalPSRight[4][contHistPSRight]= PS_RightFoot4;
						contHistPSRight++;

						//Si en la segunda medición vuelve a detectar valores bajos de presión, baja la grada
						if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold) {
							StairReady();
							sleep(1);
							DownStair();
							sleep(1);
						}
					}
				}
			//PASO IMPAR
			}else{
				//Da un paso adelante, empezado con el pie derecho
				WalkSR1();
				usleep(footDelay);

				PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
				usleep(readDelay);
				PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
				usleep(readDelay);

				PS_RightFoot1= Read_Pressure(idRight_Foot, 1);
				usleep(readDelay);
				PS_RightFoot2= Read_Pressure(idRight_Foot, 2);
				usleep(readDelay);

				//Datalog histórico
				historicalPSRight[1][contHistPSRight]= PS_RightFoot1;
				historicalPSRight[2][contHistPSRight]= PS_RightFoot2;
				historicalPSRight[3][contHistPSRight]= PS_RightFoot3;
				historicalPSRight[4][contHistPSRight]= PS_RightFoot4;
				contHistPSRight++;

				//Si detecta que hay baja presion en los sensores de adelante realiza otra lectura
				if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold && stairCont>=3) {

					PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
					usleep(readDelay);
					PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
					usleep(readDelay);

					PS_RightFoot1= Read_Pressure(idRight_Foot, 1);
					usleep(readDelay);
					PS_RightFoot2= Read_Pressure(idRight_Foot, 2);
					usleep(readDelay);

					//Datalog histórico
					historicalPSRight[1][contHistPSRight]= PS_RightFoot1;
					historicalPSRight[2][contHistPSRight]= PS_RightFoot2;
					historicalPSRight[3][contHistPSRight]= PS_RightFoot3;
					historicalPSRight[4][contHistPSRight]= PS_RightFoot4;
					contHistPSRight++;

					//Si la presion nuevamente es baja recoge la pierna y baja
					if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold) {
						BackSR();
						StairReady();
						sleep(1);
						DownStair();
						sleep(1);

					}
				}else{
					//Si no hay baja presión, sólo termina el movimiento de dar un paso y mide de nuevo
					WalkSR2();
					usleep(footDelay);

					PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
					usleep(readDelay);
					PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
					usleep(readDelay);

					PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
					usleep(readDelay);
					PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
					usleep(readDelay);

					//Datalog histórico
					historicalPSLeft[1][contHistPSLeft]= PS_LeftFoot1;
					historicalPSLeft[2][contHistPSLeft]= PS_LeftFoot2;
					historicalPSLeft[3][contHistPSLeft]= PS_LeftFoot3;
					historicalPSLeft[4][contHistPSLeft]= PS_LeftFoot4;
					contHistPSLeft++;

					//Si los sensores de presion delanteros están debajo del umbral, ingresa y hace una segunda lectura
					if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold && stairCont>=3) {

						PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
						usleep(readDelay);
						PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
						usleep(readDelay);

						PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
						usleep(readDelay);
						PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
						usleep(readDelay);

						//Datalog histórico
						historicalPSLeft[1][contHistPSLeft]= PS_LeftFoot1;
						historicalPSLeft[2][contHistPSLeft]= PS_LeftFoot2;
						historicalPSLeft[3][contHistPSLeft]= PS_LeftFoot3;
						historicalPSLeft[4][contHistPSLeft]= PS_LeftFoot4;
						contHistPSLeft++;

						//Si la segunda lectura de sensores es igual, procede a bajar la grada
						if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold) {
							StairReady();
							sleep(1);
							DownStair();
							sleep(1);
						}
					}

				}
			}

			//Cada 4 pasos da un giro a la izquierda para corregir el rumbo
			if (j%4==0){
				LeftTurnMedium();
				usleep(footDelay);
				PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
				usleep(readDelay);
				PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
				usleep(readDelay);

				PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
				usleep(readDelay);
				PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
				usleep(readDelay);

				//Datalog histórico
				historicalPSLeft[1][contHistPSLeft]= PS_LeftFoot1;
				historicalPSLeft[2][contHistPSLeft]= PS_LeftFoot2;
				historicalPSLeft[3][contHistPSLeft]= PS_LeftFoot3;
				historicalPSLeft[4][contHistPSLeft]= PS_LeftFoot4;
				contHistPSLeft++;

				//Si los sensores de presion delanteros están debajo del umbral, ingresa y hace una segunda lectura
				if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold && stairCont>=3) {

					PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
					usleep(readDelay);
					PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
					usleep(readDelay);

					PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
					usleep(readDelay);
					PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
					usleep(readDelay);

					//Datalog histórico
					historicalPSLeft[1][contHistPSLeft]= PS_LeftFoot1;
					historicalPSLeft[2][contHistPSLeft]= PS_LeftFoot2;
					historicalPSLeft[3][contHistPSLeft]= PS_LeftFoot3;
					historicalPSLeft[4][contHistPSLeft]= PS_LeftFoot4;
					contHistPSLeft++;

					//Si la segunda lectura de sensores es igual, procede a bajar la grada
					if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold) {
						StairReady();
						sleep(1);
						DownStair();
						sleep(1);
					}
				}
			}

		}else{
			stairCont= 0;
			RobotReady();
			sleep(1);
		}
		outGPIO.setValue(GPIO::LOW);
		usleep(100000);
	}

	Close_Connection();

	return 0;

}
