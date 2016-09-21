//============================================================================
// Name        : Bioloid Inverse Kinematics
// Author      : Juan Carlos Brenes. Agosto 2016. UPV, Valencia
// Version     : 1
// Copyright   : Free para todos!
// Description : Control de Robot humanoide Bioloid con servos RX-12A mediante
//				 Beaglebone Black.
//				 Funciones para ubicar la mano según y generar trayectorias.
//				 También funciones para ubicar el pie y la cadera y generar
//				 gaits de caminado.
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
#include "InstrucDynamixel.h"
#include "UtilDynamixel.h"
#include "DynamixelAXDef.h"
#include "BioloidPosDef.h"

/* Puerto de comunicación */
#define           DEVICE_PORT          "/dev/ttyUSB0"
/* Velocidad comunicación, bps. */
//#define           BAUD_RATE             19200
//#define           BAUD_RATE             38400
//#define         BAUD_RATE             57600
#define         BAUD_RATE             115200

#define	IS_RIGHT_LEG	true
#define IS_LEFT_LEG		false


using namespace std;
using namespace exploringBB;


//ID de los servos
const unsigned int idRightBase= 1;
const unsigned int idRightShoulder= 3;
const unsigned int idRightElbow= 5;

const unsigned int idLeftBase= 2;
const unsigned int idLeftShoulder= 4;
const unsigned int idLefttElbow= 6;

const unsigned int idRightHipYaw= 7;
const unsigned int idRightHipRoll= 9;
const unsigned int idRightHipPitch= 11;
const unsigned int idRightKnee= 13;
const unsigned int idRightAnklePitch= 15;
const unsigned int idRightAnkleRoll= 17;

const unsigned int idLeftHipYaw= 8;
const unsigned int idLeftHipRoll= 10;
const unsigned int idLeftHipPitch= 12;
const unsigned int idLeftKnee= 14;
const unsigned int idLeftAnklePitch= 16;
const unsigned int idLeftAnkleRoll= 18;

uint8_t idRight_Foot= 51;
uint8_t idLeft_Foot= 52;

const unsigned int idRightArmServos[3]= {idRightBase, idRightShoulder, idRightElbow};
//unsigned char idRightLegServos[7]= {(char)idRightHipYaw, (char)idRightHipRoll, (char)idRightHipPitch, (char)idRightKnee, (char)idRightAnklePitch, (char)idRightAnkleRoll};
uint8_t idRightLegServos[7] = {7,9,11,13,15,17,'}'};
uint8_t idLeftLegServos[7] = {8,10,12,14,16,18,'}'};
//extern uint8_t Dynamixels[19] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,'}'};

//Dimensiones de la herramienta y el brazo
const float longTool= 0.0255;
float armLenghts[5] = {0.047, 0.0145, 0.025, 0.0675, (0.0745+longTool)};

//Dimensiones de la pierna
float LegLenghts[5] = {0.0385, 0.031, 0.0145, 0.075, 0.0295};

//Límites en grados de los movimientos de los servos de la pierna
const int lowerLimitHipYaw= 55;
const int upperLimitHipYaw= 160;

const int lowerLimitHipRoll= 100;
const int upperLimitHipRoll= 190;

const int lowerLimitHipPitch= 50;
const int upperLimitHipPitch= 175;

const int lowerLimitKneePitch= 46;
const int upperLimitKneePitch= 160;

const int lowerLimitAnklePitch= 120;
const int upperLimitAnklePitch= 210;

const int lowerLimitAnkleRoll= 110;
const int upperLimitAnkleRoll= 180;


//Constantes para el cálculo de trayctorias
const int sample=1;  //cantidad de divisiones para la trayectoria
unsigned int sleepTimeTraj= 100000; //tiempo de delay entre cada división de la trayectoria
const int servoVelFast= 40; //Velocidad de los servos en RPM
const int servoVelSlow= 10; //Velocidad de los servos en RPM


int angDescanso[3]={150,110,240}; //Ángulos para la posición de descanso del brazo


//Puntos en el espacio para las trayectorias
float xf= -0.030;	// forward limit for foot on ground
float xb=  0.030;	// backward limit for foot on ground
float xm=  0.000;	// x position for standing still
float xu= -0.020  ;	// x position for foot up

float yu= 0.030; // y position when up
float ydOut= 0.033; // y poosition when down and coming from up
float ydIn= -0.018; //y position when down and bent

float zu = 0.100; // height of foot when up and down
float zd = 0.140;

float hRoll= 0;  //Angulo roll de la cadera
float hPitch= 20;  //Angulo pitch de la cadera

//Ubicación de inicio y fin para cada trayectoria
float legDownForwardRight [3]= {xf,ydOut,zd};
float legDownBackwardRight [3]= {xb, 0, zd};

float legDownForwardLeft [3]= {xf,-ydOut,zd};
float legDownBackwardLeft [3]= {xb, -ydOut, zd};

float legUpMiddleRight [3]= {xu, yu, zu};
float legUpMiddleLeft [3]= {xu, -yu, zu};

float orientHipRight [3]= {hPitch, hRoll, 0};
float orientHipLeft [3]= {hPitch, -hRoll, 0};
float orientHipZero [3]= {hPitch, 0, 0};

float legDownStanding [3] = {xm, 0, zd};
float legDownMiddleRight [3] = {xm, ydIn, zd};
float legDownMiddleLeft [3] = {xm, -ydOut, zd};

//Función que calcula la cinemática inversa del brazo. Recibe los puntos x, y, z del espacio.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IK3ServoArm (float xh, float yh, float zh, float armLen[5], int ang[3]){

	std::cout << "Info: X= "<<xh << "  Y= "<<yh << "  Z= "<<zh <<endl;

	//Ecuación para encontrar el ángulo del servo del hombro horizontal
	int servoBase = round( atan2(zh,xh)* 180 / M_PI );
	ang[0]=servoBase;

	//Ecuaciones para encontrar el ángulo del servo del codo
	float s = sqrt(pow(xh,2) + pow(zh,2))-armLen[2];
	float t = -yh - armLen[0] - armLen[1];
	float Lra= sqrt(pow(s,2) + pow(t,2));
	float C5=  (pow(Lra,2) - pow(armLen[3],2) - pow(armLen[4],2)) / (2 * armLen[3] * armLen[4]);
	float angleElbowRad = -atan2( sqrt(1-pow(C5,2)), C5 );
	int servoElbow= round(angleElbowRad * 180 / M_PI );
	ang[2]=servoElbow;

	//Ecuaciones para encontrar el ángulo del servo del hombro vertical
	float gamma1= atan2(s,t);
	float gamma2= atan2( (armLen[4]*sin(-angleElbowRad)), (armLen[3]+armLen[4]*cos(-angleElbowRad)));
	int servoShoulder = round( -(gamma1-gamma2) * 180 / M_PI );
	ang[1]=servoShoulder;

}

//Función que recibe 3 ángulos calculados de la cinemática inversa y los ajusta a las especificaciones
//del brazo del robot Bioloid y los servos Dynamixel.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IKAdjust3ServoArm (int ang[3]){

	//Cuando los 2 ángulos son cero, en casi todos los casos corresponde a un punto no alcanzable
	if ((ang[1]==0) || (ang[2]==0)){
			std::cout << "Error: La posición en el espacio no es alcanzable."<<endl;
	}

	//Ajusta el ángulo de la base en base.
	int servoBaseRX10= 152-(90-ang[0]); //150: ajuste del RX, 90-: ajuste con el eje de coordenadas
	if ((servoBaseRX10<0) || (servoBaseRX10>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo de la base."<<endl;
		ang[0]=150;  //Pone la posición por defecto del servo
	}else{
		ang[0]=servoBaseRX10;
	}

	ang[0]=servoBaseRX10;

	//Ajusta el ángulo del hombro
	int servoShoulderRX10= 150-(ang[1]); //150-: ajuste del RX
	//Revisa los valores máximos y minimos del servo
	if ((servoShoulderRX10<0) || (servoShoulderRX10>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo del hombro."<<endl;
		ang[1]=150;  //Pone la posición por defecto del servo
	}else if (servoShoulderRX10<60){
			std::cout << "Error: La posición deseada provoca colosión en el servo del hombro. Servo saturado a su valor mínimo."<<endl;
		ang[1]=60;
	}else if (servoShoulderRX10>240){
		std::cout << "Error: La posición deseada provoca colosión en el servo del hombro. Servo saturado a su valor máximo."<<endl;
		ang[1]=240;
	}else{
		ang[1]=servoShoulderRX10;
	}

	//Ajusta el ángulo del codo
	int servoElbowRX10= 150-(ang[2]); //150-: ajuste del RX
	//Revisa los valores máximos y minimos del servo
	if ((servoElbowRX10<0) || (servoElbowRX10>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo del codo."<<endl;
		ang[2]=150; //Pone la posición por defecto del servo
	}else if (servoElbowRX10<60){
			std::cout << "Error: La posición deseada provoca colisión en el servo del codo. Servo saturado a su valor mínimo."<<endl;
		ang[2]=60;
	}else if (servoElbowRX10>240){
		std::cout << "Error: La posición deseada provoca colosión en el servo del codo. Servo saturado a su valor máximo."<<endl;
		ang[2]=240;
	}else{
		ang[2]=servoElbowRX10;
	}

	std::cout << "Info: Ang Corregidos. Base: "<<ang[0]<< "  Hombro: "<<ang[1]<<"  Codo: "<<ang[2]<<endl;
}

//Función que calcula una trayectoria lineal para la punta del brazo. Recibe el punto de inicio y el punto final, así como las dimensiones del brazo.
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTrajArm (float pos1[3], float pos2[3], float armLen[5], int trajAng[3][sample]){

	//Se ejecuta el proceso para cada instante de tiempo. La cantidad de instantes está dada por la variable Sample
	for (int i=1;i<=sample;i++){
		//Se divide el trayecto entre la cantidad de instantes de tiempo. Obteniendo una serie de puntos intermedios
		//Se recorren uno por uno estos puntos
		float pos_xi= ( (pos2[0]-pos1[0])/sample*i ) + pos1[0] ;
		float pos_yi= ( (pos2[1]-pos1[1])/sample*i ) + pos1[1] ;
		float pos_zi= ( (pos2[2]-pos1[2])/sample*i ) + pos1[2] ;
		//En cada punto del trayecto se calcula la Cinemática Inversa
		int angles[3]={0,0,0};
	    IK3ServoArm (pos_xi, pos_yi, pos_zi, armLen, angles); //Calcula la Cinemática Inversa
	    IKAdjust3ServoArm (angles);		//Corrige los ángulos para ajustarlos a los servos RX
	    //Se almacenan los ángulos para cada punto del trayecto lineal
	    trajAng[0][i]= angles[0];
	    trajAng[1][i]= angles[1];
	    trajAng[2][i]= angles[2];
	}
}

//Función que calcula una trayectoria libre entre 2 puntos. Recibe el punto de inicio y el punto final, así como las dimensiones del brazo.
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void freeTrajArm (float pos1[3], float pos2[3], float armLen[5], int trajAng[3][sample]){

	//Se calcula la Cinemática Inversa para el punto inicial
	int angles1[3]={0,0,0};
	IK3ServoArm (pos1[0], pos1[1], pos1[2], armLen, angles1); //Calcula la Cinemática Inversa
	IKAdjust3ServoArm (angles1);		//Corrige los ángulos para ajustarlos a los servos RX
	//Se calcula la Cinemática Inversa para el punto final
	int angles2[3]={0,0,0};
	IK3ServoArm (pos2[0], pos2[1], pos2[2], armLen, angles2); //Calcula la Cinemática Inversa
	IKAdjust3ServoArm (angles2);			//Corrige los ángulos para ajustarlos a los servos RX

	//Se ejecuta el proceso para cada instante de tiempo. La cantidad de instantes está dada por la variable Sample
	for (int i=1;i<=sample;i++){
		//Se divide el ángulo inicial y el final entre la cantidas de instántes de tiempo. Obteniendo una serie de ángulos intermedios
		int angleBase_i= round( (float)(angles2[0]-angles1[0])/sample*i + angles1[0] ) ;
		int angleShoulder_i= round( (float)(angles2[1]-angles1[1])/sample*i + angles1[1] );
		int angleElbow_i= round( (float)(angles2[2]-angles1[2])/sample*i + angles1[2] );
		//Se almacenan los ángulos para cada instante de tiempo
	    trajAng[0][i]= angleBase_i;
	    trajAng[1][i]= angleShoulder_i;
	    trajAng[2][i]= angleElbow_i;
	}
}

//Función que calcula la cinemática inversa de la pierna del robot, toma como referencia el pie y ubica la cadera
//Recibe los puntos x, y, z de la ubicación deseada de la cadera. También los ángulos pitch, roll, yaw de la orientación deseada.
//Modifica por referencia un vector con el valor de los ángulos en grados. Estos ángulos toman el eje x del servo como cero
void IK6ServoLegtoHip (float xH, float yH, float zH, float pitchH, float rollH, float yawH, float legLen[5], int ang[6]){

	std::cout<<endl << "PosHip: X= "<<xH << "  Y= "<<yH << "  Z= "<<zH <<endl;

	//int L6 = LegLenghts[1];
	float L7 = LegLenghts[2];
	float L8 = LegLenghts[3];

	//Cálculo del ángulo Roll del tobillo para la Posición
	int ankle_roll= round (atan2(yH,zH) * 180/M_PI );

	//Cálculo del ángulo Pitch de la Rodilla para la Posición
	float Lrleg= sqrt( pow(xH,2) + pow(yH,2) + pow(zH,2) ); //extensión de la pierna
	float Lf= sqrt( pow(L7,2) + pow(L8,2) ); //extensión del muslo o la pantorrilla
	float alpha= atan2(L7,L8); //ángulo en que está desviado el servo de la rodilla del punto de flexión

	float cos_gamma= (pow(Lrleg,2) - 2*pow(Lf,2))/(-2*pow(Lf,2)); //ley de cosenos para el ángulo de la rodilla
	float gamma= atan2( sqrt(1-pow(cos_gamma,2)), cos_gamma); //ángulo de la rodilla sin corregir desvío
	int knee_pitch = round( (gamma - 2*alpha) * 180/M_PI); //ángulo del servo (corregido)

	//Cálculo del ángulo Pitch del tobillo para la Posición
	float Lrleg_prime = sqrt(pow(yH,2) + pow(zH,2));  //proyeccción en Z de la extensión de la pierna
	float beta= atan2( xH, Lrleg_prime); //angulo entre proyeccción y extensi'n
	//int ankle_pitch = round( (-beta + (gamma/2)) * 180/M_PI);   //Para Matlab, sin alpha
	int ankle_pitch = round( (-beta + (gamma/2) - alpha) * 180/M_PI); //Real, usa el valor de alpha

	//Cálculo del ángulo Yaw de la cadera para la Orientación
	int hip_yaw = round( yawH );

	//Cálculo del ángulo Roll de la cadera para la Orientación
	int hip_roll = round( (rollH + ankle_roll) );

	//Cálculo del ángulo Pitch de la cadera para la Orientación
	//hip_pitch = pitchPel + beta + gamma/2; //para Matlab
	int hip_pitch = round( (-pitchH*M_PI/180 + beta + gamma/2 - alpha) * 180/M_PI); //Real

	//Los angulos obtenidos se almacenan en el vector de salida
	ang[0]= hip_yaw;
	ang[1]= hip_roll;
	ang[2]= (90-hip_pitch);
	ang[3]= -(180-knee_pitch);
	ang[4]= (90-ankle_pitch);
	ang[5]= ankle_roll;
}

//Función que calcula la cinemática inversa de la pierna del robot, toma como referencia la cadera y ubica el pie
//Recibe los puntos x, y, z de la ubicación deseada del pie. La cadera se orienta a 0 en pitch, roll y yaw.
//Modifica por referencia un vector con el valor de los ángulos en grados. Estos ángulos toman el eje x del servo como cero
void IK6ServoLegtoFoot (float xFi, float yFi, float zFi, float legLen[5], int ang[6]){

	std::cout<<endl << "PosFoot: X= "<<xFi << "  Y= "<<yFi << "  Z= "<<zFi <<endl;

	//Convertimos las coordenadas como si la referencia fuera el del pie a la cadera (al revés) y con
	//eso podemos usar las ecuaciones de cinemática inversa de ese caso. Para este caso la orientación
	//de la cadera se toma como cero.
	float pitchH=0;
	float rollH=0;
	float yawH=0;
	float zF=-zFi;
	float xF=-xFi;
	float yF=-yFi;

	//int L6 = LegLenghts[1];
	float L7 = LegLenghts[2];
	float L8 = LegLenghts[3];

	//Cálculo del ángulo Roll del tobillo para la Posición
	int ankle_roll= round (atan2(yF,zF) * 180/M_PI );

	//Cálculo del ángulo Pitch de la Rodilla para la Posición
	float Lrleg= sqrt( pow(xF,2) + pow(yF,2) + pow(zF,2) ); //extensión de la pierna
	float Lf= sqrt( pow(L7,2) + pow(L8,2) ); //extensión del muslo o la pantorrilla
	float alpha= atan2(L7,L8); //ángulo en que está desviado el servo de la rodilla del punto de flexión

	float cos_gamma= (pow(Lrleg,2) - 2*pow(Lf,2))/(-2*pow(Lf,2)); //ley de cosenos para el ángulo de la rodilla
	float gamma= atan2( sqrt(1-pow(cos_gamma,2)), cos_gamma); //ángulo de la rodilla sin corregir desvío
	int knee_pitch = round( (gamma - 2*alpha) * 180/M_PI); //ángulo del servo (corregido)

	//Cálculo del ángulo Pitch del tobillo para la Posición
	float Lrleg_prime = sqrt(pow(yF,2) + pow(zF,2));  //proyeccción en Z de la extensión de la pierna
	float beta= atan2( xF, Lrleg_prime); //angulo entre proyeccción y extensi'n
	//int ankle_pitch = round( (-beta + (gamma/2)) * 180/M_PI);   //Para Matlab, sin alpha
	int ankle_pitch = round( (-beta + (gamma/2) - alpha) * 180/M_PI); //Real, usa el valor de alpha

	//Cálculo del ángulo Yaw de la cadera para la Orientación
	int hip_yaw = round( yawH );

	//Cálculo del ángulo Roll de la cadera para la Orientación
	int hip_roll = round( rollH + ankle_roll);

	//Cálculo del ángulo Pitch de la cadera para la Orientación
	//hip_pitch = pitchPel + beta + gamma/2; //para Matlab
	int hip_pitch = round( (-pitchH*M_PI/180 + beta + gamma/2 - alpha) * 180/M_PI); //Real

	//Los angulos obtenidos se almacenan en el vector de salida
	ang[0]= hip_yaw;
	ang[1]= hip_roll;
	ang[2]= (90-hip_pitch);
	ang[3]= -(180-knee_pitch);
	ang[4]= (90-ankle_pitch);
	ang[5]= ankle_roll;
}

//Función que recibe 6 ángulos calculados de la cinemática inversa  de una pierna y los ajusta a las
//especificaciones del robot Bioloid y los servos Dynamixel. Diferencia entre pierna derecha o izquierda.
//Modifica por referencia un vector con el valor de los ángulos en grados
void IKAdjust6ServoLeg (int ang[6], const bool isRightLeg){

	//Variable de ajuste entre la pierna izquierda y la derecha, se utiliza en las fórmulas
	int legDir= 1;
	int minLimitHY = lowerLimitHipYaw;
	int maxLimitHY = upperLimitHipYaw;

	int minLimitHP = lowerLimitHipPitch;
	int maxLimitHP = upperLimitHipPitch;

	int minLimitKP = lowerLimitKneePitch;
	int maxLimitKP = upperLimitKneePitch;

	int minLimitAP = lowerLimitAnklePitch;
	int maxLimitAP = upperLimitAnklePitch;

	//Reajusta los límites cuando es la pierna izquierda
	if (not isRightLeg) {
		legDir=-1;
		minLimitHY= 300 - upperLimitHipYaw;
		maxLimitHY= 300 - lowerLimitHipYaw;
		minLimitHP = 300 - upperLimitHipPitch;
		maxLimitHP = 300 - lowerLimitHipPitch;
		minLimitKP = 300 - upperLimitKneePitch;
		maxLimitKP = 300 - lowerLimitKneePitch;
		minLimitAP = 300 - upperLimitAnklePitch;
		maxLimitAP = 300 - lowerLimitAnklePitch;
	}

	//Cuando los 2 ángulos son cero y la rodilla 180, en casi todos los casos corresponde a un punto no alcanzable
	if ((ang[2]==0) && (ang[3]==180) && (ang[4]==0)){
			std::cout << "Error: La posición en el espacio no es alcanzable."<<endl;
	}

	//Ajusta el ángulo del servo Yaw de la cadera.
	int servoHipYaw= 150-legDir*45+ang[0]; //150: ajuste del Servo, 45: ajuste con el eje de coordenadas
	if ((servoHipYaw<0) || (servoHipYaw>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo Hip Yaw."<<endl;
		ang[0]=150-45;  //Pone la posición por defecto del servo
	}else if ( servoHipYaw < minLimitHY ){
		std::cout << "Error: La posición deseada provoca colosión en el servo Hip Yaw. Servo saturado a su valor mínimo."<<endl;
		ang[0]=minLimitHY;
	}else if ( servoHipYaw > maxLimitHY ){
		std::cout << "Error: La posición deseada provoca colosión en el servo Hip Yaw. Servo saturado a su valor máximo."<<endl;
		ang[0]=maxLimitHY;
	}else{
		ang[0]=servoHipYaw;
	}

	//Ajusta el ángulo del servo Roll de la cadera.
	int servoHipRoll= 150-ang[1]; //150: ajuste del Servo
	if ((servoHipRoll<0) || (servoHipRoll>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo Hip Roll."<<endl;
		ang[1]=150;  //Pone la posición por defecto del servo
	}else if (servoHipRoll<lowerLimitHipRoll){
		std::cout << "Error: La posición deseada provoca colosión en el servo Hip Roll. Servo saturado a su valor mínimo."<<endl;
		ang[1]=lowerLimitHipRoll;
	}else if (servoHipRoll>upperLimitHipRoll){
		std::cout << "Error: La posición deseada provoca colosión en el servo Hip Roll. Servo saturado a su valor máximo."<<endl;
		ang[1]=upperLimitHipRoll;
	}else{
		ang[1]=servoHipRoll;
	}

	//Ajusta el ángulo del servo Pitch de la cadera.
	int servoHipPitch= 150-legDir*ang[2]; //150: ajuste del Servo
	if ((servoHipPitch<0) || (servoHipPitch>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo Hip Pitch."<<endl;
		ang[2]=150;  //Pone la posición por defecto del servo
	}else if (servoHipPitch<minLimitHP){
		std::cout << "Error: La posición deseada provoca colosión en el servo Hip Pitch. Servo saturado a su valor mínimo."<<endl;
		ang[2]=minLimitHP;
	}else if (servoHipPitch>maxLimitHP){
		std::cout << "Error: La posición deseada provoca colosión en el servo Hip Pitch. Servo saturado a su valor máximo."<<endl;
		ang[2]=maxLimitHP;
	}else{
		ang[2]=servoHipPitch;
	}

	//Ajusta el ángulo del servo Pitch de la rodilla.
	int servoKneePitch= 150+legDir*ang[3]; //150: ajuste del Servo
	if ((servoKneePitch<0) || (servoKneePitch>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo Knee Pitch."<<endl;
		ang[3]=150;  //Pone la posición por defecto del servo
	}else if (servoKneePitch<minLimitKP){
		std::cout << "Error: La posición deseada provoca colosión en el servo Knee Pitch. Servo saturado a su valor mínimo."<<endl;
		ang[3]=minLimitKP;
	}else if (servoKneePitch>maxLimitKP){
		std::cout << "Error: La posición deseada provoca colosión en el servo Knee Pitch. Servo saturado a su valor máximo."<<endl;
		ang[3]=maxLimitKP;
	}else{
		ang[3]=servoKneePitch;
	}

	//Ajusta el ángulo del servo Pitch del tobillo.
	if (not isRightLeg) { legDir=-1; }
	int servoAnklePitch= 150+legDir*ang[4]; //150: ajuste del Servo
	if ((servoAnklePitch<0) || (servoAnklePitch>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo Ankle Pitch."<<endl;
		ang[4]=150;  //Pone la posición por defecto del servo
	}else if (servoAnklePitch<minLimitAP){
		std::cout << "Error: La posición deseada provoca colosión en el servo Ankle Pitch. Servo saturado a su valor mínimo."<<endl;
		ang[4]=minLimitAP;
	}else if (servoAnklePitch>maxLimitAP){
		std::cout << "Error: La posición deseada provoca colosión en el servo Ankle Pitch. Servo saturado a su valor máximo."<<endl;
		ang[4]=maxLimitAP;
	}else{
		ang[4]=servoAnklePitch;
	}

	//Ajusta el ángulo del servo Roll del tobillo.
	int servoAnkleRoll= 150-ang[5]; //150: ajuste del Servo
	if ((servoAnkleRoll<0) || (servoAnkleRoll>300)){
		std::cout << "Error: La posición deseada no es alcanzable. Error de valor en el servo Ankle Roll."<<endl;
		ang[5]=150;  //Pone la posición por defecto del servo
	}else if (servoAnkleRoll<lowerLimitAnkleRoll){
		std::cout << "Error: La posición deseada provoca colosión en el servo Ankle Roll. Servo saturado a su valor mínimo."<<endl;
		ang[5]=lowerLimitAnkleRoll;
	}else if (servoAnkleRoll>upperLimitAnkleRoll){
		std::cout << "Error: La posición deseada provoca colosión en el servo Ankle Roll. Servo saturado a su valor máximo."<<endl;
		ang[5]=upperLimitAnkleRoll;
	}else{
		ang[5]=servoAnkleRoll;
	}


	std::cout << "Info: Áng corregidos:  Hip Yaw: "<<ang[0]<< "  Hip Roll: "<<ang[1]<<"  Hip Pitch: "<<ang[2]<<endl;
	std::cout << "Info: Áng corregidos:  Knee Pitch: "<<ang[3]<< "  Ankle Pitch: "<<ang[4]<<"  Ankle Roll: "<<ang[5]<<endl;
}


//Función que calcula una trayectoria lineal para el desplazamiento de la pierna según la posición de la cadera.
//Recibe el punto de inicio y el punto final, también la orientación deseada durante la trayectoria
//Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTrajHip (float pos1[3], float pos2[3], float orient[3], int trajAng[][6], const bool isRightLeg, int divisionQty){

	//Se ejecuta el proceso para cada instante de tiempo. La cantidad de instantes está dada por la variable Sample
	for (int i=1;i<=divisionQty;i++){
		//Se divide el trayecto entre la cantidad de instantes de tiempo. Obteniendo una serie de puntos intermedios
		//Se recorren uno por uno estos puntos
		float pos_xi= ( (pos2[0]-pos1[0])/divisionQty*i ) + pos1[0] ;
		float pos_yi= ( (pos2[1]-pos1[1])/divisionQty*i ) + pos1[1] ;
		float pos_zi= ( (pos2[2]-pos1[2])/divisionQty*i ) + pos1[2] ;
		//En cada punto del trayecto se calcula la Cinemática Inversa
		int angles[6]={0,0,0,0,0,0};
		IK6ServoLegtoHip ( pos_xi, pos_yi, pos_zi, orient[0], orient[1], orient[2], LegLenghts, angles); //Calcula la Cinemática Inversa
		IKAdjust6ServoLeg (angles, isRightLeg);	//Corrige los ángulos para ajustarlos a los servos Dynamixel
	    //Se almacenan los ángulos para cada punto del trayecto lineal
	    trajAng[i][0]= angles[0];
	    trajAng[i][1]= angles[1];
	    trajAng[i][2]= angles[2];
	    trajAng[i][3]= angles[3];
	    trajAng[i][4]= angles[4];
	    trajAng[i][5]= angles[5];
	}
}

//Función que hace que los servos de la pierna vayan a su posicion de descanso
void legGoToRest (const bool isRightLeg){

	int hipId=7;
	int dir=1;
	if (not isRightLeg){
		hipId= 8;
		dir=-1;
	}

	for (int i=0; i<6; i++){
		if (i==0){
			set_Pos((hipId+2*i),150-dir*45, DEGREES);
		}else{
		set_Pos((hipId+2*i),150, DEGREES);
		}
		usleep(30000);
	}
}

//Función que convierte el vector de ángulos a la matriz con el tipo que necesitan las funciones de la libreria UtilDynamixel
//Recibe un vector de ints. Regresa por referencia una matriz de uint_8
void Angles2UtilDyn (int trajAng[sample][6], int row,  uint16_t legServoAngles [1][6]){

	for (int i=0; i<6; i++){
		legServoAngles[0][i]= (uint16_t)trajAng[row][i];
	}
}


//Función que crea y ejecuta las trayectorias para dar medio paso hacia adelante
void HalfStepForward(){

	int legServoAnglesRight[6]=  {0, 0, 0, 0, 0, 0};
	int legServoAnglesLeft[6]=  {0, 0, 0, 0, 0, 0};
	uint16_t lSARight[1][6];
	uint16_t lSALeft[1][6];

	int returnAnglesRightStand2Bent[sample][6];
	int returnAnglesRightBent2Back[sample][6];
	int returnAnglesRightDown[sample][6];
	int returnAnglesRightUp1[sample][6];
	int returnAnglesRightUp2[sample][6];
	int returnAnglesRightBack2Stand[sample][6];

	int returnAnglesLeftStand2Bent[sample][6];
	int returnAnglesLeftBent2Stand[sample][6];
	int returnAnglesLeftDown[sample][6];
	int returnAnglesLeftUp1[sample][6];
	int returnAnglesLeftUp2[sample][6];


	//Se crean las trayectorias
	linearTrajHip (legDownStanding, legDownMiddleRight, orientHipRight, returnAnglesRightStand2Bent, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownMiddleRight, legDownMiddleRight, orientHipRight, returnAnglesRightDown, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownMiddleRight, legDownBackwardRight, orientHipRight, returnAnglesRightBent2Back, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownBackwardRight, legDownStanding, orientHipRight, returnAnglesRightBack2Stand, IS_RIGHT_LEG, sample);

	linearTrajHip (legDownStanding, legDownMiddleLeft, orientHipLeft, returnAnglesLeftStand2Bent, IS_LEFT_LEG, sample);
	linearTrajHip (legDownMiddleLeft, legUpMiddleLeft, orientHipLeft, returnAnglesLeftUp1, IS_LEFT_LEG, sample);
	linearTrajHip (legUpMiddleLeft, legDownForwardLeft, orientHipLeft, returnAnglesLeftUp2, IS_LEFT_LEG, sample);
	linearTrajHip (legDownForwardLeft, legDownStanding, orientHipLeft, returnAnglesLeftBent2Stand, IS_LEFT_LEG, sample);



	linearTrajHip (legDownBackwardRight, legUpMiddleRight, orientHipZero, returnAnglesRightUp1, IS_RIGHT_LEG, sample);
	linearTrajHip (legUpMiddleRight, legDownForwardRight, orientHipZero, returnAnglesRightUp2, IS_RIGHT_LEG, sample);

	linearTrajHip (legDownForwardLeft, legDownBackwardLeft, orientHipLeft, returnAnglesLeftDown, IS_LEFT_LEG, sample);



	//Velocidad inicial lenta
	set_MovSpeed(BROADCASTING_ID,servoVelSlow,RPM);
	usleep(20000);
	//Ubica la pierna derecha en la posición de inicio y setea la velocidad de los servos
	//IK6ServoLegtoFoot ( x, y, z, LegLenghts, legServoAngles);
	IK6ServoLegtoHip ( 0, 0, zd, hPitch, 0, 0, LegLenghts, legServoAnglesRight);
	IKAdjust6ServoLeg (legServoAnglesRight, IS_RIGHT_LEG);
	for (int i=0; i<6; i++){
		set_Pos((7+2*i),legServoAnglesRight[i], DEGREES);
		usleep(20000);
	}

	//Ubica la pierna izquierda en la posición de inicio y setea la velocidad de los servos
	IK6ServoLegtoHip ( 0, 0, zd, hPitch, 0, 0, LegLenghts, legServoAnglesLeft);
	IKAdjust6ServoLeg (legServoAnglesLeft, IS_LEFT_LEG);
	for (int i=0; i<6; i++){
		set_Pos((8+2*i),legServoAnglesLeft[i], DEGREES);
		usleep(20000);
	}
	sleep(5);

	//Ciclo de caminado
	int cantidad_Pasos= 3;
	for (int k=0; k<cantidad_Pasos; k++){
/*
		//Paso adelante Pierna izquierda. Paso atrás derecha
		for (int j=1; j<=sample; j++){
			Angles2UtilDyn (returnAnglesRightStand2Bent, j, lSARight);
			set_PosSync(idRightLegServos,lSARight,0,DEGREES);
			Angles2UtilDyn (returnAnglesLeftStand2Bent, j, lSALeft);
			set_PosSync(idLeftLegServos,lSALeft,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		set_MovSpeed(BROADCASTING_ID,servoVelFast,RPM);
		sleep(1);
		//usleep(500000);
		for (int j=1; j<=sample; j++){
			Angles2UtilDyn (returnAnglesRightDown, j, lSARight);
			set_PosSync(idRightLegServos,lSARight,0,DEGREES);
			Angles2UtilDyn (returnAnglesLeftUp1, j, lSALeft);
			set_PosSync(idLeftLegServos,lSALeft,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		for (int j=1; j<=sample; j++){
			Angles2UtilDyn (returnAnglesRightDown, j, lSARight);
			set_PosSync(idRightLegServos,lSARight,0,DEGREES);
			Angles2UtilDyn (returnAnglesLeftUp2, j, lSALeft);
			set_PosSync(idLeftLegServos,lSALeft,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		set_MovSpeed(BROADCASTING_ID,servoVelSlow,RPM);
		sleep(2);
		for (int j=1; j<=sample; j++){
			Angles2UtilDyn (returnAnglesRightBent2Back, j, lSARight);
			set_PosSync(idRightLegServos,lSARight,0,DEGREES);
			Angles2UtilDyn (returnAnglesLeftBent2Stand, j, lSALeft);
			set_PosSync(idLeftLegServos,lSALeft,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		sleep(2);
		for (int j=1; j<=sample; j++){
			Angles2UtilDyn (returnAnglesRightBack2Stand, j, lSARight);
			set_PosSync(idRightLegServos,lSARight,0,DEGREES);
			usleep(sleepTimeTraj);
		}

		//usleep(500000);
		sleep(4);
*/
	}

	legGoToRest(IS_LEFT_LEG);
	usleep(20000);
	legGoToRest(IS_RIGHT_LEG);
	usleep(20000);

}

//Función que inicializa parámetros y posiciones del robot
void RobotReady(){

	//Habilita la función de monitorear el torque máximo
	set_HoldingTorque(BROADCASTING_ID,ON);
	usleep(20000);

	//Define los valores CW y CCW del Margen y de la Pendiente de Compliance
	int marginValue= 1;
	int slopeValue=32;
	for (int i=1; i<=18; i++){
		if (i<=2){
			slopeValue= 128;
		}else if (i>2 && i<15){
			slopeValue=32;
		}else if (i>=15){
			slopeValue= 64;
		}
		set_Compliance_M_S(i, marginValue, marginValue, slopeValue,  slopeValue);
		usleep(20000);
	}

	//Se define una velocidad inicial
	set_MovSpeed(BROADCASTING_ID,40,HEX);
	usleep(20000);

	set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
	usleep(20000);

}

//Función que mueve el robot 2 pasos para adelante. Basada en matrices del año pasado
void WalkForward2015(){

	//Se define la velocidad y se ejecutan los 2 para los primeros 2 movimientos
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(20000);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk0, i, HEX);
		usleep(20000);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk1, i, HEX);
		usleep(20000);
	}

	set_MovSpeed(BROADCASTING_ID,200,HEX);
	usleep(20000);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk3, i, HEX);
		usleep(20000);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk4, i, HEX);
		usleep(20000);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk5, i, HEX);
		usleep(20000);
	}
/*	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk6, i, HEX);
		usleep(20000);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, FWalk7, i, HEX);
		usleep(20000);
	}*/

}


void WalkForwardL(){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_L2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_M_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_M_R2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_L2, i, HEX);
		usleep(delay);
	}
}

void WalkForwardR(){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_R2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_M_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_M_L2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_R2, i, HEX);
		usleep(delay);
	}
}

void WalkShortL (){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_L2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_R2, i, HEX);
		usleep(delay);
	}
}

void WalkSL1 (){
	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_L2, i, HEX);
		usleep(delay);
	}
}
void WalkSL2 (){
	int delay= 30000;
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_R2, i, HEX);
		usleep(delay);
	}
}

void WalkShortR (){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_R2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_L2, i, HEX);
		usleep(delay);
	}
}

void WalkSR1(){
	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_S_R2, i, HEX);
		usleep(delay);
	}
}
void WalkSR2(){
	int delay= 30000;
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, F_E_L2, i, HEX);
		usleep(delay);
	}
}

void BackSL (){
	int delay= 30000;
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_L2, i, HEX);
		usleep(delay);
	}
}
void BackSR (){
	int delay= 30000;
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_R2, i, HEX);
		usleep(delay);
	}
}

void ReverseShortL (){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_S_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_S_L2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_R2, i, HEX);
		usleep(delay);
	}
}

void ReverseShortR (){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_S_R1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_S_R2, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_L1, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, B_E_L2, i, HEX);
		usleep(delay);
	}
}

void StairReady(){

	for (int i=0; i<2; i++){
		set_VelSync(Dynamixels, StairReady_Vel, i, HEX);
		usleep(50000);
		set_PosSync(Dynamixels, StairReady0, i, HEX);
		usleep(500000);
	}

}

void UpStair(){

	for (int i=0; i<5; i++){
		set_VelSync(Dynamixels, Subir0_Vel, i, HEX);
		usleep(50000);
		set_PosSync(Dynamixels, Subir0, i, HEX);
		usleep(1800000);
	}
	//sleep(4);
	for (int i=0; i<7; i++){
		set_VelSync(Dynamixels, Subir1_Vel, i, HEX);
		usleep(50000);
		set_PosSync(Dynamixels, Subir1, i, HEX);
		usleep(1800000);
	}
}

void DownStair(){

	int limit= 7;
	for (int i=0; i<limit; i++){
		set_VelSync(Dynamixels, BajarC0_Vel, i, HEX);
		usleep(50000);
		set_PosSync(Dynamixels, BajarC0, i, HEX);
		usleep(1800000);
	}
	//sleep(2);
	for (int i=0; i<(limit-1); i++){
		set_VelSync(Dynamixels, BajarC1_Vel, i, HEX);
		usleep(50000);
		set_PosSync(Dynamixels, BajarC1, i, HEX);
		usleep(1800000);
	}
}

void LeftTurnMedium (){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroIzq0_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroIzq1_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroIzq2_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroIzq3_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroIzq4_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroIzq5_medio, i, HEX);
		usleep(delay);
	}
}

void RightTurnMedium (){

	int delay= 30000;
	set_MovSpeed(BROADCASTING_ID,175,HEX);
	usleep(delay);
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroDer0_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroDer1_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroDer2_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroDer3_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroDer4_medio, i, HEX);
		usleep(delay);
	}
	for (int i=0; i<7; i++){
		set_PosSync(Dynamixels, GiroDer5_medio, i, HEX);
		usleep(delay);
	}
}


int main() {

	int stepQty= 2000;
	int readDelay= 30000;
	int footDelay= 500000;
//	int CMThreshold= 6000;
	int PSThreshold= 1000;
	int PSThresholdBack= 80;
	int SharpProximityThreshold= 100;
	int SharpInFrontThreshold=120;
	int SharpDifferenceThreshold= 15;
	int SharpDifference= 0;
	bool SharpProximity= false;
	int stairCont= 0;
	int16_t sharpValue_RightFoot, sharpValue_LeftFoot;
	int16_t CM_RightFoot, CM_LeftFoot;
	int16_t PS_LeftFoot1, PS_LeftFoot2, PS_LeftFoot3, PS_LeftFoot4;
	int16_t PS_RightFoot1, PS_RightFoot2, PS_RightFoot3, PS_RightFoot4;
	int16_t historicalSharp[2][stepQty];
	int16_t historicalCM[2][stepQty];
	int16_t historicalPS[2][stepQty];

	cout << "Comunicando con el robot a través de la BBB..." << endl;
	Open_Connection(DEVICE_PORT,BAUD_RATE);
	sleep(1);

	GPIO outGPIO(60);
	GPIO inGPIO(48);
	outGPIO.setDirection(GPIO::OUTPUT);
	inGPIO.setDirection(GPIO::INPUT);
	inGPIO.setActiveHigh();

	RobotReady();
	sleep(4);

	for (int j=0;j<stepQty;j++){
//	while(true){

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

			//Revisión de los valores de los sensores SHARP
			//Revisa si el robot está en cercanias de una grada de subir
			if ((sharpValue_LeftFoot<SharpProximityThreshold) && (sharpValue_RightFoot<SharpProximityThreshold)){
				SharpProximity= true;
			}

			//Revisa si el sensor ya está en distancia de subir, en cuyo caso sube la grada
			if (((sharpValue_LeftFoot>SharpInFrontThreshold)||(sharpValue_RightFoot>SharpInFrontThreshold)) && (SharpProximity||stairCont>0)  && stairCont<3){

				for (int i=0; i<3; i++){
					sharpValue_LeftFoot= Read_Sharp_Foot(idLeft_Foot);
						usleep(readDelay);
					sharpValue_RightFoot= Read_Sharp_Foot(idRight_Foot);
						usleep(readDelay);

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

			//Movimiento de caminar y lectura de los sensores de Presión
			if (j%2==0){
				//Pie izquierdo para adelante y lee sensores
				//WalkForwardL();
				//WalkShortL();
				WalkSL1();
				usleep(footDelay);
				PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
				usleep(readDelay);
				PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
				usleep(readDelay);
				//Si detecta que baja presion entra
				if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold) {
					//Recoge la pierna
					//BackSL();
					//sleep(1);
					//Mueve la otra pierna, lee la presion y recoge la pierna
					//WalkSL1();
					//usleep(footDelay);

					//Lee de nuevo los sensores por is acaso
					PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
					usleep(readDelay);
					PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
					usleep(readDelay);
					//Si la presion también es baja entra
					if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold && stairCont>=3) {
						//Recoge la pierna hacia atrás
						BackSL();
						//Lee los sensores de atrás para ver si esta todo en el aire
						PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
						usleep(readDelay);
						PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
						usleep(readDelay);
						//Si hay presion en los sensores de atrás, si es una grada
//						if (PS_LeftFoot1>PSThresholdBack && PS_LeftFoot2>PSThresholdBack) {
							StairReady();
							sleep(1);
							DownStair();
							//set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
							sleep(1);
//						}
					}
				}else{
					WalkSL2();
					usleep(footDelay);
					PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
					usleep(readDelay);
					PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
					usleep(readDelay);
					if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold && stairCont>=3) {
						PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
						usleep(readDelay);
						PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
						usleep(readDelay);
						if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold) {
							//Lee los sensores de atrás para ver si esta todo en el aire
							PS_RightFoot1= Read_Pressure(idRight_Foot, 1);
							usleep(readDelay);
							PS_RightFoot2= Read_Pressure(idRight_Foot, 2);
							usleep(readDelay);
							//Si hay presion en los sensores de atrás, si es una grada
//							if (PS_RightFoot1>PSThresholdBack && PS_RightFoot2>PSThresholdBack) {
								StairReady();
								sleep(1);
								DownStair();
								//set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
								sleep(1);
//							}
						}
					}
				}
			}else{
				//WalkForwardR();
				//WalkShortR();
				WalkSR1();
				usleep(footDelay);
				PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
				usleep(readDelay);
				PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
				usleep(readDelay);
				if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold && stairCont>=3) {
					//Recoge la pierna
					//BackSR();
					//sleep(1);
					//Mueve la otra pierna, lee la presion y recoge la pierna
					//WalkSR1();
					//usleep(footDelay);
					PS_RightFoot3= Read_Pressure(idRight_Foot, 3);
					usleep(readDelay);
					PS_RightFoot4= Read_Pressure(idRight_Foot, 4);
					usleep(readDelay);
					//Si la presion también es baja, baja la grada (esta de frente y a la orilla), sino gira
					if (PS_RightFoot3<PSThreshold && PS_RightFoot4<PSThreshold) {
						BackSR();
						//Lee los sensores de atrás para ver si esta todo en el aire
						PS_RightFoot1= Read_Pressure(idRight_Foot, 1);
						usleep(readDelay);
						PS_RightFoot2= Read_Pressure(idRight_Foot, 2);
						usleep(readDelay);
						//Si hay presion en los sensores de atrás, si es una grada
//						if (PS_RightFoot1>PSThresholdBack && PS_RightFoot2>PSThresholdBack) {
							StairReady();
							sleep(1);
							DownStair();
							//set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
							sleep(1);
//						}
					}
				}else{
					WalkSR2();
					usleep(footDelay);
					PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
					usleep(readDelay);
					PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
					usleep(readDelay);
					if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold && stairCont>=3) {
						PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
						usleep(readDelay);
						PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
						usleep(readDelay);
						if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold) {
							//Lee los sensores de atrás para ver si esta todo en el aire
							PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
							usleep(readDelay);
							PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
							usleep(readDelay);
							//Si hay presion en los sensores de atrás, si es una grada
//							if (PS_LeftFoot1>PSThresholdBack && PS_LeftFoot2>PSThresholdBack) {
								StairReady();
								sleep(1);
								DownStair();
								//set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
								sleep(1);
//							}
						}
					}

				}
			}

			if (j%4==0){
				LeftTurnMedium();
				usleep(footDelay);
				PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
				usleep(readDelay);
				PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
				usleep(readDelay);
				if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold && stairCont>=3) {
					PS_LeftFoot3= Read_Pressure(idLeft_Foot, 3);
					usleep(readDelay);
					PS_LeftFoot4= Read_Pressure(idLeft_Foot, 4);
					usleep(readDelay);
					if (PS_LeftFoot3<PSThreshold && PS_LeftFoot4<PSThreshold) {
						//Lee los sensores de atrás para ver si esta todo en el aire
						PS_LeftFoot1= Read_Pressure(idLeft_Foot, 1);
						usleep(readDelay);
						PS_LeftFoot2= Read_Pressure(idLeft_Foot, 2);
						usleep(readDelay);
						//Si hay presion en los sensores de atrás, si es una grada
//							if (PS_LeftFoot1>PSThresholdBack && PS_LeftFoot2>PSThresholdBack) {
							StairReady();
							sleep(1);
							DownStair();
							//set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
							sleep(1);
//							}
					}
				}
			}else if ((j%2==0)&& (stairCont>=3)){
				LeftTurnMedium();
				usleep(footDelay);
			}

			historicalSharp[1][j]= sharpValue_LeftFoot;
			historicalSharp[2][j]= sharpValue_RightFoot;
			historicalPS[1][j]= PS_LeftFoot1;
			historicalPS[2][j]= PS_RightFoot1;
		}else{
			stairCont= 0;
			set_PosSync(Dynamixels, Robot_ready_pos, 0, HEX);
			sleep(1);
		}
		outGPIO.setValue(GPIO::LOW);
		usleep(100000);
	}

/*	for (int j=0;j<4;j++){
		//ReverseShortL();
		//sleep(4);

		//ReverseShortR();
		//sleep(6);

		RightTurnMedium();
		sleep(4);
		LeftTurnMedium();
		sleep(4);
	}
*/
/*	StairReady();
	sleep(1);
	UpStair();
	sleep(2);
*/
/*	StairReady();
	sleep(1);
	DownStair();
	sleep(2);
*/


	std::cout<< endl<< "******* Valores de los sensores ******* "<<endl;
	for (int i=0; i<stepQty; i++){
		std::cout<< "Sharp izquierdo: "<<historicalSharp[1][i]<<"   Sharp derecho: "<<historicalSharp[2][i]<<endl;
	}
	std::cout<< endl;
	for (int i=0; i<stepQty; i++){
		//std::cout<< "CM izquierdo: "<<historicalCM[1][i]<<"   CM derecho: "<<historicalCM[2][i]<<endl;
		std::cout<< "PS izquierdo: "<<historicalPS[1][i]<<"   PS derecho: "<<historicalPS[2][i]<<endl;
	}


	Close_Connection();

	return 0;



}
