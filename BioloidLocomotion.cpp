/*
 * BioloidLocomotion.cpp
 *
 *  Librería para locomoción de Robot humanoide Bioloid usando BeagleBone Black
 *  Funciones de cinemática inversa, generaciób de trayectoria y movimientos básicos
 *
 *  Creada por Juan Carlos Brenes Torres
 *  Máster en Automática e Informática Industrial
 *  Universidad Politécnica de Valencia
 */

#include "BioloidLocomotion.h"

#include "BioloidPosDef.h"

using namespace std;
using namespace exploringBB;

#define	IS_RIGHT_LEG	true
#define IS_LEFT_LEG		false


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

const unsigned int idRightArmServos[3]= {idRightBase, idRightShoulder, idRightElbow};
uint8_t idRightLegServos[7] = {7,9,11,13,15,17,'}'};
uint8_t idLeftLegServos[7] = {8,10,12,14,16,18,'}'};


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
//const int sample=5;  //cantidad de divisiones para la trayectoria
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

//Función que calcula una trayectoria lineal para el desplazamiento de la pierna según la posición del pie.
//Recibe el punto de inicio y el punto final. Recibe por referencia una matriz donde devuelve los ángulos para cada instante de tiempo.
void linearTrajFoot (float pos1[3], float pos2[3], int trajAng[][6], const bool isRightLeg, int divisionQty){

	//Se ejecuta el proceso para cada instante de tiempo. La cantidad de instantes está dada por la variable Sample
	for (int i=1;i<=divisionQty;i++){
		//Se divide el trayecto entre la cantidad de instantes de tiempo. Obteniendo una serie de puntos intermedios
		//Se recorren uno por uno estos puntos
		float pos_xi= ( (pos2[0]-pos1[0])/divisionQty*i ) + pos1[0] ;
		float pos_yi= ( (pos2[1]-pos1[1])/divisionQty*i ) + pos1[1] ;
		float pos_zi= ( (pos2[2]-pos1[2])/divisionQty*i ) + pos1[2] ;
		//En cada punto del trayecto se calcula la Cinemática Inversa
		int angles[6]={0,0,0,0,0,0};
		IK6ServoLegtoFoot ( pos_xi, pos_yi, pos_zi, LegLenghts, angles); //Calcula la Cinemática Inversa
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

//Función que convierte el vector de ángulos de cada pierna (obtenido del generador de trayectorias) a la matriz
//con el tipo que necesitan las funciones de la libreria UtilDynamixel. Asigna por defecto 150 grados (512 HEX) en
//los ángulos de los brazos.
//Recibe 2 vectores de ints y el número de fila de la matriz de trayectoria.
//Regresa por referencia una matriz de uint_8
void LegAngles2UtilDyn (int trajAngRightLeg[sample][6], int trajAngLeftLeg[sample][6], int row,  uint16_t robotServoAngles [1][18]){

	for (int i=0; i<5; i++){
		robotServoAngles[0][i]= 150;
	}
	for (int i=6; i<18; i=i+2){
		robotServoAngles[0][i]= (uint16_t)trajAngRightLeg[row][i];
	}
	for (int i=7; i<18; i=i+2){
		robotServoAngles[0][i]= (uint16_t)trajAngLeftLeg[row][i];
	}
}

//Función que crea y ejecuta las trayectorias para dar un paso hacia adelante, empezando con el pie izquierdo
void StepForwardL(){

	uint16_t allServoAngles[1][18];

	//Matrices de ángulos que utilizarán para las trayectorias
	int returnAnglesRightStand2Bent[sample][6];
	int returnAnglesRightBent2Back[sample][6];
	int returnAnglesRightDown[sample][6];
	int returnAnglesRightBack2Stand[sample][6];

	int returnAnglesLeftStand2Bent[sample][6];
	int returnAnglesLeftBent2Stand[sample][6];
	int returnAnglesLeftDown[sample][6];
	int returnAnglesLeftUp1[sample][6];
	int returnAnglesLeftUp2[sample][6];

	//Se crean las trayectorias entre los distintos puntos
	linearTrajHip (legDownStanding, legDownMiddleRight, orientHipRight, returnAnglesRightStand2Bent, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownMiddleRight, legDownMiddleRight, orientHipRight, returnAnglesRightDown, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownMiddleRight, legDownBackwardRight, orientHipRight, returnAnglesRightBent2Back, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownBackwardRight, legDownStanding, orientHipRight, returnAnglesRightBack2Stand, IS_RIGHT_LEG, sample);

	linearTrajHip (legDownStanding, legDownMiddleLeft, orientHipLeft, returnAnglesLeftStand2Bent, IS_LEFT_LEG, sample);
	linearTrajHip (legDownMiddleLeft, legUpMiddleLeft, orientHipLeft, returnAnglesLeftUp1, IS_LEFT_LEG, sample);
	linearTrajHip (legUpMiddleLeft, legDownForwardLeft, orientHipLeft, returnAnglesLeftUp2, IS_LEFT_LEG, sample);
	linearTrajHip (legDownForwardLeft, legDownStanding, orientHipLeft, returnAnglesLeftBent2Stand, IS_LEFT_LEG, sample);
	linearTrajHip (legDownStanding, legDownStanding, orientHipLeft, returnAnglesLeftDown, IS_LEFT_LEG, sample);

	//se establece la velocidad inicial de los servomotores
	set_MovSpeed(BROADCASTING_ID,servoVelSlow,RPM);
	usleep(20000);

	//Ciclo de caminado
	int cantidad_Pasos= 1;
	for (int k=0; k<cantidad_Pasos; k++){

		//Primero se inclina el cuerpo a un lado
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightStand2Bent, returnAnglesLeftStand2Bent, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		//Se cambia la velocidad de los servos a rápido
		set_MovSpeed(BROADCASTING_ID,servoVelFast,RPM);
		sleep(1);
		//Se levanta un pie y se mueve hacia adelante
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightDown, returnAnglesLeftUp1, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		//Se baja el pie
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightDown, returnAnglesLeftUp2, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		//Se cambia la velocidad de los servos a lento
		set_MovSpeed(BROADCASTING_ID,servoVelSlow,RPM);
		sleep(2);
		//Se mueve el cuerpo de estar inclinado a estar sobre el pie de adelante
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightBent2Back, returnAnglesLeftBent2Stand, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		sleep(2);
		//Se mueve el pie que estaba atrás al costado del otro pie
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightBack2Stand, returnAnglesLeftDown, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		sleep(1);
	}
}

//Función que crea y ejecuta las trayectorias para dar un paso hacia adelante, empezando con el pie izquierdo
void StepForwardR(){

	uint16_t allServoAngles[1][18];

	//Matrices de ángulos que utilizarán para las trayectorias
	int returnAnglesLeftStand2Bent[sample][6];
	int returnAnglesLeftBent2Back[sample][6];
	int returnAnglesLeftDown[sample][6];
	int returnAnglesLeftBack2Stand[sample][6];

	int returnAnglesRightStand2Bent[sample][6];
	int returnAnglesRightBent2Stand[sample][6];
	int returnAnglesRightDown[sample][6];
	int returnAnglesRightUp1[sample][6];
	int returnAnglesRightUp2[sample][6];

	//Se crean las trayectorias entre los distintos puntos
	linearTrajHip (legDownStanding, legDownMiddleLeft, orientHipLeft, returnAnglesLeftStand2Bent, IS_LEFT_LEG, sample);
	linearTrajHip (legDownMiddleLeft, legDownMiddleLeft, orientHipLeft, returnAnglesLeftDown, IS_LEFT_LEG, sample);
	linearTrajHip (legDownMiddleLeft, legDownBackwardLeft, orientHipLeft, returnAnglesLeftBent2Back, IS_LEFT_LEG, sample);
	linearTrajHip (legDownBackwardLeft, legDownStanding, orientHipLeft, returnAnglesLeftBack2Stand, IS_LEFT_LEG, sample);

	linearTrajHip (legDownStanding, legDownMiddleRight, orientHipRight, returnAnglesRightStand2Bent, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownMiddleRight, legUpMiddleRight, orientHipRight, returnAnglesRightUp1, IS_RIGHT_LEG, sample);
	linearTrajHip (legUpMiddleRight, legDownForwardRight, orientHipRight, returnAnglesRightUp2, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownForwardRight, legDownStanding, orientHipRight, returnAnglesRightBent2Stand, IS_RIGHT_LEG, sample);
	linearTrajHip (legDownStanding, legDownStanding, orientHipRight, returnAnglesRightDown, IS_RIGHT_LEG, sample);

	//se establece la velocidad inicial de los servomotores
	set_MovSpeed(BROADCASTING_ID,servoVelSlow,RPM);
	usleep(20000);

	//Ciclo de caminado
	int cantidad_Pasos= 1;
	for (int k=0; k<cantidad_Pasos; k++){

		//Primero se inclina el cuerpo a un lado
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightStand2Bent, returnAnglesLeftStand2Bent, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		//Se cambia la velocidad de los servos a rápido
		set_MovSpeed(BROADCASTING_ID,servoVelFast,RPM);
		sleep(1);
		//Se levanta un pie y se mueve hacia adelante
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightUp1, returnAnglesLeftDown, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		//Se baja el pie
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightUp2, returnAnglesLeftDown, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		//Se cambia la velocidad de los servos a lento
		set_MovSpeed(BROADCASTING_ID,servoVelSlow,RPM);
		sleep(2);
		//Se mueve el cuerpo de estar inclinado a estar sobre el pie de adelante
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightBent2Stand, returnAnglesLeftBent2Back, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		sleep(2);
		//Se mueve el pie que estaba atrás al costado del otro pie
		for (int j=1; j<=sample; j++){
			LegAngles2UtilDyn (returnAnglesRightDown, returnAnglesLeftBack2Stand, j,  allServoAngles);
			set_PosSync(Dynamixels,allServoAngles,0,DEGREES);
			usleep(sleepTimeTraj);
		}
		sleep(1);
	}
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

//Función para caminar hacia adelante, empezando con el pie izquierdo
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

//Función para caminar hacia adelante, empezando con el pie derecho
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

//Función que da un paso, empezando con la pierna izquierda
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

//Función que da medio paso adelante. empezando con la pierna izquierda
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
//Función complementaria que da el segundo medio paso luego de WalkSL1
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

//Función que da un paso, empezando con la pierna derecha
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

//Función que da medio paso adelante. empezando con la pierna derecha
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
//Función complementaria que da el segundo medio paso luego de WalkSR1
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

//Función que devuelve para atrás la pierna izquierda (estando adelantada)
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

//Función que devuelve para atrás la pierna derecha (estando adelantada)
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

//Función para ubicar al robot en la posición correcta antes de subir/bajar un escalón
void StairReady(){

	for (int i=0; i<2; i++){
		set_VelSync(Dynamixels, StairReady_Vel, i, HEX);
		usleep(50000);
		set_PosSync(Dynamixels, StairReady0, i, HEX);
		usleep(500000);
	}

}

//Función para subir un escalón
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

//Función para bajar un escalón
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

//Función para dar un giro corto a la izquierda
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

//Función para dar un giro corto a la derecha
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


