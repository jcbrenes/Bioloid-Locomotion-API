//============================================================================
// Name        : UtilDynamixel.cpp
// Author      : David Sisternes Roses
// Version     : 1.0
// Copyright   : Free_Green_Team
// Description : Cabezera para el manejo y manupulación de utilidades en los
//               servomotores Dynamixel.
//============================================================================

#include "UtilDynamixel.h"
//#include "InstrucDynamixel.h"
/* Declaración de Variables necesarias */

uint8_t  Parameters[DATA_SIZE];
uint8_t  Read_Value[DATA_SIZE];
extern uint8_t  Status_Return_Value;
//============================================================================
// FUNCIONES DE CONFIGURACIÓN O ESCRITURA EN DYNAMIXEL
//============================================================================
/* Función set_Id */
uint8_t set_Id(uint8_t id_old,uint8_t id_new){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar arrays globales */

	Parameters[0]= P_ID;
	Parameters[1]= (char)id_new;
	Parameters[2]= '}';

	error=Write_Servo(id_old,Automatic,INST_WRITE, Parameters);

    if(error!=0){
    	printf("\nERROR %x!!! Al configurar Id: %u a >> Id: %u <<",error,id_old,id_new);
    }else
      {
    	printf("\nDynamixel con Id: %u configura >> Id a %u << ",id_old,id_new);
      }

	return error;
}

/* Función set_BaudRate */
uint8_t set_BaudRate(uint8_t id,uint32_t BaudRate){

	uint8_t error =0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_BAUD_RATE;

	switch(BaudRate)
	{
	  case 1000000:
		 Parameters[1]=(char)0x01;
		 break;
	  case 2250000:
	 	 Parameters[1]=(char)0xfa;
	 	 break;
	  case 2500000:
	 	 Parameters[1]=(char)0xfb;
	 	 break;
	  case 3000000:
	  	 Parameters[1]=(char)0xfc;
	  	 break;
	  default:
	  	 Parameters[1]=(char)((2000000/BaudRate)-1);
	  	 break;
	}
	Parameters[2]= '}';
	error = Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
	    printf("\nERROR %x!!! Al configurar >> BaudRate a %u Baudios <<",error,BaudRate);
	 }else
	   {
	   	printf("\nDynamixel con Id: %u configura >> BaudRate a %u Baudios << ",id,BaudRate);
	   }
	return error;
}

/* Función set_ReturnDelay */
uint8_t set_ReturnDelay(uint8_t id,uint16_t us){

	uint8_t error = 0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

    Parameters[0]= P_RETURN_DELAY_TIME;
	Parameters[1]= (char)(us/2);
	Parameters[2]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
		printf("\nERROR %x!!! Al configurar >> Return Delay Time a %u usec <<",error,us);
    }else
      {
		printf("\nDynamixel con Id: %u configura >> Return Delay Time a %u usec <<",id,us);
      }
	return error;
}

/* Función set_Mode */
uint8_t set_Mode(uint8_t id, uint8_t mode, uint16_t Angle_Limit_CW, uint16_t Angle_Limit_CCW,uint8_t type){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	if(type == DEGREES){
		Angle_Limit_CW = (uint16_t)(Angle_Limit_CW/0.2932);
		Angle_Limit_CCW = (uint16_t)(Angle_Limit_CCW/0.2932);
	}

	Parameters[0]= P_CW_ANGLE_LIMIT_L;

	if(mode == WHEEL || (Angle_Limit_CW==0 && Angle_Limit_CCW ==0)){
		Parameters[1]= (char)0x00;
	    Parameters[2]= (char)0x00;
	    Parameters[3]= (char)0x00;
	    Parameters[4]= (char)0x00;


	}else{
		Parameters[1]= (char)(0xff & Angle_Limit_CW);
		Parameters[2]= (char)((Angle_Limit_CW>>8) & 0xff);
		Parameters[3]= (char)(0xff & Angle_Limit_CCW);
		Parameters[4]= (char)((Angle_Limit_CCW>>8) & 0xff);
	 }
	Parameters[5]= '}';

	error = Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){

		if(mode == WHEEL || (Angle_Limit_CW==0 && Angle_Limit_CCW ==0)){
			    printf("\nERROR %x!!! Al configurar el modo rotación continua",error);
		 }else{
				printf("\nERROR %x!!! Al configurar el >> modo servo con límites: CW a %u y CCW a %u <<",error,(uint16_t)(Angle_Limit_CW*0.2933),(uint16_t)(Angle_Limit_CCW*0.2933));
		   }
	  }else
	    {
		  if(mode == WHEEL || (Angle_Limit_CW==0 && Angle_Limit_CCW ==0)){
			 printf("\nDynamixel %u en modo rotación continua (WHEEL)",id);
		  }else{
			 printf("\nDynamixel %u configura modo servo >> límite CW a: %u y límite CCW a %u grados <<",id,(uint16_t)(Angle_Limit_CW*0.2933),(uint16_t)(Angle_Limit_CCW*0.2933));
		    }
	    }
  return error;
}

/* Función set_HLimiTemp */
uint8_t set_HLimitTemp(uint8_t id,uint8_t tmp){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	if(tmp < 0){
		  tmp=0;
	}
	if(tmp>150){
		  tmp=150;
	}
     Parameters[0]= P_LIMIT_TEMEPERATURE;
	 Parameters[1]= (char)tmp;
	 Parameters[2]= '}';

	 error = Write_Servo(id,Automatic,INST_WRITE, Parameters);

	 if(error!=0){
	 		printf("\nERROR %x!!! Al configurar >> Highest Limit Temperature a %u ºC <<",error,tmp);
	  }else
	    {
		  printf("\nDynamixel con Id: %u configura >> Highest Limit Temperature a %u ºC <<",id,tmp);
	    }
	 return error;
}
/* Función set_LimitVoltage */
uint8_t set_LimitVoltage(uint8_t id,uint8_t Low_Limit_Voltage, uint8_t Highest_Limit_Voltage){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	if(Low_Limit_Voltage < 0){
		Low_Limit_Voltage=50;
	 }
	if(Low_Limit_Voltage>250){
		Low_Limit_Voltage=250;
	 }
	if(Highest_Limit_Voltage < 0){
		Highest_Limit_Voltage=50;
	 }
	if(Highest_Limit_Voltage>250){
		Highest_Limit_Voltage=250;
	 }

	Parameters[0]= P_DOWN_LIMIT_VOLTAGE;
	Parameters[1]= (char)(0xff & Low_Limit_Voltage);
	Parameters[2]= (char)(0xff & Highest_Limit_Voltage);
	Parameters[3]= '}';

	error = Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
	  printf("\nERROR %x!!! Al configurar >> Límites de Tensión a (Min: %u V -- Max: %u V) <<",error,(Low_Limit_Voltage/10),(Highest_Limit_Voltage/10));
     }else
	   {
    	printf("\nDynamixel %u configura  >> Límites de Tensión a (Min: %u V -- Max: %u V) <<",id,(Low_Limit_Voltage/10),(Highest_Limit_Voltage/10));
	   }
	return error;
}

/* set_MaxTorque */
uint8_t set_MaxTorque(uint8_t id,uint16_t Mtorque,uint8_t type){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	if(type == PERCENT){
		Mtorque = (uint16_t)(Mtorque*10.23);
	}

	Parameters[0]= P_MAX_TORQUE_L;
	Parameters[1]= (char)(0xff & Mtorque);
	Parameters[2]= (char)((Mtorque>>8) & 0xff);
	Parameters[3]= '}';

	error = Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
		if(Mtorque ==0){
		  printf("\nERROR %x!!! al configurar >> Max Torque a %u %c (FREE RUN MODE) <<",error,(uint16_t)(Mtorque/10.23),37);
		 }else{
		  printf("\nERROR %x!!! al configurar >> Max Torque a %u %c <<",error,(uint16_t)(Mtorque/10.23),37);
		 }
	  }else
	    {
		  if(Mtorque ==0){
		  		printf("\nDynamixel con Id: %u configura >> Max Torque a %u %c (FREE RUN MODE) <<",id,(uint16_t)(Mtorque/10.23),37);
		  	}else{
		  		printf("\nDynamixel con Id: %u configura >> Max Torque a %u %c <<",id,(uint16_t)(Mtorque/10.23),37);
		  	}
	    }
    return error;
}

/* Función set_StatusPacket */
uint8_t set_StatusPacket(uint8_t id,uint8_t set){

	uint8_t error =0;
    memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_RETURN_LEVEL;
	Parameters[1]= (char)set;
	Parameters[2]= '}';

	Status_Return_Value = set;
	error = Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
		  printf("\nERROR %x!!! Al configurar >> Status Return Level a %u <<",error,set);
	     }else
		   {
	    	 printf("\nDynamixel con Id: %u configurar >> Status Return Level a %u <<",id,set);
		   }
	return error;
}

/* set_AlarmLED */
uint8_t set_AlarmLED(uint8_t id,uint8_t errors){
	  uint8_t error =0;
	  memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	  Parameters[0]= P_ALARM_LED;
	  Parameters[1]= (char)errors;
	  Parameters[2]= '}';

	  error=Write_Servo(id,Automatic,INST_WRITE, Parameters);

	  if(error!=0){
	  	  printf("\nERROR %x!!! Al configurar >> AlarmLED a %u <<",error,errors);
	   }else
	  	 {
	  	  printf("\nDynamixel con Id: %u configura >> AlarmLED a %u <<",id,errors);
	  	 }
	 return error;
}

/* set_AlarmShutdown */
uint8_t set_AlarmShutdown(uint8_t id,uint8_t errors){
	uint8_t error =0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_ALARM_SHUTDOWN;
	Parameters[1]= (char)errors;
	Parameters[2]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
	   printf("\nERROR %x!!! Al configurar >> Alarm Shutdown a %u <<",error,errors);
	 }else
	  {
	   printf("\nDynamixel con Id: %u configura >> Alarm Shutdown a %u <<",id,errors);
	  }
	 return error;
}

/* Función set_HoldingTorque */
uint8_t set_HoldingTorque(uint8_t id,uint8_t set){
	uint8_t error =0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_TORQUE_ENABLE;
	Parameters[1]= (char)set;
	Parameters[2]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! Al configurar >> Torque Enable a %u <<",error,set);
	 }else
	  {
		 printf("\nDynamixel con Id: %u configura >> Torque Enable a %u <<",id,set);
	  }
    return error;
}

/* Función set_LED */
uint8_t set_LED(uint8_t id,uint8_t set){
	uint8_t error =0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_LED;
	Parameters[1]= (char)set;
	Parameters[2]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! Al configurar >> LED a %u <<",error,set);
	 }else
	  {
		 printf("\nDynamixel con Id: %u configura >> LED a %u <<",id,set);
	  }
    return error;
}

/* set_Compliance_M_S */
uint8_t set_Compliance_M_S(uint8_t id,uint8_t CW_Margin,uint8_t CCW_Margin,uint8_t CW_Slope, uint8_t CCW_Slope){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_CW_COMPLIANCE_MARGIN;
	Parameters[1]= (char)CW_Margin;
	Parameters[2]= (char)CCW_Margin;
	Parameters[3]= (char)CW_Slope;
	Parameters[4]= (char)CCW_Slope;
	Parameters[5]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
		printf("\nERROR %x!!! Al configurar >> Compliance Margin y Slope <<",error);
     }else
	   {
		 printf("\nDynamixel con Id: %u configura >> Compliance Margin y Slope << ",id);
		 printf("\n>> Compliance Margin (CW: %u y CCW: %u) // Compliance Slope (CW: %u y CCW: %u) << ",CW_Margin,CCW_Margin,CW_Slope,CCW_Slope);
	   }
	return error;
}

/* Función set_Punch */
uint8_t set_Punch(uint8_t id,uint16_t punch, uint8_t type){
   uint8_t error=0;
   memset(Parameters,0,DATA_SIZE); /* Resetar array global */

   if(type == PERCENT){
	   punch = (uint16_t)(punch*10.23);
   }
	Parameters[0]= P_PUNCH_L;
	Parameters[1]= (char)(0xff & punch);
	Parameters[2]= (char)((punch>>8) & 0xff);
	Parameters[3]= '}';

	error = Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! Al configurar >> Punch a %u %c <<",error,(uint16_t)(punch/10.23),37);
     }else
	   {
    	 printf("\nDynamixel con Id: %u configura >> Punch a %u %c <<",id,(uint16_t)(punch/10.23),37);
	   }
	return error;
}

/* Función set_Lock */
uint8_t set_Lock(uint8_t id){
    uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	Parameters[0]= P_LOCK;
	Parameters[1]= (char)0x01;
	Parameters[2]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! Al configurar modo escritura reducido (Lock)",error);
     }else
	   {
    		printf("\nDynamixel con Id: %u activa modo escritura reducido (Lock)",id);
	   }
	return error;
}

/* Función set_TorqueLimit */
uint8_t set_TorqueLimit(uint8_t id,uint16_t torque_limit,uint8_t type){

    uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	if(type == PERCENT){
	  torque_limit = (uint16_t)(torque_limit*10.23);
	 }

	 Parameters[0]= P_TORQUE_LIMIT_L;
	 Parameters[1]= (char)(0xff & torque_limit);
	 Parameters[2]= (char)((torque_limit>>8) & 0xff);
	 Parameters[3]= '}';

	 error= Write_Servo(id,Automatic,INST_WRITE, Parameters);

	 if(error!=0){
		if(torque_limit ==0){
		  printf("\nERROR %x!!! al configurar >> Torque Limit a %u %c (FREE RUN MODE) <<",error,(uint16_t)(torque_limit/10.23),37);
		 }else{
		  printf("\nERROR %x!!! al configurar >> Torque Limit a %u %c <<",error,(uint16_t)(torque_limit/10.23),37);
		 }
	 }else
	  {
		if(torque_limit ==0){
			printf("\nDynamixel con Id: %u configura >> Torque Limit %u %c (FREE RUN MODE) <<",id,(uint16_t)(torque_limit/10.23),37);
	 	}else{
			printf("\nDynamixel con Id: %u configura >> Torque Limit a %u %c <<",id,(uint16_t)(torque_limit/10.23),37);
	 	}
	  }
	return error;
}

/* Función set_Pos */
uint8_t set_Pos(uint8_t id,uint16_t pos,  uint8_t type){
    uint8_t error=0;
    memset(Parameters,0,DATA_SIZE); /* Resetar array global */

    if(type == DEGREES){
    	pos = (uint16_t)(pos/0.2932);
    }

	Parameters[0]= P_GOAL_POSITION_L;
	Parameters[1]= (char)(0xff & pos);
	Parameters[2]= (char)((pos>>8) & 0xff);
	Parameters[3]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! En posición %u º",error,(uint16_t)(pos*0.2932));
     }else
	   {
    	 printf("\nDynamixel %u en posición %u º",id,(uint16_t)(pos*0.2932));
	   }
	return error;
}

/* Función set_PosVel */
uint8_t set_PosVel(uint8_t id, uint16_t pos, uint16_t vel, uint8_t type_pos, uint8_t type_vel){
	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

    if(type_pos == DEGREES){
    	pos = (uint16_t)(pos/0.2932);
    }
    if(type_vel == RPM){
    	vel = (uint16_t)(vel/0.1114);
    }
	Parameters[0]= P_GOAL_POSITION_L;
	Parameters[1]= (char)(0xff & pos);
	Parameters[2]= (char)((pos>>8) & 0xff);
	Parameters[3]= (char)(0xff & vel);
	Parameters[4]= (char)((vel>>8) & 0xff);
	Parameters[5]= '}';

	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! En posición o/y velocidad",error);
     }else
	   {
    	 if(vel==0){
    	 	 printf("\nDynamixel %u en posición: %u a velocidad a máxima posible ~114 RPM",id,(uint16_t)(pos*0.2932));
    	 }else{
    		 printf("\nDynamixel con Id: %u en Posición: %u º y a Velocidad: %u RPM << ",id,(uint16_t)(pos*0.2932),(uint16_t)(vel*0.1114));
    	   }

	   }
	return error;
}

/* Función set_PosVelPreload */
uint8_t set_PosVelPreload(uint8_t id, uint16_t pos, uint16_t vel,uint8_t type_pos, uint8_t type_vel){

	uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */
    if(type_pos == DEGREES){
    	pos = (uint16_t)(pos/0.2932);
    }
    if(type_vel == RPM){
    	vel = (uint16_t)(vel/0.1114);
    }
	Parameters[0]= P_GOAL_POSITION_L;
	Parameters[1]= (char)(0xff & pos);
	Parameters[2]= (char)((pos>>8) & 0xff);
	Parameters[3]= (char)(0xff & vel);
	Parameters[4]= (char)((vel>>8) & 0xff);
	Parameters[5]= '}';

	error=Write_Servo(id,Automatic,INST_REG_WRITE, Parameters);
	if(error!=0){
		printf("\nERROR %x!!! En posición o/y velocidad",error);
     }else
	   {
    	 printf("\nDynamixel con Id: %u Carga parámetros >> Posición: %u º y Velocidad: %u RPM<< ",id,(uint16_t)(pos*0.2932),(uint16_t)(vel*0.1114));
    	 printf("\n>>>Dynamixel a la espera de Action_Servo()");
	   }
	return error;
}

/* Función set_PosSync */
void set_PosSync(uint8_t *id, uint16_t pos[][NUM_DYNAMIXELS],uint8_t column, uint8_t type){

	  uint8_t i=0, num_dyna;
      uint16_t cnt=0;
      memset(Parameters,0,DATA_SIZE); /* Resetar array global */
      //printf("\n***Set_PosSync 1 ******");
       num_dyna= Paralen(id);
       Parameters[0]=P_GOAL_POSITION_L;
	   cnt++;
	   //printf("\n***Set_PosSync 2 ******");
	   printf("\n Cantidad de ids: %d", num_dyna);
	   //printf("\n Otra cantidad de ids: %d", sizeof(id));
	   //printf("\n Id #1: %d", id[0]);
	   if(type ==DEGREES){
		   for(i=0;i<num_dyna; i++){
			   //printf("\n***Set_PosSync 3_1,   i: %d  ****", i);
			   //printf("\n***Set_PosSync 3_1, col: %d  ****", column);
		   	     Parameters[cnt]=(char)(0xff & ((uint16_t)(pos[column][i]/0.293)));
		   	     //printf("\n***Set_PosSync 3_2 , pos: %d  ***", pos[column][i]);
		   	     cnt++;
		   	     Parameters[cnt]= (char)(0xff & (((uint16_t)(pos[column][i]/0.293))>>8));
		   	     //printf("\n***Set_PosSync 3_3 , pos: %d  ****", pos[column][i]);
		   	     cnt++;
		   	 }
	   }else
	     {
		   for(i=0;i<num_dyna; i++){

		   	     Parameters[cnt]=(char)(0xff & pos[column][i]);
		   	     cnt++;
		   	     Parameters[cnt]= (char)(0xff & (pos[column][i]>>8));
		   	     cnt++;
		   	 }
	     }
	  //printf("\n***Set_PosSync 5 ******");
	  SyncWrite_Servo(id,2,Parameters);
	  printf("\nDynamixels en posiciones indicadas");
}

/* Función set_VelSync */
void set_VelSync(uint8_t *id,const uint16_t vel[][NUM_DYNAMIXELS],uint8_t column,uint8_t type){

	uint8_t i=0, num_dyna;
    uint16_t cnt=0;
    memset(Parameters,0,DATA_SIZE); /* Resetar array global */

     num_dyna= Paralen(id);
     Parameters[0]=P_GOAL_SPEED_L;
	   cnt++;

	   printf("\n %d", num_dyna);
	   if(type ==RPM){
		   for(i=0;i<num_dyna; i++){

		   	     Parameters[cnt]=(char)(0xff & ((uint16_t)(vel[column][i]/0.1114)));
		   	     cnt++;
		   	     Parameters[cnt]= (char)(0xff & (((uint16_t)(vel[column][i]/0.1114))>>8));
		   	     cnt++;
		   	 }
	   }else
	     {
		   for(i=0;i<num_dyna; i++){

		   	     Parameters[cnt]=(char)(0xff & vel[column][i]);
		   	     cnt++;
		   	     Parameters[cnt]= (char)(0xff & (vel[column][i]>>8));
		   	     cnt++;
		   	 }
	     }

	  SyncWrite_Servo(id,2,Parameters);
	  printf("\nDynamixels con velocidades indicadas");
}
/* Función set_MovSpeed */
uint8_t set_MovSpeed(uint8_t id, uint16_t vel,uint8_t type){
    uint8_t error=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

    if(type == RPM){
    	vel = (uint16_t)(vel/0.1114);
    }

	 Parameters[0]= P_GOAL_SPEED_L;
	 Parameters[1]= (char)(0xff & vel);
	 Parameters[2]= (char)((vel>>8) & 0xff);
	 Parameters[3]= '}';

	 error=Write_Servo(id,Automatic,INST_WRITE, Parameters);
	if(error!=0){
			printf("\nERROR %x!!! En velocida %u RPM",error,(uint16_t)(vel*0.1114));
	 }else
	  {
		 if(vel==0){
			printf("\nDynamixel %u configura velocidad a máximo RPM possible ~114 RPM",id);
		 }else{
		    printf("\nDynamixel %u >> Velocidad a %u RPM <<",id,(uint16_t)(vel*0.1114));
		 }
	  }
	return error;
}

/* Función mode_Wheel */
uint8_t mode_Wheel(uint8_t id, uint8_t rotation, uint16_t speed, uint8_t type){
    uint8_t error=0;
    uint16_t spd=0;
	memset(Parameters,0,DATA_SIZE); /* Resetar array global */

	spd=speed;
    if(type == RPM){
    	speed = (uint16_t)(speed/0.1114);
    	spd   = speed;
    }
    Parameters[0]= P_GOAL_SPEED_L;
    Parameters[1]= (char)speed;

	if(rotation==LEFT){
	 speed = (speed>>8);
	}else{
	 speed = (speed>>8)+4;
	}

    Parameters[2]= (char)speed;
	Parameters[3]= '}';
	error=Write_Servo(id,Automatic,INST_WRITE, Parameters);

	if(error!=0){
			printf("\nERROR %x!!! En el modo rotación continua",error);
	 }else
	  {
		if(rotation==LEFT){
			 printf("\nDynamixel %u gira a la izquierda a una velocidad %u RPM",id,(uint16_t)(spd*0.1114));
		}else{
			 printf("\nDynamixel %u gira a la derecha a una velocidad %u RPM",id,(uint16_t)(spd*0.1114));
		}
	  }
	return error;
}

/*  Función mode_WheelPreload */
uint8_t mode_WheelPreload(uint8_t id, uint8_t rotation, uint16_t speed, uint8_t type){
   uint8_t error=0;
   uint16_t spd=0;
   memset(Parameters,0,DATA_SIZE); /* Resetar array global */

   spd=speed;
   if(type == RPM){
   	speed = (uint16_t)(speed/0.1114);
	spd   = speed;
   }

   Parameters[0]= P_GOAL_SPEED_L;
   Parameters[1]= (char)speed;

	if(rotation==0){
		speed = (speed>>8);
	 }else{
		speed = (speed>>8)+4;
	 }

	Parameters[2]= (char)speed;
	Parameters[3]= '}';

    Write_Servo(id,Automatic,INST_REG_WRITE, Parameters);

	if(error!=0){
			printf("\nERROR %x!!! En el modo rotación continua",error);
	 }else
	  {
		if(rotation==LEFT){
			printf("\nDynamixel %u Modo Wheel, espera Action_Servo para girar a la izquierda a una velocidad de %u RPM",id,(uint16_t)(spd*0.1114));
		}else{
			 printf("\nDynamixel %u Modo Wheel, espera Action_Servo para girar a la derecha a una velocidad de %u RPM",id,(uint16_t)(spd*0.1114));
		}
	  }
	return error;
}

//============================================================================
// FUNCIONES DE LECTURA EN DYNAMIXEL
//============================================================================
/* Función readPos */
int16_t readPos(uint8_t id, uint8_t type){

	uint16_t pos=0;
	uint8_t error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error = Read_Servo(id,P_PRESENT_POSITION_L,2,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura Posición Dynamixel(%u) Incorrecta",error,id);
	  pos = 4096;
	}else
	 {
	   pos = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
	   if(type == DEGREES){
	     	pos = (uint16_t)(pos*0.2932);
	     	printf("\nDynamixel(%u) en posición: %u º",id,pos);
	     }else{
	        printf("\nDynamixel(%u) en posición: %u º",id,(uint16_t)(pos*0.2932));
	      }
	 }
	return pos;
}

/* Función readSpeed */
int16_t readSpeed(uint8_t id, uint8_t type){

	uint16_t Present_Spd=0;
	uint8_t error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_PRESENT_SPEED_L,2,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura Velocidad Actual en Dynamixel(%u) Incorrecta",error,id);
	  Present_Spd = 4096;
	}else
	 {
	  Present_Spd = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
	   if(type == RPM){
		   Present_Spd = (uint16_t)(Present_Spd*0.1114);
		   printf("\nVelocidad Actual Dynamixel %u: %u RPM",id,Present_Spd);
	     }else{
		   printf("\nVelocidad Actual Dynamixel %u: %u RPM",id,(uint16_t)(Present_Spd*0.1114));
	     }
	 }
  return Present_Spd;
}

/* Función readMovSpeed */
int16_t readMovSpeed(uint8_t id, uint8_t type){

	uint16_t Spd=0;
    uint8_t error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error = Read_Servo(id,P_GOAL_SPEED_L,2,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura Velocidad Dynamixel(%u) Incorrecta",error,id);
	  Spd = 4096;
	}else
	 {
		Spd = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
	   if(type == RPM){
		   Spd = (uint16_t)(Spd*0.1114);
		   printf("\nVelocidad Dynamixel(%u): %u RPM",id,Spd);
	     }else{
		   printf("\nVelocidad Dynamixel(%u): %u RPM",id,(uint16_t)(Spd*0.1114));
	     }
	 }
   return Spd;
}

/* Función readTemperature */
int16_t readTemperature(uint8_t id){

	uint16_t  temperature=0;
	uint8_t error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

    error=Read_Servo(id,P_PRESENT_TEMPERATURE,1,Read_Value);

    if(error!= 0){
    	  printf("\nERROR %x!!! Lectura Temperatura en Dynamixel(%u) Incorrecta",error,id);
    	  temperature = 4096;
    	}else
    	 {
    		temperature = (unsigned int)(Read_Value[0]);
    		printf("\nTemperatura Dynamixel(%u): %u ºC",id,temperature);
    	 }
    return temperature;
}

/* Función readVoltage */
int16_t readVoltage(uint8_t id){
    uint16_t voltage=0;
	uint8_t  error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_PRESENT_VOLTAGE,1,Read_Value);

    if(error!= 0){
    	  printf("\nERROR %x!!! Lectura Voltaje en Dynamixel(%u) Incorrecta",error,id);
    	  voltage = 4096;
    	}else
    	 {
    		voltage = (unsigned int)(Read_Value[0]);
    		printf("\nVoltaje Dynamixel(%u): %u V",id,(voltage/10));
    	 }
	return voltage;
}

/* Función readAngleLimit */
int16_t readAngleLimit(uint8_t id, uint8_t Limit_Select, uint8_t type){

    uint16_t angle=0;
    uint8_t error=0;
    memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

    if(Limit_Select == CW){
     error = Read_Servo(id,P_CW_ANGLE_LIMIT_L,2,Read_Value);
    }else{
     error = Read_Servo(id,P_CCW_ANGLE_LIMIT_L,2,Read_Value);
    }

    if(error!= 0){
    	  printf("\nERROR %x!!! Lectura Ángulo Límite en Dynamixel(%u) Incorrecta",error,id);
    	  angle = 4096;
     }else
      {
    	angle = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
    	if(type == DEGREES){
    	  angle = (uint16_t)(angle*0.2932);

    	  if(Limit_Select == CW){
    	     printf("\nLimitado el CW del servo(%u) a %u º",id,angle);
    	   }else{
    	     printf("\nLimitado el CCW del servo(%u) a %u º",id,angle);
    	   }
    	}else{
    	 if(Limit_Select == CW){
    	   printf("\nLimitado el CW del servo(%u) a %u º",id,(uint16_t)(angle*0.2932));
    	  }else{
    	   printf("\nLimitado el CCW del servo(%u) a %u º",id,(uint16_t)(angle*0.2932));
    	  }
    	}
      }
    return angle;
}

/*  Función readLoad */
int16_t readLoad(uint8_t id,uint8_t type){
    uint16_t load=0;
	uint8_t  error=0,direction=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_PRESENT_LOAD_L,2,Read_Value);

    if(error!= 0){
    	  printf("\nERROR %x!!! Lectura Voltaje en Dynamixel(%u) Incorrecta",error,id);
    	  load = 4096;
     }else
      {
    	load = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
    	direction = ((Read_Value[1]<<8)+4);
    	printf("\n%x %x ",Read_Value[0],Read_Value[1]<<8);
    	if(type == PERCENT){
    		load = (uint16_t)(load/10.23);
    		printf("\nLoad Dynamixel(%u): %u %c, sentido %u",id,load,37,direction);
    	}else{
    		printf("\nLoad Dynamixel(%u): %u %c, sentido %u",id,(uint16_t)(load/10.23),37,direction);
    	  }
      }
	return load;
}

/*  Función checkRegister */
int16_t checkRegister(uint8_t id){
	uint16_t reg=0;
	uint8_t  error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_REGISTERED_INSTRUCTION,1,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura Register en Dynamixel(%u) Incorrecta",error,id);
	  reg = 4096;
	 }else
	   {
		 reg = (unsigned int) (Read_Value[0]);
		printf("\nRegister en Dynamixel(%u): %u ",id,reg);
	   }
	return reg;
}

/*  Función checkMovement */
int16_t checkMovement(uint8_t id) {
	uint16_t movement=0;
	uint8_t  error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_MOVING,1,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura Movement en Dynamixel(%u) Incorrecta",error,id);
	  movement = 4096;
	 }else
	   {
		 movement = (unsigned int) (Read_Value[0]);
		printf("\nMovement en Dynamixel(%u): %u ",id,movement);
	   }
	return movement;
}
/*  Función checkLock */
int16_t checkLock(uint8_t id){
	uint16_t lock=0;
	uint8_t  error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_LOCK,1,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura Lock en Dynamixel(%u) Incorrecta",error,id);
	  lock = 4096;
	 }else
	   {
		 lock = (unsigned int) (Read_Value[0]);
		printf("\nLock en Dynamixel(%u): %u ",id,lock);
	   }
	return lock;
}

/*  Función ledState */
int16_t ledState(uint8_t id){
	uint16_t led=0;
	uint8_t  error=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	error=Read_Servo(id,P_LED,1,Read_Value);

	if(error!= 0){
	  printf("\nERROR %x!!! Lectura estado LED en Dynamixel(%u) Incorrecta",error,id);
	  led = 4096;
	 }else
	   {
	  	led = (unsigned int) (Read_Value[0]);
		printf("\nLed en Dynamixel(%u): %u ",id,led);
	   }
	return led;
}
//============================================================================
// FUNCIONES DE LECTURA PLANTA DEL PIE
//============================================================================
int16_t Read_Sharp_Foot(uint8_t id){
  uint8_t error=0;
  uint16_t distance=0;
   memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

   error=Read_Servo(id,P_PRESENT_DISTANCE_SHARP_L,2,Read_Value);

   if(error!= 0){
   	  printf("\nERROR %x!!! Lectura Sharp Pie %u Incorrecta",error,id);
      distance = 4096;
   	 }else
   	   {
   		distance = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
   		printf("\nSharp del pie %u, distancia: %u cm",id,distance);
   	   }
  return distance;
}

int16_t Read_Pressure(uint8_t id, uint8_t sensor){
   uint8_t error=0;
   uint16_t pressure=0;
   memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

   switch (sensor)
   {

   case 0x01:
	   error=Read_Servo(id,P_PRESENT_PRESSURE_SENSOR_1_L,2,Read_Value);
	   break;
   case 0x02:
	   error=Read_Servo(id,P_PRESENT_PRESSURE_SENSOR_2_L,2,Read_Value);
	   break;
   case 0x03:
	   error=Read_Servo(id,P_PRESENT_PRESSURE_SENSOR_3_L,2,Read_Value);
	   break;
   case 0x04:
	   error=Read_Servo(id,P_PRESENT_PRESSURE_SENSOR_4_L,2,Read_Value);
	   break;
   default:
	   printf("\nError! No se puede leer presión del sensor %u seleccionado",sensor);
	   error=127;
	   break;
   }

   if(error!= 0){
   	  printf("\nERROR %x!!! Lectura sensor de presión pie %u Incorrecta",error,id);
   	pressure = 4096;
   	 }else
   	   {
   		pressure = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
   		printf("\nPresión  del sensor %u del pie %u: %u N",sensor,id,pressure);
   	   }
  return pressure;
}

int16_t Read_CM(uint8_t id, uint8_t axis){
	uint8_t error=0;
	uint16_t CM=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

    switch (axis)
	 {
	  case 0x01:
  	    error=Read_Servo(id,P_PRESENT_CM_X_L,2,Read_Value);
  	   if(error!= 0){
  	   	  printf("\nERROR %x!!! Lectura CM eje X del pie %u Incorrecta",error,id);
  	   	  CM = 4096;
  	   	 }else
  	   	   {
  		    CM = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
  		    printf("\nCentro de massas X del pie %u: %u mm",id,CM);
  	   	   }

	    break;
	  case 0x02:
	   error=Read_Servo(id,P_PRESENT_CM_Y_L,2,Read_Value);
  	   if(error!= 0){
  	   	  printf("\nERROR %x!!! Lectura CM eje Y del pie %u Incorrecta",error,id);
  	   	  CM = 4096;
  	   	 }else
  	   	   {
  		    CM = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8));
  		    printf("\nCentro de massas Y del pie %u: %u mm",id,CM);
  	   	   }
	    break;
	  default:
	     printf("\nError! No se puede leer centro de masas del eje %u seleccionado",axis);
		 CM=4096;
	     break;
	  }
   return CM;
}

void Read_CM_Axes(uint8_t id, uint16_t *axis_x, uint16_t *axis_y){

	uint16_t CM=0;
	memset(Read_Value,0,DATA_SIZE); /* Resetar array global */

	Read_Servo(id,P_PRESENT_CM_X_L,4,Read_Value);
	CM = (unsigned int) (Read_Value[0] | (Read_Value[1]<<8)); // Centro de massas eje X
	*axis_x= CM;
	CM = (unsigned int) (Read_Value[2] | (Read_Value[3]<<8)); // Centro de massas eje Y
	*axis_y= CM;

	printf("\nCentro de masas eje X: %u mm y eje Y: %u mm",*axis_x,*axis_y);
}

/* FIN_UTILDYNAMIXEL */


