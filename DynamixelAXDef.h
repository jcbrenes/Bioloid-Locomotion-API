/*
 *  DynamixelAXdef.h
 *
 *  Created on: 3 de mayo de 2016
 *      Author: dasisro
 */

#ifndef DYNAMIXELAX_DEF_H_
#define DYNAMIXELAX_DEF_H_

// Register names according to the datasheet.
// "DYNAMIXEL_AX_12 Register Map and Descriptions Revision 1.0",

//------Control Table Address------------------------------//

//EEPROM AREA
#define P_MODEL_NUMBER_L       0x00   // R
#define P_MODEL_NUMBER_H       0x01   // R
#define P_VERSION              0x02   // R
#define P_ID                   0x03   // R/W
#define P_BAUD_RATE            0x04   // R/W
#define P_RETURN_DELAY_TIME    0x05   // R/W
#define P_CW_ANGLE_LIMIT_L     0x06   // R/W
#define P_CW_ANGLE_LIMIT_H     0x07   // R/W
#define P_CCW_ANGLE_LIMIT_L    0x08   // R/W
#define P_CCW_ANGLE_LIMIT_H    0x09   // R/W
#define P_SYSTEM_DATA2         0x0A   /* Reserved */
#define P_LIMIT_TEMEPERATURE   0x0B   // R/W
#define P_DOWN_LIMIT_VOLTAGE   0x0C   // R/W
#define P_UP_LIMIT_VOLTAGE     0x0D   // R/W
#define P_MAX_TORQUE_L         0x0E   // R/W
#define P_MAX_TORQUE_H         0x0F   // R/W
#define P_RETURN_LEVEL         0x10   // R/W
#define P_ALARM_LED            0x11   // R/W
#define P_ALARM_SHUTDOWN       0x12   // R/W
#define P_OPERATING_MODE       0x13   // R/W
#define P_DOWN_CALIBRATION_L   0x14   // R
#define P_DOWN_CALIBRATION_H   0x15   // R
#define P_UP_CALIBRATION_L     0x16   // R
#define P_UP_CALIBRATION_H     0x17   // R

//RAM AREA
#define P_TORQUE_ENABLE          0x18   // R/W
#define P_LED                    0x19   // R/W
#define P_CW_COMPLIANCE_MARGIN   0x1A   // R/W
#define P_CCW_COMPLIANCE_MARGIN  0x1B   // R/W
#define P_CW_COMPLIANCE_SLOPE    0x1C   // R/W
#define P_CCW_COMPLIANCE_SLOPE   0x1D   // R/W
#define P_GOAL_POSITION_L        0x1E   // R/W
#define P_GOAL_POSITION_H        0x1F   // R/W
#define P_GOAL_SPEED_L           0x20   // R/W
#define P_GOAL_SPEED_H           0x21   // R/W
#define P_TORQUE_LIMIT_L         0x22   // R/W
#define P_TORQUE_LIMIT_H         0x23   // R/W
#define P_PRESENT_POSITION_L     0x24   // R
#define P_PRESENT_POSITION_H     0x25   // R
#define P_PRESENT_SPEED_L        0x26   // R
#define P_PRESENT_SPEED_H        0x27   // R
#define P_PRESENT_LOAD_L         0x28   // R
#define P_PRESENT_LOAD_H         0x29   // R
#define P_PRESENT_VOLTAGE        0x2A   // R
#define P_PRESENT_TEMPERATURE    0x2B   // R
#define P_REGISTERED_INSTRUCTION 0x2C   // R/W
#define P_PAUSE_TIME             0x2D   /* Reserved */
#define P_MOVING                 0x2E   // R
#define P_LOCK                   0x2F   // R/W
#define P_PUNCH_L                0x30   // R/W
#define P_PUNCH_H                0x31   // R/W

//INSTRUCTION
#define INST_PING             0x01
#define INST_READ             0x02
#define INST_WRITE            0X03
#define INST_REG_WRITE        0x04
#define INST_ACTION           0x05
#define INST_RESET            0x06
#define INST_DIGITAL_RESET    0x07  /* No se sabe si funciona */
#define INST_SYSTEM_READ      0x0C  /* No se sabe si funciona */
#define INST_SYSTEM_WRITE     0x0D  /* No se sabe si funciona */
#define INST_SYNC_WRITE       0x83
#define INST_SYNC_REG_WRITE   0x84  /* No se sabe si funciona */

//NUM_SERVOMOTORS
#define BROADCASTING_ID   0xfe
#define S_Dyn0            0x00
#define S_Dyn1            0x01
#define S_Dyn2            0x02
#define S_Dyn3            0X03
#define S_Dyn4            0x04
#define S_Dyn5            0x05
#define S_Dyn6            0x06
#define S_Dyn7            0x07
#define S_Dyn8            0X08
#define S_Dyn9            0x09
#define S_Dyn10           0x0a
#define S_Dyn11           0x0b
#define S_Dyn12           0x0c
#define S_Dyn13           0x0d
#define S_Dyn14           0x0e
#define S_Dyn15           0X0f
#define S_Dyn16           0x10
#define S_Dyn17           0x11
#define S_Dyn18           0x12
#define S_Dyn19			  0x13
#define S_Dyn20           0x14
// To be continued...


//OTHERS
#define Automatic      0x00

#define WHEEL          0x00
#define SERVO          0x01

#define NONE           0x00
#define READ           0x01
#define ALL            0x02

#define OFF            0x00
#define ON             0x01

#define LEFT           0x00
#define RIGHT          0x01

#define HEX            0x00
#define DEGREES        0x01
#define RPM            0x02
#define PERCENT        0x03

#define CW             0x00
#define CCW            0x01

#define Input_Voltage_Error 0x01
#define Angle_Limit_Error   0x02
#define Overheating_Error   0x04
#define Range_Error         0x08
#define CheckSum_Error      0x10
#define Overload_Error      0x20
#define Instruction_Error   0x40
#define No_Response_Error   0x80
#define Read_Error          4096

//Control Table Planta del Pie
//Foots
#define Left_Foot      0x55
#define Right_Foot     0x56

#define P_PRESENT_DISTANCE_SHARP_L     0x00
#define P_PRESENT_DISTANCE_SHARP_H     0x01
#define P_PRESENT_PRESSURE_SENSOR_1_L  0x02
#define P_PRESENT_PRESSURE_SENSOR_1_H  0x03
#define P_PRESENT_PRESSURE_SENSOR_2_L  0x04
#define P_PRESENT_PRESSURE_SENSOR_2_H  0x05
#define P_PRESENT_PRESSURE_SENSOR_3_L  0x06
#define P_PRESENT_PRESSURE_SENSOR_3_H  0x07
#define P_PRESENT_PRESSURE_SENSOR_4_L  0x08
#define P_PRESENT_PRESSURE_SENSOR_4_H  0x09
#define P_PRESENT_CM_X_L               0x0A
#define P_PRESENT_CM_X_H               0x0B
#define P_PRESENT_CM_Y_L               0x0C
#define P_PRESENT_CM_Y_H               0x0D

//Sensores Predefinidos
#define PRESSURE_SENSOR_1              0x01
#define PRESSURE_SENSOR_2              0x02
#define PRESSURE_SENSOR_3              0x03
#define PRESSURE_SENSOR_4              0x04

#define Axis_X                         0x01
#define Axis_Y                         0x02

#endif /* DYNAMIXELAX_DEF_H_ */
