/*******************************************************************************
* 文件名称：md_struct.h
*
* 摘    要：自定义结构体
*
* 当前版本：
* 作    者：
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_STRUCT_H
#define __MD_STRUCT_H

#include "stm32f4xx_hal.h"
#include "main.h"

/** 
  * @brief  LED Status enumeration 
  */
typedef enum
{
  LED_OFF = 0U,
  LED_ON
}LED_Status;

typedef struct{
	int16_t x;
	int16_t y;
	int16_t z;
} Vector3_Int16;

typedef struct{
	float x;
	float y;
	float z;
}Vector3_Float;

typedef struct{
	Vector3_Int16 acc_raw;
	Vector3_Int16 gyro_raw;
	Vector3_Float accf;
	Vector3_Float gyrof;
	int16_t temp;   //temperature
}MPU6000_Data;

/* MS56XX气压计时定义的结构体*/
typedef struct {
  int32_t temp;
	int32_t pressure;
} MS56XX_Data;

/*hmc5883地磁计定义的结构体*/
typedef struct {
        float x;
        float y;
        float z;
        float orientation;
} hmc5883MagData;

typedef struct  
{
  uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
	uint16_t motor5;
	uint16_t motor6;
}Motor_Value;

typedef struct  
{
  uint16_t channel1;
	uint16_t channel2;
	uint16_t channel3;
	uint16_t channel4;
	uint16_t channel5;
	uint16_t channel6;
	uint16_t channel7;
	uint16_t channel8;
}RC_Channel;

typedef struct {
	float Roll;
	float Pitch;
	float Throttle;
	float Yaw;
	uint16_t Switch_A;
	uint16_t Switch_B;
	uint16_t Switch_C;
	uint16_t Switch_D;
}d_RC;

typedef union
{
  uint16_t value_16;
	uint8_t  value_8[2];
}UINT16_8BIT;

typedef struct{
	Vector3_Float accf;
	Vector3_Float gyrof;
	Vector3_Float magf;
	float roll;
	float pitch;
	float yaw;
	
	uint8_t xy_valid;
	uint8_t z_valid;
	uint8_t v_xy_valid;
	uint8_t v_z_valid;
	
	//position and velocity in NED frame
	float x, y, z;
	float vx, vy, vz;
	float pre_vx,pre_vy,pre_vz;
	float xacc,yacc,zacc;
	
	double ref_lat;
	double ref_lon;
	float ref_alt;
	float Altitude_Ground;
	
}Control_state;

typedef struct{
	int M1;
	int M2;
	int M3;
	int M4;
	int M5;
	int M6;
	int M7;
	int M8;
}d_Motor;


#define CONSTRAINT(in, min, max)  (in > max ? max : (in < min ? min : in))
#define ABS(a) ((a) > 0 ? (a) : -(a))
#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

#define M_PI_F			(3.14159265358f)
#define M_TWOPI_F       (M_PI_F * 2.0f)

#define sq(x) ((x)*(x))
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)

#endif
