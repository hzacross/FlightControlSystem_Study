/*******************************************************************************
* 文件名称：md_mpu6000.h
*
* 摘    要：初始化mpu6000的相关设置
*
* 当前版本：
* 作    者：xiaodaqi	
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __MD_MPU6000_H
#define __MD_MPU6000_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "md_struct.h"

//mpu6000的SPI使能引脚控制
#define MPU_CS_H     HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_SET)
#define MPU_CS_L     HAL_GPIO_WritePin(MPU_CS_GPIO_Port, MPU_CS_Pin, GPIO_PIN_RESET)

//macros
#define	SMPLRT_DIV	  	0x19
#define SMPLRT_DIV_R    (0x19 | 0x80)
#define	CONFIG			    0x1A
#define	CONFIG_R			  (0x1A | 0x80)
#define	GYRO_CONFIG		  0x1B
#define	GYRO_CONFIG_R   (0x1B | 0x80)
#define	ACCEL_CONFIG	  0x1C
#define	ACCEL_CONFIG2	  0x1D
#define	ACCEL_CONFIG_R	(0x1C | 0x80)
#define	ACCEL_XOUT_H	  0x3B
#define	ACCEL_XOUT_L	  0x3C
#define	ACCEL_YOUT_H	  0x3D
#define	ACCEL_YOUT_L  	0x3E
#define	ACCEL_ZOUT_H	  0x3F
#define	ACCEL_ZOUT_L  	0x40
#define	TEMP_OUT_H		  0x41
#define	TEMP_OUT_L		  0x42
#define	GYRO_XOUT_H		  0x43
#define	GYRO_XOUT_L		  0x44	
#define	GYRO_YOUT_H		  0x45
#define	GYRO_YOUT_L		  0x46
#define	GYRO_ZOUT_H		  0x47
#define	GYRO_ZOUT_L		  0x48
#define	PWR_MGMT_1		  0x6B
#define	PWR_MGMT_1_R		(0x6B | 0x80)
#define PWR_MGMT_2      0x6C
#define PWR_MGMT_2_R    (0x6C | 0x80)
#define	WHO_AM_I			 0x75

void MPU6000_Init(void);
void MPU6000_Get_Data(MPU6000_Data *mpu6000);
void Get_Accel(Vector3_Int16 *acc);
void Get_Gyro(Vector3_Int16 *gyro);
void MPU6000_Test(void);
void Loop_Read_MPU6000(void);
#endif // mpu6000_h
