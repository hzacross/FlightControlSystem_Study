/*******************************************************************************
* 文件名称：md_ms56xx.h
*
* 摘    要：初始化mpu5611的相关设置
*
* 当前版本：
* 作    者：Across工作室	
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __MD_ms56xx_H
#define __MD_ms56xx_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "md_struct.h"

/*根据自己的传感器类型选择相应的宏*/
#define MS5607
//#define MS5611

#if !defined (MS5607) && !defined (MS5611)
#error "Please define a macro to decide which sensor that be used!"
#endif

#if defined (MS5607) && defined (MS5611)
#error "These two macros (MS5607,MS5611) cannot be defined at the same time!"
#endif

//mpu6500的SPI使能引脚控制
#define BARO_CS_H     HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_SET)
#define BARO_CS_L     HAL_GPIO_WritePin(BARO_CS_GPIO_Port, BARO_CS_Pin, GPIO_PIN_RESET)


#define CMD_RESET    0x1E
#define CMD_ADC_READ 0x00
#define CMD_ADC_CONV 0x40
#define CMD_ADC_D1   0x00
#define CMD_ADC_D2   0x10

#define CMD_ADC_256  0x00
#define CMD_ADC_512  0x02
#define CMD_ADC_1024 0x04
#define CMD_ADC_2048 0x06
#define CMD_ADC_4096 0x08
#define CMD_PROM_RD  0xA0

void Baro_En_Conv(char cmd);
unsigned long Baro_Read_Data(void);
void Baro_Init(void);
float Baro_Get_Alt(void);
float Baro_Get_Alt2(void);
float Baro_Get_Alt3(void);
void Loop_Read_Bar(void);
void Baro_Read_Pressure_Test(void);

extern MS56XX_Data 	  m_Ms56xx;

#endif 
