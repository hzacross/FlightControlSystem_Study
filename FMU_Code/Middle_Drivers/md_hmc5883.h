/*******************************************************************************
* 文件名称：md_hmc5883.h
*
* 摘    要：1.采用软件模拟I2C通信协议
*           2.初始化hmc5883的相关设置,
*
* 当前版本：
* 作    者：Acorss工作室	
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_HMC5883_H
#define __MD_HMC5883_H

#include "stm32f4xx_hal.h"
#include "main.h"
#include "md_struct.h"

#define SDA1_IN()  FMU_I2C1_SDA_GPIO_Port->MODER &= ~(0x3<<(9*2))
#define SDA1_OUT() FMU_I2C1_SDA_GPIO_Port->MODER &= ~(0x3<<(9*2));FMU_I2C1_SDA_GPIO_Port->MODER |= (0x1<<(9*2))

#define Sim_I2C1_SCL_LOW          (FMU_I2C1_SCL_GPIO_Port->ODR &= ~(FMU_I2C1_SCL_Pin))
#define Sim_I2C1_SCL_HIG          (FMU_I2C1_SCL_GPIO_Port->ODR |=  (FMU_I2C1_SCL_Pin))
#define Sim_I2C1_SDA_LOW          (FMU_I2C1_SDA_GPIO_Port->ODR &= ~(FMU_I2C1_SDA_Pin))
#define Sim_I2C1_SDA_HIG          (FMU_I2C1_SDA_GPIO_Port->ODR |=  (FMU_I2C1_SDA_Pin))

#define Sim_I2C1_SDA_STATE        (FMU_I2C1_SDA_GPIO_Port->IDR &= (FMU_I2C1_SDA_Pin))

#define Sim_I2C1_DELAY 		Sim_I2C1_Delay(100000)
#define Sim_I2C1_NOP		  Sim_I2C1_Delay(5)  //25 

#define Sim_I2C1_READY			0x00
#define Sim_I2C1_BUS_BUSY		0x01
#define Sim_I2C1_BUS_ERROR	0x02

#define Sim_I2C1_NACK	      0x00
#define Sim_I2C1_ACK				0x01

#define I2C1_Direction_Transmitter 		0x00
#define I2C1_Direction_Receiver   	  0x01

#define HMC5883_ADDRESS_MAG           0x1E

typedef enum
    {
      HMC5883_REGISTER_MAG_CRA_REG_M             = 0x00,
      HMC5883_REGISTER_MAG_CRB_REG_M             = 0x01,
      HMC5883_REGISTER_MAG_MR_REG_M              = 0x02,
      HMC5883_REGISTER_MAG_OUT_X_H_M             = 0x03,
      HMC5883_REGISTER_MAG_OUT_X_L_M             = 0x04,
      HMC5883_REGISTER_MAG_OUT_Z_H_M             = 0x05,
      HMC5883_REGISTER_MAG_OUT_Z_L_M             = 0x06,
      HMC5883_REGISTER_MAG_OUT_Y_H_M             = 0x07,
      HMC5883_REGISTER_MAG_OUT_Y_L_M             = 0x08,
      HMC5883_REGISTER_MAG_SR_REG_Mg             = 0x09,
      HMC5883_REGISTER_MAG_IRA_REG_M             = 0x0A,
      HMC5883_REGISTER_MAG_IRB_REG_M             = 0x0B,
      HMC5883_REGISTER_MAG_IRC_REG_M             = 0x0C,
      HMC5883_REGISTER_MAG_TEMP_OUT_H_M          = 0x31,
      HMC5883_REGISTER_MAG_TEMP_OUT_L_M          = 0x32
    } hmc5883MagRegisters_t;

typedef enum
    {
      HMC5883_MAGGAIN_1_2                        = 0x20,  // +/- 1.3
      HMC5883_MAGGAIN_1_9                        = 0x40,  // +/- 1.9
      HMC5883_MAGGAIN_2_5                        = 0x60,  // +/- 2.5
      HMC5883_MAGGAIN_4_0                        = 0x80,  // +/- 4.0
      HMC5883_MAGGAIN_4_7                        = 0xA0,  // +/- 4.7
      HMC5883_MAGGAIN_5_6                        = 0xC0,  // +/- 5.6
      HMC5883_MAGGAIN_8_1                        = 0xE0   // +/- 8.1
    } hmc5883MagGain;	
		
void HMC5883_Start_Convst(void);		
void HMC5883_Init(void);
void HMC5883_Read_Identif(void);
void HMC5883_Read_Data(hmc5883MagData * _magData);
void HMC5886_Test(void);	
void Loop_Read_Mag(void);		
#endif 
		
