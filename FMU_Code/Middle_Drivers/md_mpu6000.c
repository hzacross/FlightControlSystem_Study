/*******************************************************************************
* 文件名称：md_mpu6000.c
*
* 摘    要：初始化mpu6000的相关设置
*
* 当前版本：
* 作    者： 
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "md_mpu6000.h"
#include "string.h"
#include "md_struct.h"
#include "usart.h"

extern SPI_HandleTypeDef hspi1;
//全局变量，用于保存MPU6000的传感器数据
MPU6000_Data 		m_Mpu6000;

/*******************************函数声明****************************************
* 函数名称: static uint8_t MPU6000_Spi_Com(uint8_t addr, uint8_t data)
* 输入参数: addr:需要读写的寄存器地址；data：需要写入的参数
* 返回参数: uint8_t ：返回寄存器读取的数值
* 功    能: 读取目标寄存器/写入目标寄存器一个字节
* 作    者: by  
* 日    期: 2017/12/18
*******************************************************************************/ 
static uint8_t MPU6000_Spi_Com(uint8_t addr, uint8_t data)
{
	uint8_t txdata[2];
	uint8_t rxdata[2];
	
	txdata[0] = addr;
	txdata[1] = data;
	
	//write data and receive reg value
	MPU_CS_L;
	HAL_Delay(1);
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 2, 500);
	MPU_CS_H;	
	
	return rxdata[1];
}

/*******************************函数声明****************************************
* 函数名称: void MPU6000_Init(void)
* 输入参数: void	
* 返回参数: void 
* 功    能: MPU6000初始化
* 作    者: by  
* 日    期: 2017/12/18
*******************************************************************************/ 
void MPU6000_Init(void)
{
	MPU6000_Spi_Com(PWR_MGMT_1, 0x80);
	HAL_Delay(10);
	MPU6000_Spi_Com(0x68, 0x07);
	HAL_Delay(10);
	
	MPU6000_Spi_Com(PWR_MGMT_1, 0x01);  //配置取样时钟为陀螺X轴
	HAL_Delay(10);
	MPU6000_Spi_Com(PWR_MGMT_2, 0x00);  //使能加速度计、陀螺仪
	HAL_Delay(10);
	/* (1000 / (1+1) = 500Hz) */
	MPU6000_Spi_Com(SMPLRT_DIV, 0x00);  //0x01 
	HAL_Delay(10);
	/*   lowpass : 41 */
	MPU6000_Spi_Com(CONFIG, 0x03);  //0x03
	HAL_Delay(10);
	MPU6000_Spi_Com(ACCEL_CONFIG2, 0x03);
	HAL_Delay(10);
	/* 18: -2000~+2000 */
	MPU6000_Spi_Com(GYRO_CONFIG, 0x18);
	HAL_Delay(10);
	/* 00: -4g~+4g 16384/g */
	MPU6000_Spi_Com(ACCEL_CONFIG, 0x08);
	HAL_Delay(10);
	HAL_Delay(100);
}

/*******************************函数声明****************************************
* 函数名称: void MPU6000_Get_Data(mpu6000_s *mpu6000)
* 输入参数: mpu6000_s:mpu6000定义的结构体
* 返回参数: void	
* 功    能: 读取mpu6000里面的
* 作    者: by  
* 日    期: 2017/12/18
*******************************************************************************/ 
void MPU6000_Get_Data(MPU6000_Data *mpu6000)
{

	uint8_t txdata[15] = {0};
	uint8_t rxdata[15];	
	int16_t tmp;
	
	txdata[0] = ACCEL_XOUT_H | 0x80;
	MPU_CS_L;
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 15, 50);
	MPU_CS_H;
	//acc
	tmp = (rxdata[1] << 8) + rxdata[2];
	mpu6000->acc_raw.x = tmp;
	
	tmp = (rxdata[3] << 8) + rxdata[4];
	mpu6000->acc_raw.y = tmp;

	tmp = (rxdata[5] << 8) + rxdata[6];
	mpu6000->acc_raw.z = tmp;

	//temperature
	mpu6000->temp = (rxdata[7] << 8) + rxdata[8];
	//gyro
	tmp = (rxdata[9] << 8) + rxdata[10];
	mpu6000->gyro_raw.x = tmp;
	
	tmp = (rxdata[11] << 8) + rxdata[12];
	mpu6000->gyro_raw.y = tmp;
	
	
	tmp = (rxdata[13] << 8) + rxdata[14];
	mpu6000->gyro_raw.z = tmp;
}

/*******************************函数声明****************************************
* 函数名称: void Get_Accel(Vector3_Int16 *acc)
* 输入参数: Vector3_Int16：三轴定义的结构体，用于存储X,Y,Z的变量。
* 返回参数:  
* 功    能: 读取加速度计的数据
* 作    者: by  
* 日    期: 2017/12/18
*******************************************************************************/  
void Get_Accel(Vector3_Int16 *acc)
{
	uint8_t txdata[7] = {0};
	uint8_t rxdata[7];
	uint16_t tmp;
	
	txdata[0] = ACCEL_XOUT_H | 0x80;
	MPU_CS_L;
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 7, 50);
	MPU_CS_H;
	tmp = (rxdata[1] << 8) + rxdata[2];
	acc->x = tmp;
	tmp = (rxdata[3] << 8) + rxdata[4];
	acc->y = tmp;
	tmp = (rxdata[5] << 8) + rxdata[6];
	acc->z = tmp;
}

/*******************************函数声明****************************************
* 函数名称: void Get_Gyro(Vector3_Int16 *gyro)
* 输入参数: Vector3_Int16：三轴定义的结构体，用于存储X,Y,Z的变量。
* 返回参数:  
* 功    能: 读取陀螺仪的数据
* 作    者: by  
* 日    期: 2017/12/18
*******************************************************************************/ 
void Get_Gyro(Vector3_Int16 *gyro)
{
	uint8_t txdata[7] = {0};
	uint8_t rxdata[7];
	int16_t tmp;
	
	txdata[0] = GYRO_XOUT_H | 0x80;
	MPU_CS_L;
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 7, 50);
	MPU_CS_H;
	tmp = (rxdata[1] << 8) + rxdata[2];
	gyro->x = tmp;
	tmp = (rxdata[3] << 8) + rxdata[4];
	gyro->y = tmp;
	tmp = (rxdata[5] << 8) + rxdata[6];
	gyro->z = tmp;
}

/*******************************函数声明****************************************
* 函数名称: 
* 输入参数:
* 返回参数:  
* 功    能:
* 作    者:  Across工作室
* 日    期: 
*******************************************************************************/ 
void MPU6000_Test(void)
{
   MPU6000_Data data;
	 MPU6000_Get_Data(&data);
	 printf("%d,%d,%d,%d,%d,%d\r\n",data.acc_raw.x,data.acc_raw.y,data.acc_raw.z,data.gyro_raw.x,data.gyro_raw.y,data.gyro_raw.z);
	 HAL_Delay(500);
}

/*******************************函数声明****************************************
* 函数名称: void Loop_Read_MPU6000(void)
* 输入参数: void	
* 返回参数: void 
* 功    能: 循环读取mpu6000的传感器数据，并将数据保存在全局变量m_Mpu6000中。
* 作    者: Across工作室
* 日    期: 2018.4.3 
*******************************************************************************/ 
void Loop_Read_MPU6000(void)
{
   MPU6000_Get_Data(&m_Mpu6000);
}

