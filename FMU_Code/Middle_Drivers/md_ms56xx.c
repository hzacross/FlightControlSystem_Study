/*******************************************************************************
* 文件名称：md_ms56xx.c
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

#include "md_ms56xx.h"
#include "math.h"
#include "stdio.h"

#define CYCLE_100HZ_FROM_500HZ   2

extern SPI_HandleTypeDef hspi1;

const double T1 = 15.0 + 273.15;	/* temperature at base height in Kelvin */
const double A  = -6.5 / 1000;		/* temperature gradient in degrees per metre */
const double g  = 9.80665;				/* gravity constant in m/s/s */
const double RR  = 287.05;				/* ideal gas constant in J/kg/K */
const double p1 = 1013.15;
uint16_t C[8];  							// calibration coefficients

__IO ITStatus Bar_Conv_Flag = RESET;
__IO uint32_t Bar_RunCount=0;
__IO uint32_t D1; // ADC value of the pressure conversion
__IO uint32_t D2; // ADC value of the temperature conversion
//气压计读取的气压值
MS56XX_Data 	  m_Ms56xx;

/*******************************函数声明****************************************
* 函数名称: void Baro_Reset(void)
* 输入参数:
* 返回参数:
* 功    能: 复位ms56xx传感器
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
void Baro_Reset(void)
{
	uint8_t txdata = CMD_RESET;
	BARO_CS_L;
	HAL_SPI_Transmit(&hspi1, &txdata, 1, 50);
	HAL_Delay(10);
	BARO_CS_H;
}

/*******************************函数声明****************************************
* 函数名称: unsigned int Baro_Cmd_Prom(char coef_num)
* 输入参数: coef_num：系数number
* 返回参数:
* 功    能: 读取传感器里面的系数
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
unsigned int Baro_Cmd_Prom(char coef_num)
{
	unsigned int rc = 0;
	uint8_t txdata[3] = {0}, rxdata[3] = {0};

	BARO_CS_L;
	txdata[0] = CMD_PROM_RD + coef_num * 2;
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 3, 50);
	BARO_CS_H;

	rc = 256 * rxdata[1] + rxdata[2];

	return rc;
}

/*******************************函数声明****************************************
* 函数名称: void Baro_Read_Coe(void)
* 输入参数:
* 返回参数:
* 功    能: 读取参数
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
void Baro_Read_Coe(void)
{
	// read calibration coefficients
	for (uint8_t i = 0; i < 8; i++)
	{
		C[i] = Baro_Cmd_Prom(i);
	}
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Baro_Crc4_Check(uint32_t n_prom[])
* 输入参数: n_prom[]：读取的系数参数变量
* 返回参数: uint8_t：返回计算的CRC校验值。正确校验返回值应该是0X0B；
* 功    能: CRC 计算，用来验证读取到的8位参数系数是否正确
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t Baro_Crc4_Check(uint16_t n_prom[])
{
	uint32_t cnt;
	uint32_t n_rem, crc_read;
	uint8_t n_bit;

	crc_read = n_prom[7];
	n_prom[7] = (0xff00 & (n_prom[7]));

	for (cnt = 0; cnt < 16; cnt++)
	{
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
				n_rem = (n_rem << 1) ^ 0x3000;
			else
				n_rem = (n_rem << 1);
		}
	}

	n_rem = (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
	n_prom[7] = crc_read;

	return (n_rem ^ 0x00);
}



/*******************************函数声明****************************************
* 函数名称: void Baro_En_Conv(char cmd)
* 输入参数:
* 返回参数:
* 功    能: enable pressure adc cov
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
void Baro_En_Conv(char cmd)
{
	uint8_t txdata;

	BARO_CS_L;
	txdata = CMD_ADC_CONV + cmd;
	HAL_SPI_Transmit(&hspi1, &txdata, 1, 50);
	BARO_CS_H;
}

/*******************************函数声明****************************************
* 函数名称: void Baro_Enable_Temp_Conv(void)
* 输入参数:
* 返回参数:
* 功    能: 使能温度转换功能
* 作    者: by xiaodaqi
* 日    期: 2017/12/18
*******************************************************************************/
void Baro_Enable_Temp_Conv(void)
{
	Baro_En_Conv(CMD_ADC_D2+CMD_ADC_4096);
}

/*******************************函数声明****************************************
* 函数名称: void Baro_Enable_Press_Conv(void)
* 输入参数:
* 返回参数:
* 功    能: 使能压力转换功能
* 作    者: by xiaodaqi
* 日    期: 2017/12/18
*******************************************************************************/
void Baro_Enable_Press_Conv(void)
{
	Baro_En_Conv(CMD_ADC_D1+CMD_ADC_4096);
}

/*******************************函数声明****************************************
* 函数名称: unsigned long Baro_Read_Data(void)
* 输入参数:
* 返回参数:  返回读取的数值
* 功    能: read adc result, after calling Baro_En_Conv()
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
unsigned long Baro_Read_Data(void)
{
	uint8_t txdata[4];
	uint8_t rxdata[4];
	unsigned long temp;

	BARO_CS_L;
	txdata[0] = CMD_ADC_READ;
	HAL_SPI_TransmitReceive(&hspi1, txdata, rxdata, 4, 50);
	BARO_CS_H;

	temp = 65535 * rxdata[1] + 256 * rxdata[2] + rxdata[3];

	return temp;
}

/*******************************函数声明****************************************
* 函数名称: float Baro_Cal_Alt(unsigned long D1, unsigned long D2)
* 输入参数: D1:气压原始数值 ； D2温度原始数值
* 返回参数:
* 功    能: 计算的气压值（单位mba）
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
float Baro_Cal_Alt(unsigned long D1, unsigned long D2)
{
	int32_t P; // compensated pressure value
	int32_t TEMP; // compensated temperature value
	int32_t dT; // difference between actual and measured temperature
	int64_t OFF,OFF2; // offset at actual temperature
	int64_t SENS,SENS2; // sensitivity at actual temperature

	dT = D2 - (uint32_t)C[5] * 256;

#ifdef MS5611
	OFF = (int64_t)C[2] * 65536 + (int64_t)C[4] * dT / 128;
	SENS = (int64_t)C[1] * 32768 + (int64_t)C[3] * dT / 256;
#endif

#ifdef MS5607
	OFF = (int64_t)C[2] * 131072 + (int64_t)C[4] * dT / 64;
	SENS = (int64_t)C[1] * 65536 + (int64_t)C[3] * dT / 128;
#endif

	TEMP = 2000 + ((int64_t) dT * C[6]) / 8388608;
	//printf("TEMP is ：%ld  ",TEMP);

	OFF2 = 0;
	SENS2 = 0;

	if (TEMP < 2000)
	{
		OFF2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 2;
		SENS2 = 5 * ((TEMP - 2000) * (TEMP - 2000)) / 4;
	}

	if (TEMP < -1500)
	{
		OFF2 = OFF2 + 7 * ((TEMP + 1500) * (TEMP + 1500));
		SENS2 = SENS2 + 11 * ((TEMP + 1500) * (TEMP + 1500)) / 2;
	}

	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	P = (D1 * SENS / 2097152 - OFF) / 32768;
	//printf(" P is ：%ld \r\n ", P);
	return P;
}

/*******************************函数声明****************************************
* 函数名称: void Baro_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化ms56xx
* 作    者: by Across工作室
* 日    期: 2017/12/18
*******************************************************************************/
void Baro_Init(void)
{
	Baro_Reset();
	Baro_Read_Coe();
	//启动一次温度转换
	Baro_Enable_Temp_Conv();
}

/*******************************函数声明****************************************
* 函数名称: void Baro_Read_Pressure_Test(void)
* 输入参数:
* 返回参数:
* 功    能: 测试传感器数据读取是否正确。
* 作    者: Across工作室
* 日    期: 2018.4.15
*******************************************************************************/
void Baro_Read_Pressure_Test(void)
{
	/* 1.启动一次气压转换 */
	Baro_Enable_Press_Conv();
	/* 2.延时10ms，后读取转换后的温度数值*/
	HAL_Delay(10);
	D1 = Baro_Read_Data();
	/* 3.启动一次压力转换*/
	Baro_Enable_Temp_Conv();
	/*4.延时10ms，读取转换后的压力数值*/
	HAL_Delay(10);
	D2 = Baro_Read_Data();
	/* 5.利用公式计算最终的气压数值。*/
	m_Ms56xx.pressure = Baro_Cal_Alt(D1,D2);
	printf("pressure is %lf mbar \r\n",m_Ms56xx.pressure/100.0);
}

/*******************************函数声明****************************************
* 函数名称: void Loop_Read_Bar(void)
* 输入参数:
* 返回参数:
* 功    能: 500hz的频率调用该函数，最后计算的气压计数据会以50hz频率更新。
            注意：再第一次条用该函数前，调用启动一次温度转换。
* 作    者:  Across工作室
* 日    期:
*******************************************************************************/
void Loop_Read_Bar(void)
{
	//函数执行一次该变量+1
	Bar_RunCount++;

	//Bar_RunCount每变化5次，if语句执行一次，即该if语句100hz执行
	if(Bar_RunCount % CYCLE_100HZ_FROM_500HZ ==0)
	{
		if(Bar_Conv_Flag==RESET)
		{
			Bar_Conv_Flag = SET;
			//获取温度raw值
			D2 = Baro_Read_Data();
			//启动下一次气压转换
			Baro_Enable_Press_Conv();
		}
		else
		{
			Bar_Conv_Flag=RESET;
			//获取气压raw值
			D1 = Baro_Read_Data();
			m_Ms56xx.pressure = Baro_Cal_Alt(D1,D2);			
			//启动下一次温度转换
			Baro_Enable_Temp_Conv();
			//printf("pressure is %lf mbar \r\n",m_Ms56xx.pressure/100.0);
      //printf("%ld\r\n",m_Ms56xx.pressure);			
		}

	}
}

