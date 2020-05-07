/*******************************************************************************
* 文件名称：md_hmc5883.c
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

#include "md_hmc5883.h"
#include "stdio.h"

#define CYCLE_50HZ_FROM_500HZ   4

static float _hmc5883_Gauss_LSB_XYZ = 1024.0F;  // Varies with gain
__IO uint32_t Mag_RunCount=0;

hmc5883MagGain   _magGain;
hmc5883MagData 	m_Hmc5883;
/*******************************************************************************/
/**
  * @brief  模拟IIC延时
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
void Sim_I2C1_Delay(uint32_t delay)
{
	while(--delay);	//dly=100: 8.75us; dly=100: 85.58 us (SYSCLK=72MHz)
}

/**
  * @brief  模拟IIC开始时序
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
uint8_t Sim_I2C1_START(void)
{
	SDA1_OUT();
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;

// if(!Sim_I2C1_SDA_STATE) return Sim_I2C1_BUS_BUSY;

	Sim_I2C1_SDA_LOW;
	Sim_I2C1_NOP;

	Sim_I2C1_SCL_LOW;
	Sim_I2C1_NOP;

	//if(Sim_I2C1_SDA_STATE) return Sim_I2C1_BUS_ERROR;

	return Sim_I2C1_READY;
}

/**
  * @brief  模拟IIC停止时序
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
void Sim_I2C1_STOP(void)
{
	SDA1_OUT();
	Sim_I2C1_SCL_LOW;
	Sim_I2C1_SDA_LOW;
	Sim_I2C1_NOP;

//	Sim_I2C1_SCL_LOW;
//  Sim_I2C1_NOP;

	Sim_I2C1_SCL_HIG;
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_NOP;
}

unsigned char Sim_I2C1_Wait_Ack(void)
{
	volatile unsigned char ucErrTime=0;
	SDA1_IN();
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_NOP;;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;;
	while(Sim_I2C1_SDA_STATE)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			Sim_I2C1_STOP();
			return 1;
		}
	}
	Sim_I2C1_SCL_LOW;
	return Sim_I2C1_READY;
}

/**
  * @brief  模拟IIC应答时序
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
void Sim_I2C1_SendACK(void)
{
	Sim_I2C1_SCL_LOW;
	SDA1_OUT();
	Sim_I2C1_SDA_LOW;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_LOW;
	Sim_I2C1_NOP;
}

/**
  * @brief  模拟IIC无应答时序
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
void Sim_I2C1_SendNACK(void)
{
	Sim_I2C1_SCL_LOW;
	SDA1_OUT();
	Sim_I2C1_SDA_HIG;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_HIG;
	Sim_I2C1_NOP;
	Sim_I2C1_SCL_LOW;
	Sim_I2C1_NOP;
}

/**
  * @brief  模拟IIC发送单字节时序
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
uint8_t Sim_I2C1_SendByte(uint8_t Sim_i2c_data)
{
	uint8_t i;
	SDA1_OUT();
	Sim_I2C1_SCL_LOW;
	for(i=0; i<8; i++)
	{
		if(Sim_i2c_data&0x80) Sim_I2C1_SDA_HIG;
		else Sim_I2C1_SDA_LOW;

		Sim_i2c_data<<=1;
		Sim_I2C1_NOP;

		Sim_I2C1_SCL_HIG;
		Sim_I2C1_NOP;
		Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
	}
	return Sim_I2C1_READY;
}

/**
  * @brief  模拟IIC读单字节，无应答
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
uint8_t Sim_I2C1_ReceiveByte(void)
{
	uint8_t i,Sim_i2c_data;
	SDA1_IN();
	//Sim_I2C1_SDA_HIG;
// Sim_I2C1_SCL_LOW;
	Sim_i2c_data=0;

	for(i=0; i<8; i++)
	{
		Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
		Sim_I2C1_SCL_HIG;
		// Sim_I2C1_NOP;
		Sim_i2c_data<<=1;

		if(Sim_I2C1_SDA_STATE)	Sim_i2c_data|=0x01;

		// Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
	}
	Sim_I2C1_SendNACK();
	return Sim_i2c_data;
}

/**
  * @brief  模拟IIC读单字节，带应答
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
uint8_t Sim_I2C1_ReceiveByte_WithACK(void)
{

	uint8_t i,Sim_i2c_data;
	SDA1_IN();
	//Sim_I2C1_SDA_HIG;
// Sim_I2C1_SCL_LOW;
	Sim_i2c_data=0;

	for(i=0; i<8; i++)
	{
		Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
		Sim_I2C1_SCL_HIG;
		// Sim_I2C1_NOP;
		Sim_i2c_data<<=1;

		if(Sim_I2C1_SDA_STATE)	Sim_i2c_data|=0x01;

		// Sim_I2C1_SCL_LOW;
		Sim_I2C1_NOP;
	}
	Sim_I2C1_SendACK();
	return Sim_i2c_data;
}


/**
  * @brief  模拟IIC的多字节读
  * @param
  * @note
  * @retval void
  * @author Acorss工作室
  */
uint8_t Sim_DMP_I2C_Read8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{

	Sim_I2C1_START();
	Sim_I2C1_SendByte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	Sim_I2C1_Wait_Ack();
	Sim_I2C1_SendByte(moni_reg_addr);
	Sim_I2C1_Wait_Ack();
	//Sim_I2C1_STOP();
	
	Sim_I2C1_START();
	Sim_I2C1_SendByte(moni_dev_addr << 1 | I2C1_Direction_Receiver);
	Sim_I2C1_Wait_Ack();
	while (moni_i2c_len)
	{
		if (moni_i2c_len==1) *moni_i2c_data_buf =Sim_I2C1_ReceiveByte();
		else *moni_i2c_data_buf =Sim_I2C1_ReceiveByte_WithACK();
		moni_i2c_data_buf++;
		moni_i2c_len--;
	}
	Sim_I2C1_STOP();
	return 0x00;
}

/*******************************************************************************/
/**
  * @brief  模拟IIC的多字节写
  * @param
  * @note   当启用check功能的时候，只能是单字节写的情况，多字接写不可启用check功能
  * @retval void
  * @author Acorss工作室
  */
int8_t Sim_I2C1_Write8(uint8_t moni_dev_addr, uint8_t moni_reg_addr, uint8_t moni_i2c_len, uint8_t *moni_i2c_data_buf)
{
	uint8_t i;
	Sim_I2C1_START();
	Sim_I2C1_SendByte(moni_dev_addr << 1 | I2C1_Direction_Transmitter);
	Sim_I2C1_Wait_Ack();
	Sim_I2C1_SendByte(moni_reg_addr);
	Sim_I2C1_Wait_Ack();
	
	//Sim_I2C1_START();
	for (i=0; i<moni_i2c_len; i++)
	{
		Sim_I2C1_SendByte(moni_i2c_data_buf[i]);
		Sim_I2C1_Wait_Ack();
	}
	Sim_I2C1_STOP();	
		return 0;
}

/*******************************函数声明****************************************
* 函数名称: void HMC5883_Set_MagGain(hmc5883MagGain gain)
* 输入参数: gain：需要设置的增益数值
* 返回参数:  
* 功    能: 设置传感器的增益
* 作    者: by Acorss工作室
* 日    期: 2017/12/18
*******************************************************************************/ 
void HMC5883_Set_MagGain(hmc5883MagGain gain)
{
	uint8_t writebyte;
	writebyte = gain;
  Sim_I2C1_Write8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_CRB_REG_M,1,&writebyte);
	
	_magGain = gain;

  switch(gain)
  {
    case HMC5883_MAGGAIN_1_2:
      _hmc5883_Gauss_LSB_XYZ = 1024.0;
      break;
    case HMC5883_MAGGAIN_1_9:
      _hmc5883_Gauss_LSB_XYZ = 768.0;
      break;
    case HMC5883_MAGGAIN_2_5:
     _hmc5883_Gauss_LSB_XYZ = 614.0;
      break;
    case HMC5883_MAGGAIN_4_0:
    _hmc5883_Gauss_LSB_XYZ = 415.0;
      break;
    case HMC5883_MAGGAIN_4_7:
      _hmc5883_Gauss_LSB_XYZ = 361.0;
      break;
    case HMC5883_MAGGAIN_5_6:
     _hmc5883_Gauss_LSB_XYZ = 307.0;
      break;
    case HMC5883_MAGGAIN_8_1:
      _hmc5883_Gauss_LSB_XYZ = 219.0;
      break;
  } 
}
/*******************************函数声明****************************************
* 函数名称: void HMC5883_Start_Convst(void)
* 输入参数: 
* 返回参数:  
* 功    能: 启动hmc5883的单次测量模式 6ms后可以读数据
* 作    者: by Acorss工作室
* 日    期: 2017/12/18
*******************************************************************************/ 
void HMC5883_Start_Convst(void)
{
	uint8_t writebyte;
  // single measurement mode
	writebyte = 0x01;
  Sim_I2C1_Write8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_MR_REG_M,1,&writebyte);
}

/*******************************函数声明****************************************
* 函数名称: void HMC5883_Init(void)
* 输入参数: 
* 返回参数:  
* 功    能: 初始刷HMC5883：正常模式，增益为正负1.2g，单次测量模式
* 作    者: by Acorss工作室
* 日    期: 2017/12/18
*******************************************************************************/ 
void HMC5883_Init(void)
{
	//确保传感器上电稳定时间
	HAL_Delay(10);	
	uint8_t writebyte;
	/*
	//正常模式 ，75HZ输出频率
	printf("HMC5883_REGISTER_MAG_CRA_REG_M :0x78 \r\n");
	writebyte = 0x78;
	Sim_I2C1_Write8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_CRA_REG_M,1,&writebyte);
	Sim_DMP_I2C_Read8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_CRA_REG_M,1,&writebyte);
	printf("read data:%d\r\n",writebyte);
	
	//增益+/- 1.3
	printf("HMC5883_Set_MagGain :0x20\r\n");
	HMC5883_Set_MagGain(HMC5883_MAGGAIN_1_2);
	Sim_DMP_I2C_Read8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_CRB_REG_M,1,&writebyte);
	printf("read data:%d\r\n",writebyte);
	
	// Contimuous Measurement mode
	printf("Measurement mode :0x00\r\n");
	writebyte = 0x00;
  Sim_I2C1_Write8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_MR_REG_M,1,&writebyte);
	Sim_DMP_I2C_Read8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_MR_REG_M,1,&writebyte);
	printf("read data:%d\r\n",writebyte);
	*/
	/*正常模式，平均采样8次*/
	writebyte = 0x70;
	Sim_I2C1_Write8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_CRA_REG_M,1,&writebyte);
	
	//增益+/- 1.3
	HMC5883_Set_MagGain(HMC5883_MAGGAIN_1_2);
	
	// single measurement mode
	writebyte = 0x01;
  Sim_I2C1_Write8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_MR_REG_M,1,&writebyte);
	//单次装换时间最快在160hz
	
	HAL_Delay(10);	
}

/*******************************函数声明****************************************
* 函数名称: void HMC5883_Read_Data(hmc5883MagData * _magData)
* 输入参数: 
* 返回参数:  
* 功    能: 读取传感器里面三轴的磁场信息
* 作    者: by Acorss工作室
* 日    期: 2017/12/18
*******************************************************************************/ 
uint8_t readBuff[6];
void HMC5883_Read_Data(hmc5883MagData * _magData)
{	
  Sim_DMP_I2C_Read8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_OUT_X_H_M,6,readBuff);
	uint8_t xhi = readBuff[0];
  uint8_t xlo = readBuff[1];
  uint8_t zhi = readBuff[2];
  uint8_t zlo = readBuff[3];
  uint8_t yhi = readBuff[4];
  uint8_t ylo = readBuff[5];
	
	_magData->x = (int16_t)(xlo | ((int16_t)xhi << 8));
  _magData->y = (int16_t)(ylo | ((int16_t)yhi << 8));
  _magData->z = (int16_t)(zlo | ((int16_t)zhi << 8));
	//printf("%lf ,%lf，%lf \r\n",_magData.x ,_magData.y,_magData.z);
	//换算成高斯单位
	_magData->x /= _hmc5883_Gauss_LSB_XYZ;
	_magData->y /= _hmc5883_Gauss_LSB_XYZ;
	_magData->z /= _hmc5883_Gauss_LSB_XYZ;
//	printf("%lf ,%lf，%lf \r\n",_magData->x ,_magData->y,_magData->z);
}

/*******************************函数声明****************************************
* 函数名称: void HMC5883_Read_Identif(void)
* 输入参数: 
* 返回参数:  
* 功    能: 读取ID值
* 作    者: by Acorss工作室
* 日    期: 2017/12/18
*******************************************************************************/ 
void HMC5883_Read_Identif(void)
{
  Sim_DMP_I2C_Read8(HMC5883_ADDRESS_MAG,HMC5883_REGISTER_MAG_IRA_REG_M,3,readBuff);
	printf("IdA:%d,IdB:%d,IdC:%d\r\n",readBuff[0],readBuff[1],readBuff[2]);
	HAL_Delay(100);
}

/*******************************函数声明****************************************
* 函数名称: void HMC5886_Test(void)
* 输入参数: 
* 返回参数:  
* 功    能: 简单的测试函数。验证I2C通信，数据读写是否正常等。
* 作    者: by Acorss工作室
* 日    期: 2017/12/18
*******************************************************************************/ 
void HMC5886_Test(void)
{
  //启动一次转换
	HMC5883_Start_Convst();
	HAL_Delay(10);
	//100hz读地磁数据
	HMC5883_Read_Data(&m_Hmc5883);
}

/*******************************函数声明****************************************
* 函数名称: void Loop_Read_Mag(void)
* 输入参数: 
* 返回参数:  
* 功    能: 50HZ循环地磁采集函数，并将更新的数据保存到m_Hmc5883中。调用该函数前执行一次HMC5883_Start_Convst();
* 作    者: by Acorss工作室
* 日    期: 2018/4/22
*******************************************************************************/ 
void Loop_Read_Mag(void)
{
	//函数执行一次该变量+1
	Mag_RunCount++;

	//Bar_RunCount每变化10次，if语句执行一次，即该if语句50hz执行
	if(Mag_RunCount % CYCLE_50HZ_FROM_500HZ ==0)
	{
      //50hz读地磁数据
			HMC5883_Read_Data(&m_Hmc5883);
				//启动下一次转换
		  HMC5883_Start_Convst();	
	}
}

