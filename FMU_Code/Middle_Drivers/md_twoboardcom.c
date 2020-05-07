/*******************************************************************************
* 文件名称：md_ACROSScom.c
*
* 摘    要：双机通信相关的串口设置及循环队列的使用
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "md_twoboardcom.h"
#include "ringbuffer.h"
#include <string.h>
#include "checksum.h"


/*
数据包格式
uint8_t head1;      //数据头1 ：0XAA；
uint8_t head2;      //数据头2 ：0X55；
uint8_t length;     //数据有效长度（payload + checksum）
uint8_t payload[n]  //信息包数组
uint16_t cjecksum;  //16位crc校验; 高八位在前。
*/

/*
payload[n] 信息包数组
uint8_t[0];         //type of payload
uint8_t[1...n];     //details of payload
*/

/*****************************************************************************/
/*
  数据包格式如下

	数据头1    数据头2   有效长度     数据1---数据n    CRC校验低八位    CRC校验高八位
	0xAA       0x55      n+2         0xXX----0xXX     0xXX             0xXX
	                      |------CRC校验区域------|
	*/
/*****************************************************************************/

/*发送数据缓冲区数组*/
uint8_t   m_Com_Page_Transmit[TRANSMIT_LENGTH];


/*缓冲区管理变量*/
//ringbuffer管理变量
RingBuffer  m_Com_RX_RingBuffMgr;
//接收数据存放变量
uint8_t     m_Com_RX_SaveBuff[BUFFER_LENGTH];
//存放数据包里面的有效数据长度
__IO uint8_t     m_Com_Page_Length = 0;
__IO uint8_t     m_Com_Page_Incres = 0;
uint8_t     m_Com_Page_Data[100];
enum Com_States e_ComStates = searchingHead1;
extern UART_HandleTypeDef huart6;
extern __IO ITStatus f_ComUartTxReady;

__IO ITStatus f_ComUartTxReady = SET;
__IO ITStatus f_ComUartRxReady = SET;
uint8_t twoboard_rx_data;

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_Transmit(void)
* 输入参数:
* 返回参数:
* 功    能: 将数据发送到另一端处理器
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_Finalize_Message_Send(uint8_t* buf ,uint8_t length)
{
	//1.通过DMA发送数组
	// 先判断上次发送有没有结束，如果还没有则退出此次发送
	if(f_ComUartTxReady == SET)
	{
		f_ComUartTxReady = RESET;
		if(HAL_UART_Transmit_DMA(&huart6, buf, length)!= HAL_OK)
		{
			Error_Handler();
		}
	}
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RX_Enable(void)
* 输入参数:
* 返回参数:
* 功    能: DMA接收使能
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RX_Enable(void)
{
	HAL_UART_Receive_DMA(&huart6, &twoboard_rx_data,1);
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RX_InterruptHandler(void)
* 输入参数:
* 返回参数:
* 功    能: 接收中断处理函数
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RX_InterruptHandler(void)
{
	f_ComUartRxReady = SET;
	Com_TwoBoard_RB_Push(twoboard_rx_data);
	//printf("%d ",twoboard_rx_data);
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_TX_InterruptHandler(void)
* 输入参数:
* 返回参数:
* 功    能: 发送中断处理函数，SET标志
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_TX_InterruptHandler(void)
{
	f_ComUartTxReady = SET;
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RB_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化一个循环队列，用来管理接收到的串口数据。其实就是一个数据缓冲区
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RB_Init(void)
{
	//将m_Com_RX_SaveBuff采用m_Com_RX_RingBuffMgr循环队列进行关联管理。
	rbInit(&m_Com_RX_RingBuffMgr, m_Com_RX_SaveBuff, BUFFER_LENGTH);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Com_TwoBoard_RB_IsReadyToRead(void)
* 输入参数:
* 返回参数:  如果有数据则返回1，反之为0
* 功    能: 判断ringbuffer管理器里面是否有接收但未处理的数据
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t Com_TwoBoard_RB_IsReadyToRead(void)
{
	return !rbIsEmpty(&m_Com_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RB_Clear(void)
* 输入参数:
* 返回参数:
* 功    能: 归零ringbuffer里面的设置。
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RB_Clear(void)
{
	rbClear(&m_Com_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t  Com_TwoBoard_RB_IsOverFlow(void)
* 输入参数:
* 返回参数:  溢出为1，反之为0
* 功    能: 判断缓冲器是否溢出
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t  Com_TwoBoard_RB_IsOverFlow(void)
{
	return m_Com_RX_RingBuffMgr.flagOverflow;
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RB_Push(uint8_t data)
* 输入参数: data：待压入的数据
* 返回参数:
* 功    能: 将接收的数据压入缓冲区
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RB_Push(uint8_t data)
{
	rbPush(&m_Com_RX_RingBuffMgr, (uint8_t)(data & (uint8_t)0xFFU));
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Com_TwoBoard_RB_Pop(void)
* 输入参数:
* 返回参数: uint8_t 读出的数据
* 功    能: 从缓冲器读出数据
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t Com_TwoBoard_RB_Pop(void)
{
	return rbPop(&m_Com_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: uint8_t Com_TwoBoard_RB_HasNew(void)
* 输入参数:
* 返回参数:
* 功    能: 判断是否有新的数据
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t Com_TwoBoard_RB_HasNew(void)
{
	return !rbIsEmpty(&m_Com_RX_RingBuffMgr);
}

/*******************************函数声明****************************************
* 函数名称: void Clear_Page_DatasArea(void)
* 输入参数:
* 返回参数:
* 功    能: 清零相关标志及接收数据的数组
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Clear_Page_DatasArea(void)
{
	m_Com_Page_Length=0;
	m_Com_Page_Incres = 0;
	//清零数组
	memset(m_Com_Page_Data,0, sizeof(m_Com_Page_Data));
}

/*******************************函数声明****************************************
* 函数名称: uint8_t CheckSUM(uint8_t * data ,uint8_t length)
* 输入参数: data：待校验的数据包 ， length：数据包有效位长度
* 返回参数: 数据如果一致返回1，不一致返回0 ；
* 功    能: 计算数据包里面的校验和数据位是否一致。
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t CheckSUM(uint8_t * data ,uint8_t length)
{
	uint16_t CheckSum_Cacl = 0;
	uint16_t CheckSum_Recive;
	CheckSum_Recive += (uint16_t)(data[length-1]);
	CheckSum_Recive += (uint16_t)(data[length-2] <<8);
	for(uint8_t count=0; count<length; count++)
	{
		CheckSum_Cacl +=data[count];
	}
	if(CheckSum_Cacl == CheckSum_Recive)
	{
		return 1;
	}
	else
	{
		return 0 ;
	}
}

/*******************************函数声明****************************************
* 函数名称: void extra_safety_switch_value(uint8_t *buf)
* 输入参数:
* 返回参数:
* 功    能: 提取安全开关的值
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
uint8_t m_safety_switch=0;
void extra_safety_switch_value(uint8_t *buf)
{
	m_safety_switch=buf[0];
	//printf("%d\r\n",m_safety_switch);
}

/*******************************函数声明****************************************
* 函数名称: void extra_rc_input_value(uint8_t *buf)
* 输入参数:
* 返回参数:
* 功    能: 将遥控器的通道值提取出来
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
RC_Channel m_RC_input_from_F1;
__IO ITStatus f_NewRCDataReady = RESET;
void extra_rc_input_value(uint8_t *buf)
{
	UINT16_8BIT m_Value;
	m_Value.value_8[0]=buf[0];
	m_Value.value_8[1]=buf[1];
	m_RC_input_from_F1.channel1 = m_Value.value_16;

	m_Value.value_8[0]=buf[2];
	m_Value.value_8[1]=buf[3];
	m_RC_input_from_F1.channel2 = m_Value.value_16;

	m_Value.value_8[0]=buf[4];
	m_Value.value_8[1]=buf[5];
	m_RC_input_from_F1.channel3 = m_Value.value_16;

	m_Value.value_8[0]=buf[6];
	m_Value.value_8[1]=buf[7];
	m_RC_input_from_F1.channel4 = m_Value.value_16;

	m_Value.value_8[0]=buf[8];
	m_Value.value_8[1]=buf[9];
	m_RC_input_from_F1.channel5 = m_Value.value_16;

	m_Value.value_8[0]=buf[10];
	m_Value.value_8[1]=buf[11];
	m_RC_input_from_F1.channel6 = m_Value.value_16;

	m_Value.value_8[0]=buf[12];
	m_Value.value_8[1]=buf[13];
	m_RC_input_from_F1.channel7 = m_Value.value_16;

	m_Value.value_8[0]=buf[14];
	m_Value.value_8[1]=buf[15];
	m_RC_input_from_F1.channel8 = m_Value.value_16;

	f_NewRCDataReady=SET;
}


void decode_message(uint8_t *buf_msg,uint8_t length)
{
	//根据信息包的类型判断是什么数据
	switch (buf_msg[0])
	{
	case HEARTBEAT_TYPE:
		extra_safety_switch_value(&buf_msg[1]);
		break;
	case RCINPUT_TYPE:
		extra_rc_input_value(&buf_msg[1]);
		break;
	case PWM_TYPE:
		break;

	default:
		break;
	}
}
/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RB_Decode(void)
* 输入参数:
* 返回参数:
* 功    能: 将获取到的数据
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RB_Decode(void)
{
	uint16_t checksum_rx_temp =0;
	/*如果数据包buffer里面溢出了，则清零，重新计数*/
	if(Com_TwoBoard_RB_IsOverFlow())
	{
		Com_TwoBoard_RB_Clear();
	}

	while(Com_TwoBoard_RB_HasNew())
	{
		//提取数据
		uint8_t cur = Com_TwoBoard_RB_Pop();
		//状态机方式处理数据
		switch (e_ComStates)
		{
		case searchingHead1:
			//寻找数据头1
			if(cur == COM_HEAD1)
			{
				//找到数据头1 ，下一个周期跳转到searchingHead2
				e_ComStates = searchingHead2;
			}
			break;
		case searchingHead2:
			if(cur == COM_HEAD2)
			{
				//找到数据头2，则进入receiveDateArea
				e_ComStates = record_VaildLenght;
			}
			else
			{
				//如果数据对应不上，则跳转至searchingHead1重新寻找
				e_ComStates = searchingHead1;
			}
			break;

		case record_VaildLenght:
			Clear_Page_DatasArea();
			//记录数据包有效数据长度
			m_Com_Page_Length = cur;
			//清领数据记录标志，准备记录数据。
			m_Com_Page_Incres = 0;
			m_Com_Page_Data[m_Com_Page_Incres++] = cur;
			e_ComStates = record_VaildDatas;
			break;

		case record_VaildDatas:
			//依次记录数据
			m_Com_Page_Data[m_Com_Page_Incres++] = cur;
			//判断数据是否全部接收完毕
			if(m_Com_Page_Incres==(m_Com_Page_Length+1))
			{
				e_ComStates = decode_Page;
			}
			break;

		case decode_Page:
			m_Com_Page_Data[m_Com_Page_Incres++] = cur;

			if(m_Com_Page_Incres==(m_Com_Page_Length+3))
			{
				//1.先验证数据是否正确
				checksum_rx_temp = (uint16_t)(m_Com_Page_Data[m_Com_Page_Length+2]<<8) | (m_Com_Page_Data[m_Com_Page_Length+1]);
				if(crc_calculate(&m_Com_Page_Data[0],m_Com_Page_Length+BYTES_OF_LENGTH)==checksum_rx_temp)
				{
					//2.如果正确则解码；不正确直接丢弃
					decode_message(&m_Com_Page_Data[1],m_Com_Page_Length);
				}
				//3.清零标志及数据接收区，准备下一包数据接收
				Clear_Page_DatasArea();
				e_ComStates=searchingHead1;
			}
			break;

		default:
			e_ComStates=searchingHead1;
			Clear_Page_DatasArea();
			break;
		}
	}
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_RB_Update(void)
* 输入参数:
* 返回参数:
* 功    能: 处理接收到数据的函数，此函数在main函数中调用。
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_RB_Update(void)
{
	if(Com_TwoBoard_RB_IsReadyToRead())
	{
		Com_TwoBoard_RB_Decode();
	}
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_Msg_HeartBeat(void)
* 输入参数:
* 返回参数:
* 功    能: 心跳包的发送，确定双方通道通信没有障碍。已固定频率发送。F4没有安全开关，可以根据需要填入别的值。
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_Msg_HeartBeat_Send(uint8_t safeSwitch)
{
	/*将数据放入待发送数据区*/
	uint16_t checksum;
	uint8_t buf[HEARTBEAT_BUF_LENGTH]= {0};
	//数据头
	_twoboard_put_uint8_t(buf,0,COM_HEAD1);
	_twoboard_put_uint8_t(buf,1,COM_HEAD2);

	//信息包有效长度
	_twoboard_put_uint8_t(buf,2,HEARTBEAT_VALID_DATA_LENGTH);

	//信息包种类
	_twoboard_put_uint8_t(buf,3,HEARTBEAT_TYPE);
	//信息包数据
	_twoboard_put_uint8_t(buf,4,safeSwitch);

	//计算CRC校验
	checksum = crc_calculate(&buf[2],HEARTBEAT_VALID_DATA_LENGTH+BYTES_OF_LENGTH);
	_twoboard_put_uint16_t(buf,5,checksum);

	Com_TwoBoard_Finalize_Message_Send(buf,HEARTBEAT_BUF_LENGTH);
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_Msg_MotorValue_Send(uint8_t *motor_value)
* 输入参数:
* 返回参数:
* 功    能: 发送电机的驱动值
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_Msg_MotorValue_Send(d_Motor *pMotor)
{
	/*将数据放入待发送数据区*/
	uint16_t checksum;
	static uint8_t buf_Motor[PWM_BUF_LENGTH]= {0};
	//数据头
	_twoboard_put_uint8_t(buf_Motor,0,COM_HEAD1);
	_twoboard_put_uint8_t(buf_Motor,1,COM_HEAD2);

	//信息包有效长度
	_twoboard_put_uint8_t(buf_Motor,2,PWM_VALID_DATA_LENGTH);

	//信息包种类
	_twoboard_put_uint8_t(buf_Motor,3,PWM_TYPE);
	//信息包数据
	_twoboard_put_uint16_t(buf_Motor,4,pMotor->M1);
	_twoboard_put_uint16_t(buf_Motor,6,pMotor->M2);
	_twoboard_put_uint16_t(buf_Motor,8,pMotor->M3);
	_twoboard_put_uint16_t(buf_Motor,10,pMotor->M4);
	_twoboard_put_uint16_t(buf_Motor,12,pMotor->M5);
	_twoboard_put_uint16_t(buf_Motor,14,pMotor->M6);

	//计算CRC校验
	checksum = crc_calculate(&buf_Motor[2],RCINPUT_VALID_DATA_LENGTH+BYTES_OF_LENGTH);
	_twoboard_put_uint16_t(buf_Motor,16,checksum);

	Com_TwoBoard_Finalize_Message_Send(buf_Motor,PWM_BUF_LENGTH);
}

/*******************************函数声明****************************************
* 函数名称: void Com_TwoBoard_Init(void)
* 输入参数:
* 返回参数:
* 功    能: 初始化双机通信的相关设置
* 作    者: by ACROSS
* 日    期: 2017/12/18
*******************************************************************************/
void Com_TwoBoard_Init(void)
{
	Com_TwoBoard_RB_Init();
	Com_TwoBoard_RX_Enable();
}
