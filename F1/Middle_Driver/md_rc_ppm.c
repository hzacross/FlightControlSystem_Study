/*******************************************************************************
* 文件名称：md_rc_ppm.c
*
* 摘    要：遥控器PPM输入解码C文件
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2018/05/15
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "md_rc_ppm.h"
#include <string.h>
#include "tim.h"
#include "string.h"

//记录上次捕捉到的计数器数值
__IO uint16_t ppm_last_count;

//保存遥控器各通道纸
__IO uint16_t ppm_buffer[PPM_MAX_CHANNELS];
__IO ITStatus ppm_frame_captured = RESET;
__IO uint16_t ppm_count_temp=0;
/*******************************函数声明****************************************
* 函数名称: void PPM_Init(void)
* 输入参数: void
* 返回参数: void
* 功    能: PPM解码初始化变量和使能中断
* 作    者: ACROSS
* 日    期: 2018.05.15
*******************************************************************************/
void PPM_Init(void)
{
	memset((uint16_t *)ppm_buffer,0,PPM_MAX_CHANNELS);
	ppm_last_count=0;
	/*开启中断*/
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
}

/*******************************函数声明****************************************
* 函数名称: void PPM_Decode(uint32_t count)
* 输入参数: count：计数器的数值
* 返回参数: void
* 功    能: PPM解码
* 作    者: ACROSS
* 日    期: 2018.05.15
*******************************************************************************/
void PPM_Decode(uint32_t count)
{
	uint16_t width;
	static uint8_t index = 0;
	/*从上次上升沿开始到本次上升沿触发，计数器增加了多少*/
	width = count - ppm_last_count;

	if (count < ppm_last_count)
		width += PPM_MAX_COUNT;/*针对处理计数器满，翻转的情况*/

	ppm_last_count = count;

	if (width > PPM_MIN_START)
	{
		index = 0; /*找到同步位置，下一个触发即为第一通道的数值*/
		ppm_frame_captured=SET;
	}
	else
	{
		if(index<=PPM_MAX_CHANNELS)/*防止意外触发导致数组溢出*/
		{
			ppm_buffer[index++] = width;
		}
	}
}

/*******************************函数声明****************************************
* 函数名称: void PPM_RX_InterruptHandler(void)
* 输入参数: void
* 返回参数: void
* 功    能: PPM中断处理函数
* 作    者: ACROSS
* 日    期: 2018.05.15
*******************************************************************************/
void PPM_RX_InterruptHandler(void)
{
	ppm_count_temp = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_1);
	PPM_Decode(ppm_count_temp);
}

/*******************************函数声明****************************************
* 函数名称: void PPM_GetData(uint16_t *rc_pwm_in,uint8_t length)
* 输入参数: rc_pwm_in：需要传出的数值,length:需要读出的数据长度
* 返回参数: void
* 功    能: 获取PPM包解析出的一组数据
* 作    者: ACROSS
* 日    期: 2018.05.15
*******************************************************************************/
extern __IO ITStatus rc_newdata_flag;
void PPM_GetData(uint16_t *rc_pwm_in,uint8_t length)
{
	if(ppm_frame_captured == SET)
	{
		ppm_frame_captured=RESET;
		for (uint8_t i = 0; i < length; i++)
		{
			rc_pwm_in[i] = ppm_buffer[i];
		  rc_newdata_flag   = SET;
		}
	}
}

