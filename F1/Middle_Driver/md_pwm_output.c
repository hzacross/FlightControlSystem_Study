/*******************************************************************************
* 文件名称：md_pwm_output.c
*
* 摘    要：STM32F1的PWM输出驱动
*
* 当前版本：
* 作    者：Acorss	
* 日    期：2018/05/23
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "md_pwm_output.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Output_Enable(void)
* 输入参数: 
* 返回参数:  
* 功    能: 设置PWM输出使能
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Output_Enable(void)
{
	//初始化是输出设置为10；
	PWM_Set_Value_Channel_1(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_2(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_3(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_4(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_5(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_6(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_7(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_8(PWM_MINI_OUTPTU_TO_DRIVER);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Output_Enable(void)
* 输入参数: 
* 返回参数:  
* 功    能: 设置PWM输出使能
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Output_Disable(void)
{
	//初始化是输出设置为10；
	PWM_Set_Value_Channel_1(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_2(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_3(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_4(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_5(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_6(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_7(PWM_MINI_OUTPTU_TO_DRIVER);
	PWM_Set_Value_Channel_8(PWM_MINI_OUTPTU_TO_DRIVER);
  HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim4,TIM_CHANNEL_4);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop(&htim3,TIM_CHANNEL_4);
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_1(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道1的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_1(uint16_t value)
{
	htim2.Instance->CCR1 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_2(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道2的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_2(uint16_t value)
{
  htim2.Instance->CCR2 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_3(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道3的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_3(uint16_t value)
{
  htim4.Instance->CCR3 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_4(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道4的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_4(uint16_t value)
{
    htim4.Instance->CCR4 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_5(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道5的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_5(uint16_t value)
{
   htim3.Instance->CCR1 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_6(uint16_t value)
* 输入参数:value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道6的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_6(uint16_t value)
{
   htim3.Instance->CCR2 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_7(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道7的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_7(uint16_t value)
{
   htim3.Instance->CCR3 = value;
}

/*******************************函数声明****************************************
* 函数名称: void PWM_Set_Value_Channel_8(uint16_t value)
* 输入参数: value：最小最大数值为0--2499。实际的电机驱动输入有效值为1094--1933；
* 返回参数:  
* 功    能: 设置通道8的输出
* 作    者: by Acorss
* 日    期: 2018/05/23
*******************************************************************************/ 
void PWM_Set_Value_Channel_8(uint16_t value)
{
   htim3.Instance->CCR4 = value;
}
