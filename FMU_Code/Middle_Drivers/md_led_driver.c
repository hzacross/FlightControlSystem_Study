/*******************************************************************************
* 文件名称：md_led_driver.c
*
* 摘    要：STM32F1的LED中间驱动层实现（底层硬件驱动和软件应用层之间的层）
*
* 当前版本：
* 作    者：
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "md_led_driver.h"


/*******************************函数声明****************************************
* 函数名称: void MD_LED_AMBER_Control(uint8_t status)
* 输入参数: status：控制灯的亮灭状态
* 返回参数:  
* 功    能:
* 作    者: 
* 日    期: 2017/12/18
*******************************************************************************/ 
void MD_LED_AMBER_Control(uint8_t status)
{
	if(status==1)
	{
		//点亮
	  HAL_GPIO_WritePin(FMU_LED_AMBER_GPIO_Port,FMU_LED_AMBER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//熄灭
	   HAL_GPIO_WritePin(FMU_LED_AMBER_GPIO_Port,FMU_LED_AMBER_Pin,GPIO_PIN_SET);
	}
}




uint8_t count_led=0;

void Red_Fast_Flashing_Control(void)
{
	if(count_led%2 ==0)
	{
		static uint8_t falg_Red_Fast_Flashing=0;
		if(falg_Red_Fast_Flashing==1)
		{
			//点亮红灯
			MD_LED_AMBER_Control(LED_ON);
			falg_Red_Fast_Flashing=0;
		}
		else
		{
			//熄灭红灯
			MD_LED_AMBER_Control(LED_OFF);
			falg_Red_Fast_Flashing=1;
		}
	}
}

void Red_Slow_Flashing_control(void)
{
	if(count_led%5 ==0)
	{
		static uint8_t falg_Red_Slow_Flashing=0;
		if(falg_Red_Slow_Flashing==1)
		{
			MD_LED_AMBER_Control(LED_ON);
			falg_Red_Slow_Flashing=0;
		}
		else
		{
			MD_LED_AMBER_Control(LED_OFF);
			falg_Red_Slow_Flashing=1;
		}
	}
}

void LED_Display(enum LED_STATE mRed_State)
{
	switch(mRed_State)
	{
		case Red_OFF_Light:
			MD_LED_AMBER_Control(LED_OFF);
		break;
		
		case Red_Fast_Flashing:
			Red_Fast_Flashing_Control();
		break;
		
		case Red_Slow_Flashing:
			Red_Slow_Flashing_control();
		break;
		
		case Red_ON_Lighting:
			MD_LED_AMBER_Control(LED_ON);
		break;
		
		default:
			MD_LED_AMBER_Control(LED_OFF);
		break;
	}
}	



enum LED_STATE mRed_State   = Red_OFF_Light;

void LED_Handler(void)
{
	count_led++;
	count_led = count_led >= 50 ? 0 : count_led;
	LED_Display(mRed_State);
}

void set_mRed_State(enum LED_STATE state)
{
	mRed_State = state;
}

