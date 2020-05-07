/*******************************************************************************
* 文件名称：md_led_driver.c
*
* 摘    要：STM32F1的LED中间驱动层实现（底层硬件驱动和软件应用层之间的层）
*
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
* 函数名称: void MD_LED_BLUE_Control(uint8_t status)
* 输入参数: uint8_t status：控制灯的亮灭状态。 1：点亮；0：熄灭
* 返回参数: void
* 功    能: 控制blue灯的亮灭
* 作    者: xxx
* 日    期: 2018.3.18
*******************************************************************************/
void MD_LED_Bule_Control(uint8_t status)
{
	if(status==1)
	{
		//点亮
		HAL_GPIO_WritePin(IO_LED_BLUE_GPIO_Port,IO_LED_BLUE_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//熄灭
		HAL_GPIO_WritePin(IO_LED_BLUE_GPIO_Port,IO_LED_BLUE_Pin,GPIO_PIN_SET);
	}
}

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
		HAL_GPIO_WritePin(IO_LED_AMBER_GPIO_Port,IO_LED_AMBER_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//熄灭
		HAL_GPIO_WritePin(IO_LED_AMBER_GPIO_Port,IO_LED_AMBER_Pin,GPIO_PIN_SET);
	}
}

/*******************************函数声明****************************************
* 函数名称: void MD_LED_AMBER_Control(uint8_t status)
* 输入参数: status：控制灯的亮灭状态
* 返回参数:
* 功    能:
* 作    者:
* 日    期: 2017/12/18
*******************************************************************************/
void MD_LED_SAFETY_Control(uint8_t status)
{
	if(status==1)
	{
		//点亮
		HAL_GPIO_WritePin(IO_LED_SAFETY_GPIO_Port,IO_LED_SAFETY_Pin,GPIO_PIN_RESET);
	}
	else
	{
		//熄灭
		HAL_GPIO_WritePin(IO_LED_SAFETY_GPIO_Port,IO_LED_SAFETY_Pin,GPIO_PIN_SET);
	}
}
