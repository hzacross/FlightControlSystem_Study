/*******************************************************************************
* 文件名称：md_led_driver.h
*
* 摘    要：
*
* 当前版本：
* 作    者：
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_LED_DRIVER_H
#define __MD_LED_DRIVER_H

#include "md_include.h"

#define Status_Safety   HAL_GPIO_ReadPin(SAFETY_GPIO_Port,SAFETY_Pin)

void MD_LED_Bule_Control(uint8_t status);
void MD_LED_AMBER_Control(uint8_t status);
void MD_LED_SAFETY_Control(uint8_t status);

#endif

