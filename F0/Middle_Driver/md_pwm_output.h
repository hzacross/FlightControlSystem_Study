/*******************************************************************************
* 文件名称：md_pwm_output.h
*
* 摘    要：STM32F1的PWM输出驱动
*
* 当前版本：
* 作    者：xiaodaqi	
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_PWM_OUTPUT_H
#define __MD_PWM_OUTPUT_H

#include "main.h"
#include "stm32f1xx_hal.h"

#define PWM_MINI_OUTPTU_TO_DRIVER  1000   //电调0转速脉冲时间1000us
#define PWM_MAX_OUTPTU_TO_DRIVER   2000   //电调最大转速脉冲时间2000us

void PWM_Set_Output_Enable(void);
void PWM_Set_Value_Channel_1(uint16_t value);
void PWM_Set_Value_Channel_2(uint16_t value);
void PWM_Set_Value_Channel_3(uint16_t value);
void PWM_Set_Value_Channel_4(uint16_t value);
void PWM_Set_Value_Channel_5(uint16_t value);
void PWM_Set_Value_Channel_6(uint16_t value);
void PWM_Set_Value_Channel_7(uint16_t value);
void PWM_Set_Value_Channel_8(uint16_t value);

#endif
