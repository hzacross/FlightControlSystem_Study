/*******************************************************************************
* 文件名称：md_rc.h
*
* 摘    要：遥控器相关头文件
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2018/05/15
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __MD_RC_H
#define __MD_RC_H

#include "stm32f1xx_hal.h"
#include "main.h"

#define RC_INPUT_CHANNELS  8

extern __IO uint8_t rc_frame_byte;
extern uint16_t rc_pwm_in[RC_INPUT_CHANNELS];

void RC_Input_Init(void);
void RC_Input_Loop(void);

#endif

