/*******************************************************************************
* 文件名称：md_rc_sbus.h
*
* 摘    要：遥控器sbus输入解码头文件
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2018/05/15
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __MD_RC_SBUS_H
#define __MD_RC_SBUS_H

#include "stm32f1xx_hal.h"
#include "main.h"

/*
 * s-bus decode
 */

#define SBUS_FRAME_SIZE		25
#define SBUS_INPUT_CHANNELS	16
#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2
#define SBUS1_FRAME_DELAY	14000

#define SBUS_RANGE_MIN 200.0f
#define SBUS_RANGE_MAX 1800.0f

#define SBUS_TARGET_MIN 1000.0f
#define SBUS_TARGET_MAX 2000.0f

#define SBUS_MAX_VALUES  8

#define SBUS_PACKAGE_INTERVAL_TIME   3  /*ms*/
/* pre-calculate the floating point stuff as far as possible at compile time */
#define SBUS_SCALE_FACTOR ((SBUS_TARGET_MAX - SBUS_TARGET_MIN) / (SBUS_RANGE_MAX - SBUS_RANGE_MIN))
#define SBUS_SCALE_OFFSET (int)(SBUS_TARGET_MIN - (SBUS_SCALE_FACTOR * SBUS_RANGE_MIN + 0.5f))

void SBUS_Init(void);
void SBUS_Decode(uint8_t *frame, uint16_t *values, uint8_t *sbus_failsafe, uint8_t *sbus_frame_drop,
                 uint16_t max_values);
void SBUS_RX_InterruptHandler(void);
void SBUS_GetData(uint16_t *rc_pwm_in,uint8_t lenght);
#endif
