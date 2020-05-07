/*******************************************************************************
* 文件名称：md_struct.h
*
* 摘    要：自定义结构体
*
* 当前版本：
* 作    者：
* 日    期：2017/12/18
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#ifndef __MD_STRUCT_H
#define __MD_STRUCT_H

#include "stm32f1xx_hal.h"

/** 
  * @brief  LED Status enumeration 
  */
typedef enum
{
  LED_OFF = 0U,
  LED_ON
}LED_Status;

typedef struct  
{
  uint16_t motor1;
	uint16_t motor2;
	uint16_t motor3;
	uint16_t motor4;
	uint16_t motor5;
	uint16_t motor6;
}Motor_Value;

typedef union
{
  uint16_t value_16;
	uint8_t  value_8[2];
}UINT16_8BIT;

#define CONSTRAINT(in, min, max)  (in > max ? max : (in < min ? min : in))
#define ABS(a) ((a) > 0 ? (a) : -(a))
#define MIN(x, y) (x < y ? x : y)
#define MAX(x, y) (x > y ? x : y)

#endif
