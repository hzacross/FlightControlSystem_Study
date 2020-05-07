#ifndef __RC_PROCESS_H
#define __RC_PROCESS_H

#include "stm32f4xx_hal.h"
#include "md_struct.h"


#define RC_Dz    50


#define Nomalization_RC_Min  (-1.0f)
#define Nomalization_RC_Mid  ( 0.0f)
#define Nomalization_RC_Max  ( 1.0f)

#define Nomalization_Switch_Min  0
#define Nomalization_Switch_Mid  1
#define Nomalization_Switch_Max  2

void RCDataProcess(void);
extern d_RC *pRC_Normalization;

#endif

