
/*******************************************************************************
* 文件名称：rc_process.c
* 摘    要：遥控器数据预处理，归一化
* 当前版本：v1.0
* 作    者：across 
* 日    期：2020/02/27
* 编译环境：keil5
* 历史信息：
*******************************************************************************/

#include "rc_process.h"


extern RC_Channel m_RC_input_from_F1;

d_RC RC_Normalization;
d_RC *pRC_Normalization =&RC_Normalization;

void RCDataProcess(void)
{
	m_RC_input_from_F1.channel1 = CONSTRAINT(m_RC_input_from_F1.channel1,1000,2000);//roll
	m_RC_input_from_F1.channel2 = CONSTRAINT(m_RC_input_from_F1.channel2,1000,2000);//
	m_RC_input_from_F1.channel3 = CONSTRAINT(m_RC_input_from_F1.channel3,1000,2000);
	m_RC_input_from_F1.channel4 = CONSTRAINT(m_RC_input_from_F1.channel4,1000,2000);

	uint16_t RC_Max = 1939;
  uint16_t RC_Mid = 1514;
  uint16_t RC_Min = 1087; 
	
	uint16_t SWITCH_Max = 1939;
  uint16_t SWITCH_Mid = 1514;
  uint16_t SWITCH_Min = 1087; 


	pRC_Normalization->Roll     = ((float)m_RC_input_from_F1.channel1 - RC_Mid) / (RC_Max - RC_Min) * 2.0f;
	pRC_Normalization->Pitch    = ((float)m_RC_input_from_F1.channel2 - RC_Mid) / (RC_Max - RC_Min) * 2.0f;
	pRC_Normalization->Throttle = ((float)m_RC_input_from_F1.channel3 - RC_Mid) / (RC_Max - RC_Min) * 2.0f;
	pRC_Normalization->Yaw      = ((float)m_RC_input_from_F1.channel4 - RC_Mid) / (RC_Max - RC_Min) * 2.0f;
	
	if(pRC_Normalization->Roll >  1.0f)     pRC_Normalization->Roll =  1.0f;
	if(pRC_Normalization->Roll < -1.0f)     pRC_Normalization->Roll = -1.0f;
	if(pRC_Normalization->Pitch >  1.0f)    pRC_Normalization->Pitch =  1.0f;
	if(pRC_Normalization->Pitch < -1.0f)    pRC_Normalization->Pitch = -1.0f;
	if(pRC_Normalization->Throttle >  1.0f) pRC_Normalization->Throttle =  1.0f;
	if(pRC_Normalization->Throttle < -1.0f) pRC_Normalization->Throttle = -1.0f;
	if(pRC_Normalization->Yaw >  1.0f)      pRC_Normalization->Yaw =  1.0f;
	if(pRC_Normalization->Yaw < -1.0f)      pRC_Normalization->Yaw = -1.0f;
	
	if((m_RC_input_from_F1.channel5 > SWITCH_Min - 20) && (m_RC_input_from_F1.channel5 < SWITCH_Min + 20))
    pRC_Normalization->Switch_A = 0;
  else if((m_RC_input_from_F1.channel5 > SWITCH_Mid - 20) && (m_RC_input_from_F1.channel5 < SWITCH_Mid + 20))
    pRC_Normalization->Switch_A = 1;
  else if((m_RC_input_from_F1.channel5 > SWITCH_Max - 20) && (m_RC_input_from_F1.channel5 < SWITCH_Max + 20))
    pRC_Normalization->Switch_A = 2;
	
	if((m_RC_input_from_F1.channel6 > SWITCH_Min - 20) && (m_RC_input_from_F1.channel6 < SWITCH_Min + 20))
    pRC_Normalization->Switch_B = 0;
  else if((m_RC_input_from_F1.channel6 > SWITCH_Mid - 20) && (m_RC_input_from_F1.channel6 < SWITCH_Mid + 20))
    pRC_Normalization->Switch_B = 1;
  else if((m_RC_input_from_F1.channel6 > SWITCH_Max - 20) && (m_RC_input_from_F1.channel6 < SWITCH_Max + 20))
    pRC_Normalization->Switch_B = 2;
	
	if((m_RC_input_from_F1.channel7 > SWITCH_Min - 20) && (m_RC_input_from_F1.channel7 < SWITCH_Min + 20))
    pRC_Normalization->Switch_C = 0;
  else if((m_RC_input_from_F1.channel7 > SWITCH_Mid - 20) && (m_RC_input_from_F1.channel7 < SWITCH_Mid + 20))
    pRC_Normalization->Switch_C = 1;
  else if((m_RC_input_from_F1.channel7 > SWITCH_Max - 20) && (m_RC_input_from_F1.channel7 < SWITCH_Max + 20))
    pRC_Normalization->Switch_C = 2;
	
	if((m_RC_input_from_F1.channel8 > SWITCH_Min - 20) && (m_RC_input_from_F1.channel8 < SWITCH_Min + 20))
    pRC_Normalization->Switch_D = 0;
  else if((m_RC_input_from_F1.channel8 > SWITCH_Mid - 20) && (m_RC_input_from_F1.channel8 < SWITCH_Mid + 20))
    pRC_Normalization->Switch_D = 1;
  else if((m_RC_input_from_F1.channel8 > SWITCH_Max - 20) && (m_RC_input_from_F1.channel8 < SWITCH_Max + 20))
    pRC_Normalization->Switch_D = 2;
		
}




