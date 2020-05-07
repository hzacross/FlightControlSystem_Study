/*******************************************************************************
* 文件名称：example.c
*
* 摘    要：测试程序
*
* 当前版本：
* 作    者：ACROSS
* 日    期：2018/05/24
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#include "example.h"
#include "md_include.h"
#include "md_struct.h"
#include "mc_att_control.h"

extern __IO ITStatus f_NewRCDataReady;
extern RC_Channel m_RC_input_from_F1;
extern d_Motor *pMotor;
extern d_RC *pRC_Normalization;

uint16_t temp = 0;

void Example_Control(RC_Channel*rc_input)
{
	temp = (uint16_t)((pRC_Normalization->Throttle + 1.0f)*500.0f) + MOTOR_MIN_POINT;//百分之30-80
  temp = temp > 2000 ? 2000 : temp;
  temp = temp < MOTOR_MIN_POINT ? MOTOR_MIN_POINT : temp;
	
  pMotor->M1 = temp;
	pMotor->M2 = temp;
	pMotor->M3 = temp;
	pMotor->M4 = temp;
	pMotor->M5 = temp;
	pMotor->M6 = temp;
}

void Example_Exchage_COM(void)
{	
  /*1.获取遥控器的通道值*/
	  Com_TwoBoard_RB_Update();
	/*2.进行控制运算*/
	if(f_NewRCDataReady==SET)
	{
	  f_NewRCDataReady = RESET;
		Example_Control(&m_RC_input_from_F1);
	  /*3.输出电机的控制值给F1*/
	 Com_TwoBoard_Msg_MotorValue_Send(pMotor);
	}
	 
}
