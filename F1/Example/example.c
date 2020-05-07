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

/*******************************函数声明****************************************
* 函数名称: void Example_RCin_PWMout_Direct(void)
* 输入参数: void
* 返回参数: void
* 功    能: 直接将获取的遥控器油门值输出到PWM口，测试PWM是否正确。
* 作    者: ACROSS
* 日    期: 2018.05.15
*******************************************************************************/
void Example_RCin_PWMout_Direct(void)
{
	/*获取油门通道的值 1094---1933*/
	
  uint16_t m_throttle = rc_pwm_in[2];
	PWM_Set_Value_Channel_1(m_throttle);
	PWM_Set_Value_Channel_2(m_throttle);
	PWM_Set_Value_Channel_3(m_throttle);
	PWM_Set_Value_Channel_4(m_throttle);
	PWM_Set_Value_Channel_5(m_throttle);
	PWM_Set_Value_Channel_6(m_throttle);
	PWM_Set_Value_Channel_7(m_throttle);
	PWM_Set_Value_Channel_8(m_throttle);
}


extern __IO ITStatus rc_newdata_flag;
extern uint16_t rc_pwm_in[RC_INPUT_CHANNELS];
extern Motor_Value m_Motor_From_F4;
void Example_Exchage_COM(void)
{	
	/*1.发送遥控器数据给F4*/
	if(rc_newdata_flag==SET)
	{
	  rc_newdata_flag=RESET;
		Com_TwoBoard_Msg_RCinput_Send(rc_pwm_in);
		Com_TwoBoard_Msg_HeartBeat_Send(Status_Safety);
	}
	/*2.接收F4返回的PWM数据*/
	Com_TwoBoard_RB_Update();
	/*3.控制电机转动*/
	PWM_Set_Value_Channel_1(m_Motor_From_F4.motor1);
	PWM_Set_Value_Channel_2(m_Motor_From_F4.motor2);
	PWM_Set_Value_Channel_3(m_Motor_From_F4.motor3);
	PWM_Set_Value_Channel_4(m_Motor_From_F4.motor4);
	
}
