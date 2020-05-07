/*******************************************************************************
* 文件名称：mc_pos_control.c
* 摘    要：位置控制器实现，包含了高度方向的控制器，生成角度目标值给姿态控制器
* 当前版本：v1.0
* 作    者：across 
* 日    期：2020/02/27
* 编译环境：keil5
* 历史信息：
*******************************************************************************/

#include "mc_pos_control.h"
#include "mc_att_control.h"
#include "sensor.h"
#include "position_estimator_inav.h"
#include "attitude_estimator_mahony.h"
#include "math.h"
#include "rc_process.h"

//k1 高度环增益  k2 速度环增益  k3  加速度环增益
float Altitude_K1,Altitude_K2,Altitude_K3;
float Altitude_Ref = 0.0f,Altitude_VelRef = 0.0f,Takeoff_VelRef = 0.0f,ALTCONTROLLER_ControlU_Zero = 0.0f;

uint8_t Alt_State = Alt_Takeoff;
uint8_t Takeoff_Step = 0;


enum{
	Pos_Hold = 0,
	Pos_Changed	
};

uint8_t Pos_State = Pos_Hold;
float Vel_Ref_Y = 0.0f,Vel_Ref_X = 0.0f,Vel_Ref_Vn = 0.0f,Vel_Ref_Ve = 0.0f,Pos_Ref_N = 0.0f,Pos_Ref_E = 0.0f;
float u_vn_feedback_i = 0.0f;
float u_ve_feedback_i = 0.0f;
float pos_kp,vel_kp,vel_ki,vel_kd;
float pitchsp,rollsp,yawsp;
float N_Position_Zero = 0.0f,E_Position_Zero = 0.0f;
float Pos_ErrorN = 0.0f,Pos_ErrorE = 0.0f,Vel_ErrorN = 0.0f,Vel_ErrorE = 0.0f;


/*******************************函数声明****************************************
* 函数名称: void Altitude_control_Init(void)
* 输入参数: void
* 返回参数: void
* 功    能: 高度控制初始化
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Altitude_control_Init(void)
{

	Takeoff_VelRef = 0.0f;
	Altitude_VelRef = 0.0f;
	Altitude_Ref = Ctrl_state.z - Ctrl_state.Altitude_Ground;
	ALTCONTROLLER_ControlU_Zero = control_throttle;
	
	Altitude_K1 = 150.0f;
	Altitude_K2 = 180.0f; 
	Altitude_K3 = 20.0f;
}


/*******************************函数声明****************************************
* 函数名称: void Altitude_Reference(float saturation_max_controlu, float saturation_min_controlu)
* 输入参数: 高度控制限幅
* 返回参数: void
* 功    能: 高度控制的高度目标值与高度方向的速度目标值计算
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Altitude_Reference(float saturation_max_controlu, float saturation_min_controlu)
{
	if(Alt_State == Alt_Takeoff)
	{
		Altitude_VelRef = Takeoff_VelRef;
	}
	else if(Alt_State == Alt_Normal)
	{
		if(pRC_Normalization->Throttle > 0.15f)
		{
			Altitude_VelRef = (pRC_Normalization->Throttle - 0.15f)/(1.0f - 0.15f)*(MaxRiseVel);
		}
		else if(pRC_Normalization->Throttle < -0.15f)
		{
			Altitude_VelRef = (pRC_Normalization->Throttle + 0.15f)/(1.0f - 0.15f)*(MaxLandVel);
		}    
		else
		{
			Altitude_VelRef = 0.0f;
		}
	}

	if(Alt_State == Alt_Takeoff)
	{
		Altitude_Ref += Altitude_VelRef * 0.04f;
	}
	else if(Alt_State == Alt_Normal)
	{
		Altitude_Ref += Altitude_VelRef * 0.02f;
	}

	float Saturation_altitude_ref_max = (saturation_max_controlu - ALTCONTROLLER_ControlU_Zero)/(Altitude_K1)+(Ctrl_state.z - Ctrl_state.Altitude_Ground);
  float Saturation_altitude_ref_min = (saturation_min_controlu - ALTCONTROLLER_ControlU_Zero)/(Altitude_K1)+(Ctrl_state.z - Ctrl_state.Altitude_Ground);
  Altitude_Ref = Altitude_Ref > Saturation_altitude_ref_max ? Saturation_altitude_ref_max : Altitude_Ref;
  Altitude_Ref = Altitude_Ref < Saturation_altitude_ref_min ? Saturation_altitude_ref_min : Altitude_Ref;

}



/*******************************函数声明****************************************
* 函数名称: void Altitude_control_update(float saturation_max_controlu, float saturation_min_controlu)
* 输入参数: 高度控制限幅
* 返回参数: void
* 功    能: 高度控制计算
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Altitude_control_update(float saturation_max_controlu, float saturation_min_controlu)
{
  float Ueq;
	float Alt_Error = 0.0f,vel_error = 0.0f; 
	
	Altitude_Reference(saturation_max_controlu, saturation_min_controlu);
	
	float altitude_feedback = Ctrl_state.z - Ctrl_state.Altitude_Ground;
	Alt_Error = Altitude_Ref    - altitude_feedback;
	vel_error = Altitude_VelRef - Ctrl_state.vz;
	
	Ueq  = (Altitude_K2)*vel_error + (Altitude_K1)*Alt_Error + (Altitude_K3)*(-Ctrl_state.zacc);

  /*----------------------------- 控制量限幅 ---------------------------------*/  
  Ueq      = Ueq + ALTCONTROLLER_ControlU_Zero > saturation_max_controlu ? (saturation_max_controlu - ALTCONTROLLER_ControlU_Zero) : Ueq;
  Ueq      = Ueq + ALTCONTROLLER_ControlU_Zero < saturation_min_controlu ? (saturation_min_controlu - ALTCONTROLLER_ControlU_Zero) : Ueq;
  control_throttle    = Ueq + ALTCONTROLLER_ControlU_Zero;  
}





/*******************************函数声明****************************************
* 函数名称: void Velocity_Reference_Calculation(void)
* 输入参数: void
* 返回参数: void
* 功    能: 位置控制的速度目标值计算
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Velocity_Reference_Calculation(void)
{ 
	if(pRC_Normalization->Roll > VELOCITY_DEAD_ZONE_VALUE)
		Vel_Ref_Y= (pRC_Normalization->Roll - VELOCITY_DEAD_ZONE_VALUE) * MAX_XY_VEL;
	else if(pRC_Normalization->Roll < -VELOCITY_DEAD_ZONE_VALUE)
		Vel_Ref_Y= -(-(pRC_Normalization->Roll + VELOCITY_DEAD_ZONE_VALUE)) * MAX_XY_VEL;
	else
		Vel_Ref_Y=0.0f;

	if(pRC_Normalization->Pitch > VELOCITY_DEAD_ZONE_VALUE)
		Vel_Ref_X= (pRC_Normalization->Pitch - VELOCITY_DEAD_ZONE_VALUE) * MAX_XY_VEL;
	else if(pRC_Normalization->Pitch < -VELOCITY_DEAD_ZONE_VALUE)
		Vel_Ref_X= -(-(pRC_Normalization->Pitch + VELOCITY_DEAD_ZONE_VALUE)) * MAX_XY_VEL;
	else
		Vel_Ref_X=0.0f;


	Vel_Ref_Vn=( Vel_Ref_X*cos(attitude_yaw) + Vel_Ref_Y*sin(attitude_yaw));
	Vel_Ref_Ve=(-Vel_Ref_X*sin(attitude_yaw) + Vel_Ref_Y*cos(attitude_yaw));  
}


/*******************************函数声明****************************************
* 函数名称: void Pos_Control_Init(void)
* 输入参数: void
* 返回参数: void
* 功    能: 位置控制初始化，包括参数，积分清零，位置取零点
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Pos_Control_Init(void)
{

	Vel_Ref_Vn = 0.0f;
	Vel_Ref_Ve = 0.0f;
	Pos_Ref_N = 0.0f;
	Pos_Ref_E = 0.0f;
	Pos_State = Pos_Hold;
	u_vn_feedback_i = 0.0f;
  u_ve_feedback_i = 0.0f;
	N_Position_Zero = Ctrl_state.x;
	E_Position_Zero = Ctrl_state.y;
	
	pos_kp = 0.066f;
	vel_kp = 0.1f;
	vel_ki = 0.022f;
	vel_kd = 0.02f;
}

/*******************************函数声明****************************************
* 函数名称: static float nonlinear_p_caculate(float vel_error, float vel_kp)
* 输入参数: 速度误差与控制参数
* 返回参数: P控制量
* 功    能: 在小速度时，提供较大的控制量
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
static float nonlinear_p_caculate(float vel_error, float vel_kp)
{
  float breakpoint_vel = 0.5f;//阶梯p系数的拐点速度误差
  float param = 1.5f;//速度误差较小部分，p的放大倍数
  
  if( fabs(vel_error) <= breakpoint_vel )
    return (param*vel_error*vel_kp);
  else if(vel_error > breakpoint_vel)
    return (vel_error*vel_kp + (param-1.0f)*breakpoint_vel*vel_kp);
  else if(vel_error < -breakpoint_vel)
    return (vel_error*vel_kp - (param-1.0f)*breakpoint_vel*vel_kp);
  else
    return (vel_error*vel_kp);
}



/*******************************函数声明****************************************
* 函数名称: void Pos_Control(void)
* 输入参数: void
* 返回参数: void
* 功    能: 位置控制器
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Pos_Control(void)
{
		static float u_vn_feedback_p, u_vn_feedback_d, u_pn_feedback_p;
		static float u_ve_feedback_p, u_ve_feedback_d, u_pe_feedback_p;
		static uint8_t saturation_xy = 1;
		float u_vn_feedback, u_ve_feedback;

		if(Pos_State == Pos_Hold)
		{
			u_vn_feedback_i = 0.0f;
			u_ve_feedback_i = 0.0f;
			
			if(sqrt(Vel_Ref_Vn*Vel_Ref_Vn + Vel_Ref_Ve*Vel_Ref_Ve) > 0.05f)
			{
				Pos_State = Pos_Changed;
			}	
		}
		else if(Pos_State == Pos_Changed)
		{
			if(sqrt(Ctrl_state.vx*Ctrl_state.vx+Ctrl_state.vy*Ctrl_state.vy) < 0.5f 
       && sqrt(Vel_Ref_Vn*Vel_Ref_Vn + Vel_Ref_Ve*Vel_Ref_Ve) <= 0.02f)
			{
				Pos_State = Pos_Hold;
				
				u_vn_feedback_i = 0.0f;
				u_ve_feedback_i = 0.0f;
				
				Pos_Ref_N -= nonlinear_p_caculate(-Ctrl_state.vx, vel_kp) / pos_kp;
				Pos_Ref_E -= nonlinear_p_caculate(-Ctrl_state.vy, vel_kp) / pos_kp;
			}
		}	
		 
		
		if(Pos_State == Pos_Hold)
		{
			Pos_Ref_N += Vel_Ref_Vn*0.02f;
		}
		else if(Pos_State == Pos_Changed)
		{
			Pos_Ref_N = (Ctrl_state.x - N_Position_Zero);
		}      
    
    Pos_ErrorN = Pos_Ref_N  - (Ctrl_state.x - N_Position_Zero);
    Vel_ErrorN = Vel_Ref_Vn - Ctrl_state.vx;
    
    u_vn_feedback_p   = nonlinear_p_caculate(Vel_ErrorN, vel_kp);
		if(!saturation_xy && (Pos_State == Pos_Changed))
			u_vn_feedback_i += 0.02f * Vel_ErrorN * vel_ki;
		u_vn_feedback_d   = -Ctrl_state.xacc*vel_kd;
		u_pn_feedback_p   = Pos_ErrorN*pos_kp;  
		u_pn_feedback_p   = u_pn_feedback_p >  0.22f ?  0.22f : u_pn_feedback_p;
		u_pn_feedback_p   = u_pn_feedback_p < -0.22f ? -0.22f : u_pn_feedback_p;
		
		
		if(Pos_State == Pos_Hold)
		{
			Pos_Ref_E += Vel_Ref_Ve*0.02f;
		}
		else if(Pos_State == Pos_Changed)
		{
			Pos_Ref_E = (Ctrl_state.y - E_Position_Zero);
		}      
    
    Pos_ErrorE = Pos_Ref_E  - (Ctrl_state.y - E_Position_Zero);
    Vel_ErrorE = Vel_Ref_Ve - Ctrl_state.vy;
    
    u_ve_feedback_p   = nonlinear_p_caculate(Vel_ErrorE, vel_kp);
		if(!saturation_xy && (Pos_State == Pos_Changed))
			u_ve_feedback_i += 0.02f * Vel_ErrorE * vel_ki;
		u_ve_feedback_d   = -Ctrl_state.yacc*vel_kd;
		u_pe_feedback_p   = Pos_ErrorE*pos_kp;  
		u_pe_feedback_p   = u_pe_feedback_p >  0.22f ?  0.22f : u_pe_feedback_p;
		u_pe_feedback_p   = u_pe_feedback_p < -0.22f ? -0.22f : u_pe_feedback_p;
	
		u_vn_feedback        = u_vn_feedback_p + u_vn_feedback_i + u_vn_feedback_d;
		u_ve_feedback        = u_ve_feedback_p + u_ve_feedback_i + u_ve_feedback_d;
		
		float att_saturation = sqrt(u_vn_feedback*u_vn_feedback + u_ve_feedback*u_ve_feedback);
		if(att_saturation > 0.4f)
		{
			saturation_xy   = 1;
			u_vn_feedback   = u_vn_feedback*0.4f/att_saturation;
			u_ve_feedback   = u_ve_feedback*0.4f/att_saturation;
		}
		else
		{
			saturation_xy = 0;
		}
		
		u_vn_feedback        = u_vn_feedback + u_pn_feedback_p;
		u_ve_feedback        = u_ve_feedback + u_pe_feedback_p;
		
		pitchsp  = (u_vn_feedback*cos(attitude_yaw) - u_ve_feedback*sin(attitude_yaw));
		rollsp   = (u_vn_feedback*sin(attitude_yaw) + u_ve_feedback*cos(attitude_yaw)); 
		
		pitchsp  = (pitchsp >  0.4f) ?  0.4f : pitchsp;
		pitchsp  = (pitchsp < -0.4f) ? -0.4f : pitchsp;   
		rollsp   = (rollsp  >  0.4f) ?  0.4f : rollsp;
		rollsp   = (rollsp  < -0.4f) ? -0.4f : rollsp;   
		
}



/*******************************函数声明****************************************
* 函数名称: void Alt_Control_Takeoff_Init(void)
* 输入参数: void
* 返回参数: void
* 功    能: 起飞阶段的高度控制初始化
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/
void Alt_Control_Takeoff_Init(void)
{
	Takeoff_Step = 0;
	Alt_State = Alt_Takeoff;
	Altitude_control_Init();
	control_throttle = 0.0f;
}

/*******************************函数声明****************************************
* 函数名称: void Alt_Control_Takeoff(void)
* 输入参数: void
* 返回参数: void
* 功    能: 起飞阶段的高度控制状态切换
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/ 
void Alt_Control_Takeoff(void)
{                                
    switch (Takeoff_Step)
    {
        case 0:
					mc_att_control_init();
					Altitude_control_Init();
					Pos_Control_Init();
					control_throttle = 0.0f;
					Get_Altitude_Ground();

					if(pRC_Normalization->Throttle > 0.15f)
					{
						Takeoff_Step = 1;
						Altitude_Ref = Ctrl_state.z - Ctrl_state.Altitude_Ground;
					}
					
					Alt_State = Alt_Takeoff;
					
					break;
        
        case 1:
					if(pRC_Normalization->Throttle > 0.15f)
						Takeoff_VelRef = (pRC_Normalization->Throttle - 0.15f)/(1.0f - 0.15f)*(MaxRiseVel);
					else if(pRC_Normalization->Throttle < -0.15f)
						Takeoff_VelRef = (pRC_Normalization->Throttle + 0.15f)/(1.0f - 0.15f)*(MaxLandVel);
					else
						Takeoff_VelRef = 0.0f;
					
					if(Ctrl_state.vz > 0.15f)
					{
						Takeoff_VelRef = 0.0f;
						Takeoff_Step = 2;
					}
					
					Alt_State = Alt_Takeoff;
					
					break;
					
        case 2:
					Alt_State = Alt_Normal;

					if(pRC_Normalization->Throttle < 0.15f && pRC_Normalization->Throttle > -0.15f)
							Takeoff_Step = 3;
					
					break;
					
        case 3:
					Alt_State = Alt_Normal;
				break;
				
        default:
					break;
    };
    
    if( 2 == Takeoff_Step || 3 == Takeoff_Step )
    {
        Altitude_control_update(ALTCONTROL_MAX_THROTTLE, 0.0f);
				Velocity_Reference_Calculation();
				Pos_Control();
    }
    else if(Takeoff_Step >= 1)
    {
        Altitude_control_update(ALTCONTROL_MAX_THROTTLE, 0.0f);
				Attitude_Reference_caculation();
    }
}

/*******************************函数声明****************************************
* 函数名称: void Get_Altitude_Ground(void)
* 输入参数: void
* 返回参数: void
* 功    能: 在地面阶段，取估计高度的零点
* 作    者: by across 
* 日    期: 2020/02/27
*******************************************************************************/ 
void Get_Altitude_Ground(void)
{
  static uint8_t groud_cnt = 0;
  static float Sampling[20] = {0.0f};
  float  Alt_temp = 0.0f;
	
  Sampling[groud_cnt++]  = Ctrl_state.z;
  groud_cnt = groud_cnt > 19 ? 0 : groud_cnt;
  for (uint8_t j=0; j<20; j++)
  {
    Alt_temp  += Sampling[j];
  }
  Ctrl_state.Altitude_Ground  = (Alt_temp * 0.05f);
}



