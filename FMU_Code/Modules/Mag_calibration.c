/*
******************************************************************************
* @file    Mag_calibration.c 
* @author  
* @version V1.0.0
* @date    2018-10-22
* @brief   地磁静态校准
* @freq    50hz
******************************************************************************/

#include "Mag_calibration.h"
#include "sensor.h"
#include "scheduler.h"
#include "md_led_driver.h"
#include "drv_flash.h"
#include <string.h>

uint8_t Mag_Calibration_Instrction_Step = 0;

float magxoffset = 0.0f,magyoffset = 0.0f,magzoffset = 0.0f;

void Mag_Calibration_update(void)
{
	Mag_Calibration_Instrction();
	Mag_Calibration_Operation();
}

/*------------------------------------------------------------------------------ 
地磁校准步骤指令更新
STEP0：校准初始化，待切换遥控器开关进入地磁校准模式，STEP=1；
STEP1：
------------------------------------------------------------------------------*/
void Mag_Calibration_Instrction(void)
{
	static float circlingcngle_roll=0.0f, circlingangle_pitch=0.0f, circlingangle_yaw=0.0f;
	static uint8_t direectionZ_magcali_cnt;
	static uint8_t delay_1sec = 0;
	
	switch(Mag_Calibration_Instrction_Step)
	{
		case 0://未进入校准地磁状态，角度计算初始化
			circlingangle_yaw   = 0.0f;
			circlingcngle_roll  = 0.0f;
			circlingangle_pitch = 0.0f;
			delay_1sec = 0;    
			direectionZ_magcali_cnt = 0;

		break;
		
		case 1://进入地磁校准状态，校准XY轴地磁
			circlingangle_yaw += Ctrl_state.gyrof.z * 0.02f;
			if( fabs(circlingangle_yaw) > 3.0f*M_PI_F)
			  Mag_Calibration_Instrction_Step = 2;
			set_mRed_State(Red_Slow_Flashing);
		break;
		
		case 2://XY平面地磁校准完成，将飞机立起，初始化Roll、Pitch
			circlingcngle_roll  = 0.0f;
			circlingangle_pitch = 0.0f;
			if(( Ctrl_state.accf.z > -2.0f )&&( Ctrl_state.accf.z < 2.0f ))
			  direectionZ_magcali_cnt++;
			else
			  direectionZ_magcali_cnt = 0;
			if(direectionZ_magcali_cnt >= 50)
			  Mag_Calibration_Instrction_Step = 3;
			set_mRed_State(Red_Fast_Flashing);
		break;
		
		case 3://飞机立起到位，校准Z轴地磁
			circlingcngle_roll  += Ctrl_state.gyrof.x*0.02f;
			circlingangle_pitch += Ctrl_state.gyrof.x*0.02f;
			if( fabs(circlingcngle_roll) > 3.0f*M_PI_F || fabs(circlingangle_pitch) > 3.0f*M_PI_F)
			  Mag_Calibration_Instrction_Step = 4;
			set_mRed_State(Red_Slow_Flashing);
			break;
			
		case 4://Z轴地磁校准完成，停止地磁校准,等待反馈结果
			delay_1sec++;
			if(delay_1sec >=50 && isCali_XYFactor_Check_Succeed())
			{
			  Mag_Calibration_Instrction_Step = 5;
			  set_System_State(INIT_STATE);
			}
			else if(delay_1sec >=50 && !isCali_XYFactor_Check_Succeed())
			  Mag_Calibration_Instrction_Step = 6;
			set_mRed_State(Red_Fast_Flashing);
		break;
		case 5://退出地磁校准阶段（来回拨动开关），LED灯指示校准结果是否正常
			delay_1sec = 0;
			  Mag_Calibration_Instrction_Step = 7;
		break;
		
		case 6://等待飞机复位放平，等待起飞准备完成
			Mag_Calibration_Instrction_Step = 0;
		break;
		
		case 7://飞机复位放平，起飞准备完成，状态归零，IMU初始化
			if(delay_1sec++ >= 50)
			{
			  Mag_Calibration_Instrction_Step = 0;
			}
			set_mRed_State(Red_OFF_Light);
		break;
		default:
			Mag_Calibration_Instrction_Step = 0;
	};
}

uint8_t get_Mag_Calibration_Instrction_Step(void)
{
  return Mag_Calibration_Instrction_Step;
}

//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t Mag_Calibration_State = 0;
float HMC5983_maxx,HMC5983_maxy,HMC5983_maxz;
float HMC5983_minx,HMC5983_miny,HMC5983_minz;

void Mag_Calibration_Operation(void)
{
  switch(Mag_Calibration_Instrction_Step)
  {
  case 0:
    Mag_calibration_init();
    break;
  case 1:
    Traverse_magvalue_XY();  
    break;
  case 2:
    Cali_parameter_caculation_XY();
    break;
  case 3:
    Traverse_magvalue_Z();
    break;
  case 4:
    Cali_parameter_caculation_Z();
    if(isCali_XYFactor_Check_Succeed())
    {
      Save_Cali_parameter();
      Mag_Calibration_State = 0;
    }
    else
    {
      Reset_Mag_calibration();
      Mag_Calibration_State = 1;
    }
    break;
  default:
    break;
  };
}


void Mag_calibration_init(void)
{
  Mag_calibration_init_XY();
  Mag_calibration_init_Z();
}
void Mag_calibration_init_XY(void)
{
  HMC5983_maxx = -0.2;
  HMC5983_maxy = -0.2; 
  HMC5983_minx = 0.2;
  HMC5983_miny = 0.2;
}
void Mag_calibration_init_Z(void)
{
  HMC5983_maxz = -0.2; 
  HMC5983_minz = 0.2;
}

void Traverse_magvalue_XY(void)
{
  if(HMC5983_minx > Ctrl_state.magf.x)               HMC5983_minx = Ctrl_state.magf.x;
  if(HMC5983_miny > Ctrl_state.magf.y)               HMC5983_miny = Ctrl_state.magf.y;
  if(HMC5983_maxx < Ctrl_state.magf.x)               HMC5983_maxx = Ctrl_state.magf.x;
  if(HMC5983_maxy < Ctrl_state.magf.y)               HMC5983_maxy = Ctrl_state.magf.y;
}

void Cali_parameter_caculation_XY(void)
{
  magxoffset = (HMC5983_maxx+HMC5983_minx)/2;
  magyoffset = (HMC5983_maxy+HMC5983_miny)/2;
}

uint8_t isCali_XYFactor_Check_Succeed(void)
{      
  float magx_range,magy_range;
  magx_range = HMC5983_maxx-HMC5983_minx;
  magy_range = HMC5983_maxy-HMC5983_miny; 
  if(magx_range==0) return 0;
  if(magy_range==0) return 0;
  float MAGXY_factor = magx_range/magy_range;
  if(fabs(MAGXY_factor - 1.0f) <= 0.2f)  
    return 1;
  else
    return 0;
}

void Traverse_magvalue_Z(void)
{
  if(HMC5983_minz > Ctrl_state.magf.z)                 HMC5983_minz = Ctrl_state.magf.z;
  if(HMC5983_maxz < Ctrl_state.magf.z)                 HMC5983_maxz = Ctrl_state.magf.z;    
}

void Cali_parameter_caculation_Z(void)
{
  magzoffset = (HMC5983_maxz+HMC5983_minz)/2;
}

uint8_t isCali_XZFactor_Check_Succeed(void)
{      
  float magx_range,magz_range;
  magx_range = HMC5983_maxx-HMC5983_minx;
  magz_range = HMC5983_maxz-HMC5983_minz; 
  if(magx_range==0) return 0;
  if(magz_range==0) return 0;
  float MAGXZ_factor = magx_range/magz_range;
  if(fabs(MAGXZ_factor - 1.0f) <= 0.2f)  
    return 1;
  else
    return 0;
}



void Save_Cali_parameter(void)
{
	//保存校准的参数
	uint32_t magcailData[3];
	magcailData[0] = (uint32_t)(magxoffset * 1000);
	magcailData[1] = (uint32_t)(magyoffset * 1000);
	magcailData[2] = (uint32_t)(magzoffset * 1000);
	
	STMFLASH_Write(ADDR_FLASH_SECTOR_11+4,magcailData,3);
}

void Reset_Mag_calibration(void)
{ 
	
}
void Get_calibration_offset(void)
{
	//上电初始化后，获取之前校准并保存的参数
	uint32_t magcailData[3];
	STMFLASH_Read(ADDR_FLASH_SECTOR_11+4,magcailData,3);
	magxoffset = magcailData[0] / 1000.0f;
	magyoffset = magcailData[1] / 1000.0f;
	magzoffset = magcailData[2] / 1000.0f;
}



