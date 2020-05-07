#ifndef __MC_POS_CONTROL_H
#define __MC_POS_CONTROL_H

#include "stm32f4xx_hal.h"

#define  MaxRiseVel   (2.0f)
#define  MaxLandVel   (1.0f)
#define  ALTCONTROL_MAX_THROTTLE     (1200.0f)
#define  ALTCONTROL_MIN_THROTTLE     (150.0f)
#define  MAX_XY_VEL    (6.0f)
#define  VELOCITY_DEAD_ZONE_VALUE    (0.15f)

void Altitude_control_Init(void);
void Altitude_Reference(float saturation_max_controlu, float saturation_min_controlu);
void Altitude_control_update(float saturation_max_controlu, float saturation_min_controlu);
void Velocity_Reference_Calculation(void);
void Pos_Control_Init(void);
static float nonlinear_p_caculate(float vel_error, float vel_kp);
void Pos_Control(void);
void Alt_Control_Takeoff_Init(void);
void Alt_Control_Takeoff(void);
void Get_Altitude_Ground(void);


enum{
	Alt_Takeoff = 0,
	Alt_Normal,	
};

extern float pitchsp,rollsp,yawsp,Takeoff_VelRef;
extern uint8_t Alt_State;

#endif 
