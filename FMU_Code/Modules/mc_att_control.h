#ifndef __MC_ATT_CONTROL_H
#define __MC_ATT_CONTROL_H

#include "stm32f4xx_hal.h"

#define AttiLimit   30
#define RefYawrate  60
#define MOTOR_MIN_POINT        1000
#define MOTOR_STARTING_POINT   (1000 + 8*10)
#define MOTOR_MAX_POINT        1990 



void mc_rate_control(void);
void Attitude_Reference_caculation(void);
void mc_roll_pitch_control(void);
void Yaw_Reference_Calculation(float dt);
void mc_yaw_control(float dt);
void mc_yaw_Init(void);
void mc_att_control_init(void);
void Ctr2Motor(void);
void reset_integrator(void);

extern float control_pitch,control_roll,control_yaw,control_throttle;

#endif 
