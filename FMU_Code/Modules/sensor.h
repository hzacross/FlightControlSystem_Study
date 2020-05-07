#ifndef __SENSOR_H
#define __SENSOR_H

#include "stm32f4xx_hal.h"
#include "md_struct.h"


#define MPU6000_ONE_G					9.80665f
#define GYRO_CutoffFreq       30
#define ACC_CutoffFreq        30
#define IMU_Sample_Hz         500

void Filter_init(void);
void Transform(float Aframex,float Aframey,float Aframez,float *Bframex,float *Bframey,float *Bframez);
void IMU_Data_Combine(void); 
void Get_Gyro_Offset(void);
void mag_calibration(void);
float Get_Baro_Alt(float P);

extern Control_state  Ctrl_state;
extern float BARO_Alt;
extern float pitch_rate_filtered,roll_rate_filtered,yaw_rate_filtered;

#endif

