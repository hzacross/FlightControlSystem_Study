#ifndef __ATTITUDE_ESTIMATOR_MAHONY_H
#define __ATTITUDE_ESTIMATOR_MAHONY_H

#include "stm32f4xx_hal.h"
#include "md_struct.h"

static void imuComputeRotationMatrix(void);
void imuInit(void);
static float invSqrt(float x);

static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                uint8_t useAcc, float ax, float ay, float az,
                                uint8_t useMag, float mx, float my, float mz,
                                uint8_t useYaw, float yawError);
static void imuUpdateEulerAngles(void);
static uint8_t imuIsAccelerometerHealthy(void);
static uint8_t isMagnetometerHealthy(void);
void imuCalculateEstimatedAttitude(void);

extern float attitude_roll,attitude_pitch,attitude_yaw;			
extern float rMat[3][3];																
																
#endif

