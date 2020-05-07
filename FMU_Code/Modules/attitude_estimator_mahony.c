#include "attitude_estimator_mahony.h"
#include "sensor.h"
#include "math.h"


#define SPIN_RATE_LIMIT 20

static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; //机体坐标系相对地理坐标系的姿态

static float dcm_kp,dcm_ki;
static uint8_t attitude_estimator_init_flag = 0;

float attitude_roll,attitude_pitch,attitude_yaw;
float rMat[3][3];

static void imuComputeRotationMatrix(void)
{
    float q1q1 = sq(q1);
    float q2q2 = sq(q2);
    float q3q3 = sq(q3);

    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 - q0q3);
    rMat[0][2] = 2.0f * (q1q3 + q0q2);

    rMat[1][0] = 2.0f * (q1q2 + q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 - q0q1);

    rMat[2][0] = 2.0f * (q1q3 - q0q2);
    rMat[2][1] = 2.0f * (q2q3 + q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
}

void imuInit(void)
{
	dcm_kp = 0.25f;
	dcm_ki = 0.00f;
	
	//初始化四元数
  float initialRoll, initialPitch;
  float cosRoll, sinRoll, cosPitch, sinPitch;
  float magX, magY;
  float initialHdg, cosHeading, sinHeading;
  
  initialRoll = atan2f(-Ctrl_state.accf.y, -Ctrl_state.accf.z);//根据加速度算角度
  initialPitch = -atan2f(-Ctrl_state.accf.x, -Ctrl_state.accf.z);
  
  cosRoll = cosf(initialRoll);
  sinRoll = sinf(initialRoll);
  cosPitch = cosf(initialPitch);
  sinPitch = sinf(initialPitch);
  
  magX = (Ctrl_state.magf.x) * cosPitch + (Ctrl_state.magf.y) * sinRoll * sinPitch + (Ctrl_state.magf.z) * cosRoll * sinPitch;
  
  magY = (Ctrl_state.magf.y) * cosRoll - (Ctrl_state.magf.z) * sinRoll;
  
  initialHdg = atan2f(-magY,magX);
  
  cosRoll = cosf(initialRoll * 0.5f);
  sinRoll = sinf(initialRoll * 0.5f);
  
  cosPitch = cosf(initialPitch * 0.5f);
  sinPitch = sinf(initialPitch * 0.5f);
  
  cosHeading = cosf(initialHdg * 0.5f);
  sinHeading = sinf(initialHdg * 0.5f);
  
  q0 = (cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading);
  q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
  q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
  q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
	
	imuComputeRotationMatrix();
}

static float invSqrt(float x)
{
    return 1.0f / sqrtf(x);
}



static void imuMahonyAHRSupdate(float dt, float gx, float gy, float gz,
                                uint8_t useAcc, float ax, float ay, float az,
                                uint8_t useMag, float mx, float my, float mz,
                                uint8_t useYaw, float yawError)
{
    static float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;    
    float recipNorm;
    float hx, hy, bx;
    float ex = 0, ey = 0, ez = 0;
    float qa, qb, qc;

    float spin_rate = sqrtf(sq(gx) + sq(gy) + sq(gz));

    //GPS yaw 修正
    if (useYaw) {
        while (yawError >  3.141592f) yawError -= (2.0f * 3.141592f);
        while (yawError < -3.141592f) yawError += (2.0f * 3.141592f);

        ez += sinf(yawError / 2.0f);
    }

    //mag 修正
    recipNorm = sq(mx) + sq(my) + sq(mz);
    if (useMag && recipNorm > 0.01f) {
        // 归一化
        recipNorm = invSqrt(recipNorm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        //假设地磁与重力向量垂直，不与roll pitch 耦合
        hx = rMat[0][0] * mx + rMat[0][1] * my + rMat[0][2] * mz;
        hy = rMat[1][0] * mx + rMat[1][1] * my + rMat[1][2] * mz;
        bx = sqrtf(hx * hx + hy * hy);

        float ez_ef = -(hy * bx);

        // 地磁修正误差转换到机体坐标系
        ex += rMat[2][0] * ez_ef;
        ey += rMat[2][1] * ez_ef;
        ez += rMat[2][2] * ez_ef;
    }

    //acc 修正
    recipNorm = sq(ax) + sq(ay) + sq(az);
    if (useAcc && recipNorm > 0.01f) {
		
        recipNorm = invSqrt(recipNorm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        //与重力的误差
        ex += (ay * rMat[2][2] - az * rMat[2][1]);
        ey += (az * rMat[2][0] - ax * rMat[2][2]);
        ez += (ax * rMat[2][1] - ay * rMat[2][0]);
    }

    if(dcm_ki > 0.0f) {
		
        if (spin_rate < DEGREES_TO_RADIANS(SPIN_RATE_LIMIT)) {
            float dcmKiGain = dcm_ki;
            integralFBx += dcmKiGain * ex * dt;    
            integralFBy += dcmKiGain * ey * dt;
            integralFBz += dcmKiGain * ez * dt;
        }
    }
    else {
        integralFBx = 0.0f;    // 防止积分过饱和
        integralFBy = 0.0f;
        integralFBz = 0.0f;
    }

    float dcmKpGain = dcm_kp;

    //pi反馈修正误差
    gx += dcmKpGain * ex + integralFBx;
    gy += dcmKpGain * ey + integralFBy;
    gz += dcmKpGain * ez + integralFBz;

    gx *= (0.5f * dt);
    gy *= (0.5f * dt);
    gz *= (0.5f * dt);

    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    recipNorm = invSqrt(sq(q0) + sq(q1) + sq(q2) + sq(q3));
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    imuComputeRotationMatrix();
}


static void imuUpdateEulerAngles(void)
{
    attitude_roll = atan2f(rMat[2][1], rMat[2][2]);
    attitude_pitch = -asinf(rMat[2][0]);
    attitude_yaw = atan2f(rMat[1][0],rMat[0][0]);
}


static uint8_t imuIsAccelerometerHealthy(void)
{
    float accMagnitude = 0;

    accMagnitude = Ctrl_state.accf.x * Ctrl_state.accf.x + Ctrl_state.accf.y * Ctrl_state.accf.y + Ctrl_state.accf.z * Ctrl_state.accf.z;

    accMagnitude = accMagnitude / (sq(9.8f));

    //范围在0.80g - 1.20g
    return (0.8f < accMagnitude) && (accMagnitude < 1.2f);
}

static uint8_t isMagnetometerHealthy(void)
{
    return (Ctrl_state.magf.x != 0) && (Ctrl_state.magf.y != 0) && (Ctrl_state.magf.z != 0);
}

float testyaw = 0.0;

void imuCalculateEstimatedAttitude(void)
{
    float rawYawError = 0.0f;
    uint8_t useAcc = 0;
    uint8_t useMag = 0;
    uint8_t useYaw = 0;

		if((!attitude_estimator_init_flag) && (Ctrl_state.magf.x != 0))
    {
      imuInit();
      attitude_estimator_init_flag = 1;
      return;
    }
	
    if (imuIsAccelerometerHealthy()) {
        useAcc = 1;
    }


    if (isMagnetometerHealthy()) {
        useMag = 1;
    }

    imuMahonyAHRSupdate(0.005f,
                        Ctrl_state.gyrof.x, -Ctrl_state.gyrof.y, -Ctrl_state.gyrof.z,
                        useAcc, -Ctrl_state.accf.x, -Ctrl_state.accf.y, -Ctrl_state.accf.z, 
                        useMag, Ctrl_state.magf.x, Ctrl_state.magf.y, Ctrl_state.magf.z,
                        useYaw, rawYawError);

    imuUpdateEulerAngles();

		testyaw = atan2f(-Ctrl_state.magf.y, Ctrl_state.magf.x);
		
//		attitude_roll = atan2f(-Ctrl_state.accf.y, -Ctrl_state.accf.z);//根据加速度算角度
//  attitude_pitch = -atan2f(-Ctrl_state.accf.x, -Ctrl_state.accf.z);
//  
//  float cosRoll = cosf(attitude_roll);
//  float sinRoll = sinf(attitude_roll);
//  float cosPitch = cosf(attitude_pitch);
//  float sinPitch = sinf(attitude_pitch);
//  
//  float magX = (-Ctrl_state.magf.x) * cosPitch + (-Ctrl_state.magf.y) * sinRoll * sinPitch + (-Ctrl_state.magf.z) * cosRoll * sinPitch;
//  
//  float magY = (-Ctrl_state.magf.y) * cosRoll - (-Ctrl_state.magf.z) * sinRoll;
//  
//  attitude_yaw = atan2f(magY,-magX);
		
}


















