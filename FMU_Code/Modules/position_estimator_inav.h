#ifndef __POSITION_ESTIMATOR_INAV_H
#define __POSITION_ESTIMATOR_INAV_H

#include "stm32f4xx_hal.h"

#define EST_BUF_SIZE  25
#define EST_INTERVAL  10000             
#define GPS_DELAY     0.15f          

#define MIN_VALID_W  0.00001f

#define RAD2DEG     57.2957795130823f
#define DEG2RAD     0.01745329251994f
#define EPSILON       1.1920929e-07f
#define CONSTANTS_RADIUS_OF_EARTH   6371000	

#define CONSTANTS_ONE_G					9.80665f	

//data structures
struct map_projection_reference_s {
	double lat_rad;
	double lon_rad;
	double sin_lat;
	double cos_lat;
};


struct pos_est_param_s{
	//pos weight
	float w_xy_gps_p;
	float w_z_gps_p;
	//vel weight
	float w_xy_gps_v;
	float w_z_gps_v;
	
	float w_acc_bias;
	float w_z_baro;
	
	float w_xy_res_v;
};

struct home_position_s {
	uint64_t timestamp;
	double lat;
	double lon;
	float alt;
	float x;
	float y;
	float z;
};



//functions
void position_estimator_inav_init(void);
void position_estimator_inav_update(void);
void inertial_filter_predict(float dt, float x[2], float acc);
void inertial_filter_correct(float e, float dt, float x[2], int i, float w);
extern struct local_position_s local_pos;
extern float gps_ref[2];
extern double est_lat,est_lon;

#endif 
