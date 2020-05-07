/*
  ******************************************************************************
  * @file    position_estimator_inav.c
  * @author  across 
  * @version v1.0
  * @date    2020-02-27
  * @brief   位置估计，得到三轴的速度，位置
 ******************************************************************************
*/

#include "position_estimator_inav.h"
#include "attitude_estimator_mahony.h"
#include "sensor.h"
#include "math.h"
#include <string.h>
#include "md_ubx.h"

float x_est[2] = { 0.0f, 0.0f };	// 位置，速度
float y_est[2] = { 0.0f, 0.0f };	// 位置，速度
float z_est[2] = { 0.0f, 0.0f };	// 位置，速度

float est_buf[EST_BUF_SIZE][3][2];	// 位置估计缓冲器
float R_buf[EST_BUF_SIZE][3][3];	// 旋转矩阵缓冲器				
float R_gps[3][3];          // GPS校正旋转矩阵

int buf_ptr = 0;

static const float min_eph_epv = 2.0f;	// 权重计算使用
static const float max_eph_epv = 20.0f;	

float eph = 20.0f;
float epv = 1.0f;

float x_est_prev[2], y_est_prev[2], z_est_prev[2];
double est_lat,est_lon;

float baro_offset = 0.0f;
uint8_t baro_init_cnt = 0;
uint8_t baro_init_num = 200;

struct map_projection_reference_s ref;

uint8_t gps_valid = 0;
uint8_t ref_inited = 0;

float gps_ref[2],acczlp;

float params_w_xy_gps_p,params_w_z_gps_p,params_w_xy_gps_v,params_w_z_gps_v,params_w_z_baro,params_w_acc_bias,params_w_xy_res_v;

/* corr */
float accel_ned[] = {0.0f, 0.0f, 0.0f};
float acc_bias[] = { 0.0f, 0.0f, 0.0f };
float corr_baro = 0.0f;		// D
float corr_gps[3][2] = {
	{ 0.0f, 0.0f },		// N (pos, vel)
	{ 0.0f, 0.0f },		// E (pos, vel)
	{ 0.0f, 0.0f },		// D (pos, vel)
};
float w_gps_xy = 1.0f;
float w_gps_z = 1.0f;

uint8_t ref_init_count = 0;
uint8_t inav_init = 0;



float imu_acc[3];
extern float Baro_alt,barolp;


/*
 * 函数名：inertial_filter_predict
 * 功能：一次预测
 */
void inertial_filter_predict(float dt, float x[2], float acc)
{
	if (isfinite(dt)) {
		if (!isfinite(acc)) {
			acc = 0.0f;
		}

		x[0] += x[1] * dt + acc * dt * dt / 2.0f;
		x[1] += acc * dt;
	}
}

/*
 * 函数名：inertial_filter_correct
 * 功能：一次校正
 */
void inertial_filter_correct(float e, float dt, float x[2], int i, float w)
{
	if (isfinite(e) && isfinite(w) && isfinite(dt)) {
		float ewdt = e * w * dt;
		x[i] += ewdt;

		if (i == 0) {
			x[1] += w * w * ewdt;
		}
	}
}

/* 初始化参考位置 */
int map_projection_init(struct map_projection_reference_s *ref, double lat_0, double lon_0) 
{
	ref->lat_rad = lat_0 * DEG2RAD;
	ref->lon_rad = lon_0 * DEG2RAD;
	ref->sin_lat = sin(ref->lat_rad);
	ref->cos_lat = cos(ref->lat_rad);
  
	return 0;
}

/*
 * 函数功能：由参考点及当前位置的经纬度得到当前点与参考点的距离
 * 传入参数：经度和纬度及参考点
 * 传出参数：相对距离坐标
 */
int map_projection_project(const struct map_projection_reference_s *ref, double lat, double lon, float *x, float *y)
{
	double lat_rad = lat * DEG2RAD;
	double lon_rad = lon * DEG2RAD;

	double sin_lat = sin(lat_rad);
	double cos_lat = cos(lat_rad);
	double cos_d_lon = cos(lon_rad - ref->lon_rad);

	double c = acos(ref->sin_lat * sin_lat + ref->cos_lat * cos_lat * cos_d_lon);
	double k = (fabs(c) < EPSILON) ? 1.0 : (c / sin(c));

	*x = k * (ref->cos_lat * sin_lat - ref->sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
	*y = k * cos_lat * sin(lon_rad - ref->lon_rad) * CONSTANTS_RADIUS_OF_EARTH;

	return 0;
}

/*
 * 函数功能：由参考点及当前位置的相对坐标得到经纬度
 * 传入参数：相对距离坐标及参考点
 * 传出参数：经度和纬度
 */
int map_projection_reproject(const struct map_projection_reference_s *ref, float x, float y, double *lat, double *lon)
{
	double x_rad = x / CONSTANTS_RADIUS_OF_EARTH;
	double y_rad = y / CONSTANTS_RADIUS_OF_EARTH;
	double c = sqrtf(x_rad * x_rad + y_rad * y_rad);
	double sin_c = sin(c);
	double cos_c = cos(c);

	double lat_rad;
	double lon_rad;

	if (fabs(c) > EPSILON) {
		lat_rad = asin(cos_c * ref->sin_lat + (x_rad * sin_c * ref->cos_lat) / c);
		lon_rad = (ref->lon_rad + atan2(y_rad * sin_c, c * ref->cos_lat * cos_c - x_rad * ref->sin_lat * sin_c));

	} else {
		lat_rad = ref->lat_rad;
		lon_rad = ref->lon_rad;
	}

	*lat = lat_rad * RAD2DEG;
	*lon = lon_rad * RAD2DEG;

	return 0;
}


/*
 * 功能：位置估计器初始化
 */
void position_estimator_inav_init(void)
{
	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));

	params_w_xy_gps_p = 1.0;
	params_w_z_gps_p = 0.00001;
	params_w_xy_gps_v = 2.0;
	params_w_z_gps_v = 1.8;
	params_w_z_baro = 0.5; 
	params_w_acc_bias = 0.05;
	params_w_xy_res_v = 0.5; 
        
	ref_inited = 0;
    
	for (uint8_t i = 0; i < 100; i++) {
		baro_offset += BARO_Alt; 
	}
	baro_offset *= 0.01f;

	memset(est_buf, 0, sizeof(est_buf));
	memset(R_buf, 0, sizeof(R_buf));
	memset(R_gps, 0, sizeof(R_gps));
	memset(x_est_prev, 0, sizeof(x_est_prev));
	memset(y_est_prev, 0, sizeof(y_est_prev));
	memset(z_est_prev, 0, sizeof(z_est_prev));
	memset(&ref, 0, sizeof(ref));
}

/*
 * 功能：位置估计更新，每更新完一次姿态，调用一次此函数
 */
void position_estimator_inav_update(void)
{   
	if((!inav_init) && (BARO_Alt != 0))
	{
		position_estimator_inav_init();
		inav_init = 1;
	}
	
	
	//修正加速度计
	imu_acc[0] = Ctrl_state.accf.x;
	imu_acc[1] = Ctrl_state.accf.y;
	imu_acc[2] = Ctrl_state.accf.z;

	imu_acc[0] -= acc_bias[0];  
	imu_acc[1] -= acc_bias[1];
	imu_acc[2] -= acc_bias[2];
	
	//转换到NED坐标系
	for (uint8_t i = 0; i < 3; i++) {
		accel_ned[i] = 0.0f;
		for (uint8_t j = 0; j < 3; j++) {
			accel_ned[i] += rMat[j][i] * imu_acc[j];
		}
	}
	accel_ned[2] += CONSTANTS_ONE_G;

	corr_baro = baro_offset - barolp - z_est[0];    

	uint8_t reset_est = 0;
        
	if(gps_valid)
	{
			if (_gps_position->eph > 0.7f * max_eph_epv || _gps_position->epv > 0.7f * max_eph_epv) {
					gps_valid = 0;
			}
	}
	else
	{
			if ((_gps_position->eph < max_eph_epv * 0.4f) && (_gps_position->epv < max_eph_epv * 0.4f)
				&& _gps_position->satellites_used >= 8 ) {
					gps_valid = 1;
					reset_est = 1;
			}
	}
                
	if (gps_valid) {
	
		double lat = _gps_position->lat * 1e-7;;
		double lon = _gps_position->lon * 1e-7;;
		float  alt = _gps_position->alt * 1e-3;
		
		//如果没有初始化参考位置，则初始化参考位置
		if (!ref_inited) {
				ref_init_count++;
				if (ref_init_count > 20) {    //延时1s，进行初始位置计算
						ref_inited = 1;
						
						x_est[0] = 0.0f;
						x_est[1] = _gps_position->vel_n_m_s;   
						y_est[0] = 0.0f;
						y_est[1] = _gps_position->vel_e_m_s; 
						
						Ctrl_state.ref_lat = lat;
						Ctrl_state.ref_lon = lon;
						Ctrl_state.ref_alt = alt + z_est[0];
						
						//初始化参考位置
						map_projection_init(&ref, lat, lon);
				}
		}
		
		if(ref_inited)
		{
			float gps_proj[2];
			map_projection_project(&ref, lat, lon, &gps_proj[0], &gps_proj[1]);
			
			gps_ref[0] = gps_proj[0];
			gps_ref[1] = gps_proj[1];
			
			//当GPS信号质量很好的时候，重置估计器
			if (reset_est) {
					x_est[0] = gps_proj[0];
					x_est[1] = _gps_position->vel_n_m_s;
					y_est[0] = gps_proj[1];
					y_est[1] = _gps_position->vel_e_m_s;
			}
			
			/*计算位置估计的GPS修正值*/
			int est_i = buf_ptr - 1 - MIN(EST_BUF_SIZE - 1, MAX(0, (int)(GPS_DELAY * 1000000.0f / EST_INTERVAL)));
			
			if (est_i < 0) est_i += EST_BUF_SIZE;
			
			/* calculate correction for position */
			corr_gps[0][0] = gps_proj[0] - est_buf[est_i][0][0];
			corr_gps[1][0] = gps_proj[1] - est_buf[est_i][1][0];
			corr_gps[2][0] = (Ctrl_state.ref_alt - alt) - est_buf[est_i][2][0];

			/* calculate correction for velocity */
			corr_gps[0][1] =  _gps_position->vel_n_m_s - est_buf[est_i][0][1];
			corr_gps[1][1] =  _gps_position->vel_e_m_s - est_buf[est_i][1][1];
			corr_gps[2][1] = -_gps_position->vel_d_m_s - est_buf[est_i][2][1];
	
			/* save rotation matrix at this moment */
			memcpy(R_gps, R_buf[est_i], sizeof(R_gps));

			w_gps_xy = min_eph_epv / fmaxf(min_eph_epv, _gps_position->eph);
			w_gps_z = min_eph_epv / fmaxf(min_eph_epv, _gps_position->epv);
		}
	}
	else
	{
		//没有GPS信号
		memset(corr_gps, 0, sizeof(corr_gps));
		ref_init_count = 0;
	}
	
	float dt = 0.02f;
	
	//估计器每运行一次，增加一次eph/epv
	if (eph < 0.000001f)    eph = 0.001;
	if (eph < max_eph_epv)  eph *= 1.0f + dt;
	if (epv < 0.000001f)    epv = 0.001;
	if (epv < max_eph_epv)  epv += 0.005f *dt;
	
	uint8_t use_gps_xy = ref_inited && gps_valid && (_gps_position->satellites_used >= 8);
	uint8_t use_gps_z = 1;
    
	uint8_t can_estimate_xy = (eph < 0.2f * max_eph_epv) || use_gps_xy;
	//权重计算
	float w_xy_gps_p = params_w_xy_gps_p * w_gps_xy;
	float w_xy_gps_v = params_w_xy_gps_v * w_gps_xy;
	float w_z_gps_p = params_w_z_gps_p * w_gps_z;
	float w_z_gps_v = params_w_z_gps_v * w_gps_z;
	
	if (use_gps_z) {
		float offs_corr = corr_gps[2][0] * w_z_gps_p * dt;
		baro_offset += offs_corr;
		corr_baro += offs_corr;
	}
	
	//gps修正值计算，最终得到acc_bias
	float accel_bias_corr[3] = { 0.0, 0.0, 0.0 };
	
	if (use_gps_xy) {
		accel_bias_corr[0] -= corr_gps[0][0] * w_xy_gps_p * w_xy_gps_p;
		accel_bias_corr[0] -= corr_gps[0][1] * w_xy_gps_v;
		accel_bias_corr[1] -= corr_gps[1][0] * w_xy_gps_p * w_xy_gps_p;
		accel_bias_corr[1] -= corr_gps[1][1] * w_xy_gps_v;
	}

	if (use_gps_z) {
		accel_bias_corr[2] -= corr_gps[2][0] * w_z_gps_p * w_z_gps_p;
		accel_bias_corr[2] -= corr_gps[2][1] * w_z_gps_v;
	}

	/* transform error vector from NED frame to body frame */
	for (int i = 0; i < 3; i++) {
		float c = 0.0f;

		for (int j = 0; j < 3; j++) {
				c += R_gps[i][j] * accel_bias_corr[j];
		}

		if (isfinite(c)) {
				acc_bias[i] += c * params_w_acc_bias * dt;
		}
	}
	
	accel_bias_corr[0] = 0.0f;
	accel_bias_corr[1] = 0.0f;
	accel_bias_corr[2] = 0.0f;
	
	accel_bias_corr[2] -= corr_baro * params_w_z_baro * params_w_z_baro;
	
	for (int i = 0; i < 3; i++) {
		float c = 0.0f;

		for (int j = 0; j < 3; j++) {
				c += rMat[i][j] * accel_bias_corr[j];
		}

		if (isfinite(c)) {
				acc_bias[i] += c * params_w_acc_bias * dt;
		}
	}
	
	/*高度的预测与修正*/
	inertial_filter_predict(dt, z_est, accel_ned[2]);

	if (!(isfinite(z_est[0]) && isfinite(z_est[1]))) {
		memcpy(z_est, z_est_prev, sizeof(z_est));
	}

	inertial_filter_correct(corr_baro, dt, z_est, 0, params_w_z_baro);

	if (use_gps_z) {
		epv = fminf(epv, _gps_position->epv);

		inertial_filter_correct(corr_gps[2][0], dt, z_est, 0, w_z_gps_p);
		inertial_filter_correct(corr_gps[2][1], dt, z_est, 1, w_z_gps_v);
	}
	
	/*位置的预测与修正*/
	if (can_estimate_xy) {

		inertial_filter_predict(dt, x_est, accel_ned[0]);
		inertial_filter_predict(dt, y_est, accel_ned[1]);
		if (!(isfinite(x_est[0]) && isfinite(x_est[1]) && isfinite(y_est[0]) && isfinite(y_est[1]))) {
			memcpy(x_est, x_est_prev, sizeof(x_est));
			memcpy(y_est, y_est_prev, sizeof(y_est));
		}
		
		/* inertial filter correction for position */
		if (use_gps_xy) {
			eph = fminf(eph, _gps_position->eph);

			inertial_filter_correct(corr_gps[0][0], dt, x_est, 0, w_xy_gps_p);
			inertial_filter_correct(corr_gps[1][0], dt, y_est, 0, w_xy_gps_p);

                        inertial_filter_correct(corr_gps[0][1], dt, x_est, 1, w_xy_gps_v);
                        inertial_filter_correct(corr_gps[1][1], dt, y_est, 1, w_xy_gps_v);
		}
	} else {
		/* gradually reset xy velocity estimates */
		inertial_filter_correct(-x_est[1], dt, x_est, 1, params_w_xy_res_v);
		inertial_filter_correct(-y_est[1], dt, y_est, 1, params_w_xy_res_v);
	}
	
	/* 将估计的位置存入buffer */
	est_buf[buf_ptr][0][0] = x_est[0];
	est_buf[buf_ptr][0][1] = x_est[1];
	est_buf[buf_ptr][1][0] = y_est[0];
	est_buf[buf_ptr][1][1] = y_est[1];
	est_buf[buf_ptr][2][0] = z_est[0];
	est_buf[buf_ptr][2][1] = z_est[1];

	/* push current rotation matrix to buffer */
	memcpy(R_buf[buf_ptr], rMat, sizeof(rMat));

	buf_ptr++;
	if (buf_ptr >= EST_BUF_SIZE) {
		buf_ptr = 0;
	}

	/* update local position */
	Ctrl_state.xy_valid = can_estimate_xy;
	Ctrl_state.v_xy_valid = can_estimate_xy;

	Ctrl_state.x = x_est[0];
	Ctrl_state.vx = x_est[1];
	Ctrl_state.y = y_est[0];
	Ctrl_state.vy = y_est[1];
	Ctrl_state.z = -z_est[0];
	Ctrl_state.vz = -z_est[1];
	
 	if(can_estimate_xy)
	{
		map_projection_reproject(&ref,Ctrl_state.x,Ctrl_state.y,&est_lat,&est_lon);
	}
   
}


