/*******************************************************************************
* 文件名称：md_gps.h
*
* 摘    要：GPS相关初始化
*
* 当前版本：
* 作    者：Acorss工作室
* 日    期：2018/04/30
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/
#ifndef __GPS_H
#define __GPS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stdio.h"
	
#define GPS_READ_BUFFER_SIZE 128
#define DEBUG_GPS_ENABLE
	
/*重新定GPS_DEBUG函数*/
#ifdef DEBUG_GPS_ENABLE
#define 	GPS_DEBUG(format, ...) printf (format, ##__VA_ARGS__)
#else
#define GPS_DEBUG(format,...)   
#endif  
	
	typedef enum {
		GPS = 0,    ///< normal GPS output
		RTCM        ///< request RTCM output. This is used for (fixed position) base stations
	}
	OutputMode;

	struct satellite_info_s
	{
		uint64_t timestamp; // required for logger
		uint8_t count;
		uint8_t svid[20];
		uint8_t used[20];
		uint8_t elevation[20];
		uint8_t azimuth[20];
		uint8_t snr[20];
		uint8_t _padding0[3]; // required for logger
		//const uint8_t SAT_INFO_MAX_SATELLITES = 20;
	};

	struct vehicle_gps_position_s
	{
		uint64_t timestamp; // required for logger
		uint64_t time_utc_usec;
		int32_t lat;
		int32_t lon;
		int32_t alt;
		int32_t alt_ellipsoid;
		float s_variance_m_s;
		float c_variance_rad;
		float eph;
		float epv;
		float hdop;
		float vdop;
		int32_t noise_per_ms;
		int32_t jamming_indicator;
		float vel_m_s;
		float vel_n_m_s;
		float vel_e_m_s;
		float vel_d_m_s;
		float cog_rad;
		int32_t timestamp_time_relative;
		uint8_t fix_type;
		uint8_t vel_ned_valid;
		uint8_t satellites_used;
		uint8_t _padding0[5]; // required for logger
	};

	struct tm
	{
		int tm_sec;     /* second (0-61, allows for leap seconds) */
		int tm_min;     /* minute (0-59) */
		int tm_hour;    /* hour (0-23) */
		int tm_mday;    /* day of the month (1-31) */
		int tm_mon;     /* month (0-11) */
		int tm_year;    /* years since 1900 */
		int tm_wday;    /* day of the week (0-6) */                         /*not supported by NuttX*/
		int tm_yday;    /* day of the year (0-365) */                       /*not supported by NuttX*/
		int tm_isdst;   /* non-0 if daylight savings time is in effect */   /*not supported by NuttX*/
	};

	struct SurveyInStatus
	{
		uint32_t mean_accuracy;       /**< [mm] */
		uint32_t duration;            /**< [s] */
		uint8_t flags;                /**< bit 0: valid, bit 1: active */
	};



#define SAT_INFO_MAX_SATELLITES 20
	void GPS_RB_Clear(void);
	uint8_t  GPS_RB_IsOverFlow(void);
	void GPS_RB_Push(uint8_t data);
	uint8_t GPS_RB_Pop(void);
	uint8_t GPS_RB_HasNew(void);
	uint16_t GPS_RB_Count(void);


	void GPS_Init(void);
	void GPS_RX_InterruptHandler(void);
	void Loop_GPS_Parse(void);
	void GPS_Baud_Reset(uint32_t baud);
#ifdef __cplusplus
}
#endif

#endif

