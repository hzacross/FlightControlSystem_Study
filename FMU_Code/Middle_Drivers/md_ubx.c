/*******************************************************************************
* 文件名称：md_ubx.c
*
* 摘    要：GPS的ubx格式相关解码函数
*
* 当前版本：
* 作    者：Acorss工作室
* 日    期：2018/04/30
* 编译环境：keil5
*
* 历史信息：
*******************************************************************************/

#include "md_ubx.h"
#include "md_gps.h"
#include "usart.h"
#include  <string.h>

#define M_DEG_TO_RAD_F 0.01745329251994f
#define M_DEG_TO_RAD_F 0.01745329251994f

#define UBX_CONFIG_TIMEOUT	200	// ms, timeout for waiting ACK
#define UBX_PACKET_TIMEOUT	2		// ms, if now data during this delay assume that full update received
#define DISABLE_MSG_INTERVAL	1000000		// us, try to disable message with this interval

#define MIN(X,Y)	((X) < (Y) ? (X) : (Y))
#define SWAP16(X)	((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

#define FNV1_32_INIT	((uint32_t)0x811c9dc5)	// init value for FNV1 hash algorithm
#define FNV1_32_PRIME	((uint32_t)0x01000193)	// magic prime for FNV1 hash algorithm

uint8_t _rate_count_lat_lon;
uint8_t _rate_count_vel;

rtcm_message_t	*_rtcm_message = NULL;
struct vehicle_gps_position_s  m_gps_position;
struct vehicle_gps_position_s *_gps_position = &m_gps_position;
struct satellite_info_s *_satellite_info= NULL;

uint64_t _last_timestamp_time;
ubx_decode_state_t	_decode_state;

uint8_t			_got_posllh;
uint8_t			_got_velned;

ubx_ack_state_t		_ack_state;
uint8_t			_configured;
uint16_t		_rx_msg;
ubx_rxmsg_state_t	_rx_state;
uint16_t		_rx_payload_length;
uint16_t		_rx_payload_index;
uint8_t			_rx_ck_a;
uint8_t			_rx_ck_b;
gps_abstime		_disable_cmd_last;
uint8_t			_use_nav_pvt;
uint16_t		_ack_waiting_msg;
ubx_buf_t		_buf;
uint32_t		_ubx_version;
OutputMode	_output_mode = GPS;

uint8_t RTCM_BUFFER[RTCM_INITIAL_BUFFER_LENGTH];

uint8_t ParseChar(uint8_t b);

/**
 * write to the device
 * @param buf
 * @param buf_length
 * @return num written bytes, -1 on error
 */
int write(const void *buf, int buf_length)
{
	//return _callback(GPSCallbackType::writeDeviceData, (void *)buf, buf_length, _callback_user);
	HAL_StatusTypeDef ret;
	ret = HAL_UART_Transmit(&huart4,(uint8_t*)buf,buf_length,1000);
	if(ret == HAL_OK)
	{
		return buf_length;
	}
	else
	{
		return -1;
	}
}

/**
 * read from device
 * @param buf: pointer to read buffer
 * @param buf_length: size of read buffer
 * @param timeout: timeout in ms
 * @return: 0 for nothing read, or poll timed out
 *	    < 0 for error
 *	    > 0 number of bytes read
 */
int read(uint8_t *buf, int buf_length, int timeout)
{
	uint64_t time_started = HAL_GetTick();
	int i = 0;
	while(1)
	{

		if(GPS_RB_IsOverFlow())
		{
			GPS_RB_Clear();
			return 0;
		}
		while(GPS_RB_HasNew())
		{
			buf[i] = GPS_RB_Pop();
			i++;
			if(i==buf_length)
			{
				return i;
			}
		}
		if((time_started + timeout) < HAL_GetTick())
		{
			//time out
			return i;
		}
	}
}

int8_t UBX_Receive(void)
{
	int handled = 0;
	uint8_t read=0;

	if(GPS_RB_IsOverFlow())
	{
		GPS_RB_Clear();
		return handled;
	}

	while(GPS_RB_HasNew())
	{
		read = GPS_RB_Pop();
		handled |= ParseChar(read);
		//GPS_DEBUG("%x ",read);
	}
	return handled;
}

void decodeInit(void)
{
	_decode_state = UBX_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
}

void calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	for (uint16_t i = 0; i < length; i++)
	{
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}

uint8_t sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
	ubx_checksum_t checksum = {0, 0};
	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

	if (payload != NULL)
	{
		calcChecksum(payload, length, &checksum);
	}

	// Send message
	if (write((void *)&header, sizeof(header)) != sizeof(header))
	{
		return 0;
	}

	if (payload && write((void *)payload, length) != length)
	{
		return 0;
	}

	if (write((void *)&checksum, sizeof(checksum)) != sizeof(checksum))
	{
		return 0;
	}

	return 1;
}

void addByteToChecksum(const uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

uint8_t configureMessageRate(const uint16_t msg, const uint8_t rate)
{
	ubx_payload_tx_cfg_msg_t cfg_msg;	// don't use _buf (allow interleaved operation)

	cfg_msg.msg	= msg;
	cfg_msg.rate	= rate;

	return sendMessage(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
}

/**
 * Start payload rx
// -1 = abort, 0 = continue
 */
int	payloadRxInit()
{
	int ret = 0;

	_rx_state = UBX_RXMSG_HANDLE;	// handle by default

	switch (_rx_msg)
	{
	case UBX_MSG_NAV_PVT:
		if ((_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)		/* u-blox 7 msg format */
		        && (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8))  	/* u-blox 8+ msg format */
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}
		else if (!_use_nav_pvt)
		{
			_rx_state = UBX_RXMSG_DISABLE;        // disable if not using NAV-PVT
		}

		break;

	case UBX_MSG_INF_DEBUG:
	case UBX_MSG_INF_ERROR:
	case UBX_MSG_INF_NOTICE:
	case UBX_MSG_INF_WARNING:
		if (_rx_payload_length >= sizeof(ubx_buf_t))
		{
			_rx_payload_length = sizeof(ubx_buf_t) - 1; //avoid buffer overflow
		}

		break;

	case UBX_MSG_NAV_POSLLH:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}
		else if (_use_nav_pvt)
		{
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_SOL:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}
		else if (_use_nav_pvt)
		{
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_DOP:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_dop_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}
		else if (_use_nav_pvt)
		{
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_SVINFO:
		if (_satellite_info == NULL)
		{
			_rx_state = UBX_RXMSG_DISABLE;        // disable if sat info not requested

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}
		else
		{
			memset(_satellite_info, 0, sizeof(*_satellite_info));        // initialize sat info
		}

		break;

	case UBX_MSG_NAV_SVIN:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_svin_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_VELNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}
		else if (_use_nav_pvt)
		{
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_MON_VER:
		break;		// unconditionally handle this message

	case UBX_MSG_MON_HW:
		if ((_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))	/* u-blox 6 msg format */
		        && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t)))  	/* u-blox 7+ msg format */
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (!_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured
		}

		break;

	case UBX_MSG_ACK_ACK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
		}

		break;

	case UBX_MSG_ACK_NAK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t))
		{
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		}
		else if (_configured)
		{
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
		}

		break;

	default:
		_rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
		break;
	}

	switch (_rx_state)
	{
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
	{
		gps_abstime t = HAL_GetTick();

		if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL)
		{
			/* don't attempt for every message to disable, some might not be disabled */
			_disable_cmd_last = t;
			//UBX_DEBUG("ubx disabling msg 0x%04x", SWAP16((unsigned)_rx_msg));

			if (!configureMessageRate(_rx_msg, 0))
			{
				ret = -1;
			}
		}
	}

	ret = -1;	// return error, abort handling this message
	break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}

/**
 * Add NAV-SVINFO payload rx byte
 */
// -1 = error, 0 = ok, 1 = payload completed
int	payloadRxAddNavSvinfo(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t))
	{
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	}
	else
	{
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t))
		{
			// Part 1 complete: decode Part 1 buffer
			_satellite_info->count = MIN(_buf.payload_rx_nav_svinfo_part1.numCh,SAT_INFO_MAX_SATELLITES);
		}

		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t) + _satellite_info->count * sizeof(
		            ubx_payload_rx_nav_svinfo_part2_t))
		{
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(
			                         ubx_payload_rx_nav_svinfo_part2_t);
			p_buf[buf_index] = b;

			if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1)
			{
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) / sizeof(
				                         ubx_payload_rx_nav_svinfo_part2_t);
				_satellite_info->used[sat_index]	= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.flags & 0x01);
				_satellite_info->snr[sat_index]		= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.cno);
				_satellite_info->elevation[sat_index]	= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.elev);
				_satellite_info->azimuth[sat_index]	= (uint8_t)((float)_buf.payload_rx_nav_svinfo_part2.azim * 255.0f / 360.0f);
				_satellite_info->svid[sat_index]	= (uint8_t)(_buf.payload_rx_nav_svinfo_part2.svid);
			}
		}
	}
	if (++_rx_payload_index >= _rx_payload_length)
	{
		ret = 1;	// payload received completely
	}

	return ret;
}


uint32_t fnv1_32_str(uint8_t *str, uint32_t hval)
{
	uint8_t *s = str;

	/*
	 * FNV-1 hash each octet in the buffer
	 */
	while (*s)
	{

		/* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
		hval *= FNV1_32_PRIME;
#else
		hval += (hval << 1) + (hval << 4) + (hval << 7) + (hval << 8) + (hval << 24);
#endif

		/* xor the bottom with the current octet */
		hval ^= (uint32_t) * s++;
	}

	/* return our new hash value */
	return hval;
}

/**
 * Add MON-VER payload rx byte
 */
// -1 = error, 0 = ok, 1 = payload completed
int	payloadRxAddMonVer(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t))
	{
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	}
	else
	{
		if (_rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t))
		{
			// Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.hwVersion, _ubx_version);
		}

		// fill Part 2 buffer
		unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(
		                         ubx_payload_rx_mon_ver_part2_t);
		p_buf[buf_index] = b;

		if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1)
		{
			// Part 2 complete: decode Part 2 buffer
			//UBX_DEBUG("VER ext \" %30s\"", _buf.payload_rx_mon_ver_part2.extension);
		}
	}

	if (++_rx_payload_index >= _rx_payload_length)
	{
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add payload rx byte
 */
// -1 = error, 0 = ok, 1 = payload completed
int	 payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length)
	{
		ret = 1;	// payload received completely
	}

	return ret;
}



/**
 * Finish payload rx
 */
// 0 = no message handled, 1 = message handled, 2 = sat info message handled

int	payloadRxDone(void)
{
	int ret = 0;

	// return if no message handled
	if (_rx_state != UBX_RXMSG_HANDLE)
	{
		return ret;
	}

	// handle message
	switch (_rx_msg)
	{

	case UBX_MSG_NAV_PVT:
		//Check if position fix flag is good
		if ((_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1)
		{
			_gps_position->fix_type		 = _buf.payload_rx_nav_pvt.fixType;

			if (_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN)
			{
				_gps_position->fix_type = 4; //DGPS
			}

			uint8_t carr_soln = _buf.payload_rx_nav_pvt.flags >> 6;

			if (carr_soln == 1)
			{
				_gps_position->fix_type = 5; //Float RTK

			}
			else if (carr_soln == 2)
			{
				_gps_position->fix_type = 6; //Fixed RTK
			}

			_gps_position->vel_ned_valid = 1;

		}
		else
		{
			_gps_position->fix_type		 = 0;
			_gps_position->vel_ned_valid = 0;
		}

		_gps_position->satellites_used	= _buf.payload_rx_nav_pvt.numSV;

		_gps_position->lat		= _buf.payload_rx_nav_pvt.lat;
		_gps_position->lon		= _buf.payload_rx_nav_pvt.lon;
		_gps_position->alt		= _buf.payload_rx_nav_pvt.hMSL;

		_gps_position->eph		= (float)_buf.payload_rx_nav_pvt.hAcc * 1e-3f;
		_gps_position->epv		= (float)_buf.payload_rx_nav_pvt.vAcc * 1e-3f;
		_gps_position->s_variance_m_s	= (float)_buf.payload_rx_nav_pvt.sAcc * 1e-3f;

		_gps_position->vel_m_s		= (float)_buf.payload_rx_nav_pvt.gSpeed * 1e-3f;

		_gps_position->vel_n_m_s	= (float)_buf.payload_rx_nav_pvt.velN * 1e-3f;
		_gps_position->vel_e_m_s	= (float)_buf.payload_rx_nav_pvt.velE * 1e-3f;
		_gps_position->vel_d_m_s	= (float)_buf.payload_rx_nav_pvt.velD * 1e-3f;

		_gps_position->cog_rad		= (float)_buf.payload_rx_nav_pvt.headMot * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad	= (float)_buf.payload_rx_nav_pvt.headAcc * M_DEG_TO_RAD_F * 1e-5f;
		// GPS_DEBUG("eph: %f ,epv:%f\r\n" , _gps_position->eph,_gps_position->epv);
		GPS_DEBUG("_gps_position lat:%lf ,lon:%lf ,alt:%lf \r\n" , (double)_gps_position->lat/10000000.0,(double)_gps_position->lon/10000000.0,(double)_gps_position->alt/1000000.0);
		GPS_DEBUG("gps vel N:%f ,E:%f ,D:%f \r\n" , _gps_position->vel_n_m_s,_gps_position->vel_e_m_s,_gps_position->vel_d_m_s);
		// GPS_DEBUG("gps cog_rad ：%f \r\n" , _gps_position->cog_rad);

		//Check if time and date fix flags are good
		if ((_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
		        && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
		        && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED))
		{
			/* convert to unix timestamp */
			volatile struct tm timeinfo;
			timeinfo.tm_year	= _buf.payload_rx_nav_pvt.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_pvt.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_pvt.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_pvt.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_pvt.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_pvt.sec;
			_gps_position->time_utc_usec = 0;

		}

		_gps_position->timestamp = HAL_GetTick();
		_last_timestamp_time = _gps_position->timestamp;

		_rate_count_vel++;
		_rate_count_lat_lon++;

		_got_posllh = 1;
		_got_velned = 1;

		ret = 1;
		break;

	case UBX_MSG_INF_DEBUG:
	case UBX_MSG_INF_NOTICE:
	{
		uint8_t *p_buf = (uint8_t *)&_buf;
		p_buf[_rx_payload_length] = 0;
	}
	break;

	case UBX_MSG_INF_ERROR:
	case UBX_MSG_INF_WARNING:
	{
		uint8_t *p_buf = (uint8_t *)&_buf;
		p_buf[_rx_payload_length] = 0;
	}
	break;

	case UBX_MSG_NAV_POSLLH:
		_gps_position->lat	= _buf.payload_rx_nav_posllh.lat;
		_gps_position->lon	= _buf.payload_rx_nav_posllh.lon;
		_gps_position->alt	= _buf.payload_rx_nav_posllh.hMSL;
		_gps_position->eph	= (float)_buf.payload_rx_nav_posllh.hAcc * 1e-3f; // from mm to m
		_gps_position->epv	= (float)_buf.payload_rx_nav_posllh.vAcc * 1e-3f; // from mm to m
		_gps_position->alt_ellipsoid = _buf.payload_rx_nav_posllh.height;

		_gps_position->timestamp = HAL_GetTick();
		uint8_t _rate_count_lat_lon;
		_rate_count_lat_lon++;
		_got_posllh = 1;

		ret = 1;
		//GPS_DEBUG("_gps_position lat:%d ,lon:%d ,alt:%d \r\n" , _gps_position->lat,_gps_position->lon,_gps_position->alt);
		break;

	case UBX_MSG_NAV_SOL:
		_gps_position->fix_type		= _buf.payload_rx_nav_sol.gpsFix;
		_gps_position->s_variance_m_s	= (float)_buf.payload_rx_nav_sol.sAcc * 1e-2f;	// from cm to m
		_gps_position->satellites_used	= _buf.payload_rx_nav_sol.numSV;

		ret = 1;
		break;

	case UBX_MSG_NAV_DOP:
		_gps_position->hdop		= _buf.payload_rx_nav_dop.hDOP * 0.01f;	// from cm to m
		_gps_position->vdop		= _buf.payload_rx_nav_dop.vDOP * 0.01f;	// from cm to m
		GPS_DEBUG("hdop:%f ,vdop:%f\r\n" , _gps_position->hdop,_gps_position->vdop);
		ret = 1;
		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (_buf.payload_rx_nav_timeutc.valid & UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC)
		{
			// convert to unix timestamp
			volatile struct tm timeinfo;
			timeinfo.tm_year	= _buf.payload_rx_nav_timeutc.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_timeutc.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_timeutc.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_timeutc.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_timeutc.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_timeutc.sec;

			_gps_position->time_utc_usec = 0;

		}

		_last_timestamp_time = HAL_GetTick();

		ret = 1;
		break;

	case UBX_MSG_NAV_SVINFO:
		// _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
		_satellite_info->timestamp = HAL_GetTick();

		ret = 2;
		break;

	case UBX_MSG_NAV_SVIN:
		ret = 1;
		break;

	case UBX_MSG_NAV_VELNED:
		_gps_position->vel_m_s		= (float)_buf.payload_rx_nav_velned.speed * 1e-2f;
		_gps_position->vel_n_m_s	= (float)_buf.payload_rx_nav_velned.velN * 1e-2f; /* NED NORTH velocity */
		_gps_position->vel_e_m_s	= (float)_buf.payload_rx_nav_velned.velE * 1e-2f; /* NED EAST velocity */
		_gps_position->vel_d_m_s	= (float)_buf.payload_rx_nav_velned.velD * 1e-2f; /* NED DOWN velocity */
		_gps_position->cog_rad		= (float)_buf.payload_rx_nav_velned.heading * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad	= (float)_buf.payload_rx_nav_velned.cAcc * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->vel_ned_valid	= 1;

		_rate_count_vel++;
		_got_velned = 1;

		ret = 1;
		// GPS_DEBUG("gps vel N:%f ,E:%f ,D:%f \r\n" , _gps_position->vel_n_m_s,_gps_position->vel_e_m_s,_gps_position->vel_d_m_s);
		break;

	case UBX_MSG_MON_VER:
		ret = 1;
		break;

	case UBX_MSG_MON_HW:
		switch (_rx_payload_length)
		{

		case sizeof(ubx_payload_rx_mon_hw_ubx6_t):	/* u-blox 6 msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx6.noisePerMS;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx6.jamInd;

			ret = 1;
			break;

		case sizeof(ubx_payload_rx_mon_hw_ubx7_t):	/* u-blox 7+ msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx7.noisePerMS;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx7.jamInd;

			ret = 1;
			break;

		default:		// unexpected payload size:
			ret = 0;	// don't handle message
			break;
		}

		break;

	case UBX_MSG_ACK_ACK:
		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg))
		{
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_ACK_NAK:
		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg))
		{
			_ack_state = UBX_ACK_GOT_NAK;
		}

		ret = 1;
		break;

	default:
		break;
	}

	if (ret > 0)
	{
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}




uint8_t ParseChar(uint8_t b)
{
	int ret = 0;
	switch (_decode_state)
	{
		/* Expecting Sync1 */
	case UBX_DECODE_SYNC1:
		if (b == UBX_SYNC1)  	// Sync1 found --> expecting Sync2
		{
			_decode_state = UBX_DECODE_SYNC2;

		}
		else if (b == RTCM3_PREAMBLE && _rtcm_message)
		{
			_decode_state = UBX_DECODE_RTCM3;
			_rtcm_message->buffer[_rtcm_message->pos++] = b;
		}
		break;

		/* Expecting Sync2 */
	case UBX_DECODE_SYNC2:
		if (b == UBX_SYNC2)  	// Sync2 found --> expecting Class
		{
			_decode_state = UBX_DECODE_CLASS;

		}
		else  		// Sync1 not followed by Sync2: reset parser
		{
			decodeInit();
		}

		break;

		/* Expecting Class */
	case UBX_DECODE_CLASS:
		addByteToChecksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
		_rx_msg = b;
		_decode_state = UBX_DECODE_ID;
		break;

		/* Expecting ID */
	case UBX_DECODE_ID:
		addByteToChecksum(b);
		_rx_msg |= b << 8;
		_decode_state = UBX_DECODE_LENGTH1;
		break;

		/* Expecting first length byte */
	case UBX_DECODE_LENGTH1:
		addByteToChecksum(b);
		_rx_payload_length = b;
		_decode_state = UBX_DECODE_LENGTH2;
		break;

		/* Expecting second length byte */
	case UBX_DECODE_LENGTH2:
		addByteToChecksum(b);
		_rx_payload_length |= b << 8;	// calculate payload size

		if (payloadRxInit() != 0)  	// start payload reception
		{
			// payload will not be handled, discard message
			decodeInit();

		}
		else
		{
			_decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
		}

		break;


		/* Expecting payload */
	case UBX_DECODE_PAYLOAD:
		addByteToChecksum(b);

		switch (_rx_msg)
		{
		case UBX_MSG_NAV_SVINFO:
			ret = payloadRxAddNavSvinfo(b);	// add a NAV-SVINFO payload byte
			break;

		case UBX_MSG_MON_VER:
			ret = payloadRxAddMonVer(b);	// add a MON-VER payload byte
			break;

		default:
			ret = payloadRxAdd(b);		// add a payload byte
			break;
		}

		if (ret < 0)
		{
			// payload not handled, discard message
			decodeInit();

		}
		else if (ret > 0)
		{
			// payload complete, expecting checksum
			_decode_state = UBX_DECODE_CHKSUM1;

		}
		else
		{
			// expecting more payload, stay in state UBX_DECODE_PAYLOAD
		}

		ret = 0;
		break;

		/* Expecting first checksum byte */
	case UBX_DECODE_CHKSUM1:
		if (_rx_ck_a != b)
		{
			decodeInit();

		}
		else
		{
			_decode_state = UBX_DECODE_CHKSUM2;
		}

		break;

		/* Expecting second checksum byte */
	case UBX_DECODE_CHKSUM2:
		if (_rx_ck_b != b)
		{
			//	UBX_WARN("ubx checksum err");

		}
		else
		{
			ret = payloadRxDone();	// finish payload processing
		}

		decodeInit();
		break;

	default:
		break;
	}
	return ret;
}
// -1 = NAK, error or timeout, 0 = ACK
int	waitForAck(const uint16_t msg, const unsigned timeout, uint8_t report)
{
	int ret = -1;

	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check

	gps_abstime time_started = HAL_GetTick();
	gps_abstime time_new =HAL_GetTick();
	while ((_ack_state == UBX_ACK_WAITING) && (time_new < time_started + timeout))
	{
		UBX_Receive();
		time_new=HAL_GetTick();
	}

	if (_ack_state == UBX_ACK_GOT_ACK)
	{
		ret = 0;	// ACK received ok
		GPS_DEBUG("ACK IS OK--------------\r\n");
	}
	else if (report)
	{
		if (_ack_state == UBX_ACK_GOT_NAK)
		{
			//UBX_DEBUG("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));

		}
		else
		{
			//UBX_DEBUG("ubx msg 0x%04x ACK timeout", SWAP16((unsigned)msg));
		}
	}

	_ack_state = UBX_ACK_IDLE;
	return ret;
}

uint8_t configureMessageRateAndAck(uint16_t msg, uint8_t rate, uint8_t report_ack_error)
{
	if (!configureMessageRate(msg, rate))
	{
		return 0;
	}
	return waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, report_ack_error) >= 0;
}

int configure(uint32_t *baudrate, OutputMode output_mode,uint64_t timeout)
{
	_configured = 0;
	_output_mode = output_mode;
	/* try different baudrates */
	const unsigned baudrates[] = {9600, 38400, 19200, 57600, 115200};
	//const unsigned baudrates[] = {9600, 9600, 9600, 9600, 9600};
	unsigned baud_i;
	
	uint64_t time_started = HAL_GetTick();
	uint64_t time_now =  HAL_GetTick();
	
	ubx_payload_tx_cfg_prt_t cfg_prt[2];
	uint16_t out_proto_mask = output_mode == GPS ?
	                          UBX_TX_CFG_PRT_OUTPROTOMASK_GPS :
	                          UBX_TX_CFG_PRT_OUTPROTOMASK_RTCM;
	uint16_t in_proto_mask = output_mode == GPS ?
	                         UBX_TX_CFG_PRT_INPROTOMASK_GPS :
	                         UBX_TX_CFG_PRT_INPROTOMASK_RTCM;
while(!_configured)
{
	//判断是否超时
	time_now = HAL_GetTick();
	if(time_now > time_started + timeout)
	{
	  break;
	}
	
	for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++)
	{
		*baudrate = baudrates[baud_i];
		GPS_Baud_Reset(*baudrate);
			/* flush input and wait for at least 20 ms silence */
		decodeInit();
		HAL_Delay(20);
		UBX_Receive();				
		decodeInit();
		UBX_Receive();
    decodeInit();		
		/* Send a CFG-PRT message to set the UBX protocol for in and out
				 * and leave the baudrate as it is, we just want an ACK-ACK for this */
		memset(cfg_prt, 0, 2 * sizeof(ubx_payload_tx_cfg_prt_t));
		cfg_prt[0].portID		= UBX_TX_CFG_PRT_PORTID;
		cfg_prt[0].mode		= UBX_TX_CFG_PRT_MODE;
		cfg_prt[0].baudRate	= *baudrate;
		cfg_prt[0].inProtoMask	= in_proto_mask;
		cfg_prt[0].outProtoMask	= out_proto_mask;
		/*
		cfg_prt[1].portID		= UBX_TX_CFG_PRT_PORTID_USB;
		cfg_prt[1].mode		= UBX_TX_CFG_PRT_MODE;
		cfg_prt[1].baudRate	= *baudrate;
		cfg_prt[1].inProtoMask	= in_proto_mask;
		cfg_prt[1].outProtoMask	= out_proto_mask;
*/
		GPS_DEBUG("GPS UBX_MSG_CFG_PRT configure:,%d----\r\n",*baudrate);
		if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, sizeof(ubx_payload_tx_cfg_prt_t)))
		{
			continue;
		}
		//	GPS_DEBUG("waitForAck----\r\n");
		if (waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, 0) < 0)
		{
			/* try next baudrate */
			GPS_DEBUG("try next baudrate----\r\n");
			continue;
		}
		else
		{
			//	  GPS_DEBUG("GPS UBX_MSG_CFG_PRT ACK----\r\n");
		}

		/* Send a CFG-RATE message to define update rate */
		memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
		_buf.payload_tx_cfg_rate.measRate	= UBX_TX_CFG_RATE_MEASINTERVAL;
		_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
		_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

		//GPS_DEBUG("GPS UBX_MSG_CFG_RATE configure----\r\n");
		if (!sendMessage(UBX_MSG_CFG_RATE, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rate)))
		{
			return -1;
		}
		if (waitForAck(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, 1) < 0)
		{
			return -1;
		}
		else
		{
			//GPS_DEBUG("GPS UBX_MSG_CFG_RATE ACK----\r\n");
		}


		/* send a NAV5 message to set the options for the internal filter */
		memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
		_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
		_buf.payload_tx_cfg_nav5.dynModel	= output_mode ==GPS ?
		                                      UBX_TX_CFG_NAV5_DYNMODEL :
		                                      UBX_TX_CFG_NAV5_DYNMODEL_RTCM;
		_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;

		//GPS_DEBUG("GPS UBX_MSG_CFG_NAV5 configure----\r\n");
		if (!sendMessage(UBX_MSG_CFG_NAV5, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_nav5)))
		{
			return -1;
		}
		if (waitForAck(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, 1) < 0)
		{
			return -1;
		}
		else
		{
			// GPS_DEBUG("GPS UBX_MSG_CFG_NAV5 ACK----\r\n");
		}
		/* configure message rates */
		/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */

		/* try to set rate for NAV-PVT */
		/* (implemented for ubx7+ modules only, use NAV-SOL, NAV-POSLLH, NAV-VELNED and NAV-TIMEUTC for ubx6) */
		//GPS_DEBUG("GPS UBX_MSG_NAV_PVT configure----\r\n");
		if (!configureMessageRate(UBX_MSG_NAV_PVT, 1))
		{
			return -1;
		}
		if (waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, 1) < 0)
		{
			_use_nav_pvt = 0;

		}
		else
		{
			_use_nav_pvt = 1;
		}

		if (!_use_nav_pvt)
		{
			//GPS_DEBUG("GPS UBX_MSG_NAV_TIMEUTC configure----\r\n");
			if (!configureMessageRateAndAck(UBX_MSG_NAV_TIMEUTC, 5, 1))
			{
				return -1;
			}
			// GPS_DEBUG("GPS UBX_MSG_NAV_POSLLH configure----\r\n");
			if (!configureMessageRateAndAck(UBX_MSG_NAV_POSLLH, 1, 1))
			{
				return -1;
			}
			// GPS_DEBUG("GPS UBX_MSG_NAV_SOL configure----\r\n");
			if (!configureMessageRateAndAck(UBX_MSG_NAV_SOL, 1, 1))
			{
				return -1;
			}
			// GPS_DEBUG("GPS UBX_MSG_NAV_VELNED configure----\r\n");
			if (!configureMessageRateAndAck(UBX_MSG_NAV_VELNED, 1, 1))
			{
				return -1;
			}
		}
// GPS_DEBUG("GPS UBX_MSG_NAV_DOP configure----\r\n");
		if (!configureMessageRateAndAck(UBX_MSG_NAV_DOP, 1, 1))
		{
			return -1;
		}
		//GPS_DEBUG("GPS UBX_MSG_NAV_SVINFO configure----\r\n");
		/*
		if (!configureMessageRateAndAck(UBX_MSG_NAV_SVINFO, (_satellite_info != NULL) ? 5 : 0, 1))
		{
			return -1;
		}
		*/
//GPS_DEBUG("GPS UBX_MSG_MON_HW configure----\r\n");
		/*
		if (!configureMessageRateAndAck(UBX_MSG_MON_HW, 1, 1))
		{
			return -1;
		}
		*/
//GPS_DEBUG("GPS UBX_MSG_MON_VER configure----\r\n");
		/* request module version information by sending an empty MON-VER message */
		/*
		if (!sendMessage(UBX_MSG_MON_VER, NULL, 0))
		{
			return -1;
		}
		*/
		/* at this point we have correct baudrate on both ends */	
		_configured = 1;
//   	GPS_DEBUG("GPS CONFIGURE COMPLETED----\r\n");
		break;
	}
	
	
}	
	return 0;
}

