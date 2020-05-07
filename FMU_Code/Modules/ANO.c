#include "ANO.h"
#include "usart.h"
#include "md_struct.h"
#include "md_ubx.h"

uint8_t data_to_send[255] = {0};

static uint8_t Package_Data(uint8_t* txbuf);

extern Control_state  Ctrl_state;
extern float attitude_roll,attitude_pitch,attitude_yaw;
extern float testyaw;
extern float BARO_Alt,barolp;
extern float accel_ned[];
extern float gps_ref[2];

/**********************************************************************************************************
*函 数 名: Communication_Transmit
*功能说明: 发送飞控自定义数据至匿名地面站显示或保存
*形    参: 
*返 回 值: 无
**********************************************************************************************************/
void FCS2Ground_Transmit(void)
{
	HAL_UART_Transmit_DMA(&huart2, data_to_send, Package_Data(data_to_send));
}


static uint8_t Package_Data(uint8_t* txbuf)
{
  uint16_t sTemp = 0;
  uint16_t data_totalnumber = 0;
  __IO int32_t _temp2 = 0;


  txbuf[data_totalnumber++] = 0xAA;                                    //帧头
  txbuf[data_totalnumber++] = 0xAA;                                    //帧头
  txbuf[data_totalnumber++] = 0xF1;                                   
  txbuf[data_totalnumber++] = 0;      

  
  /*--------------------------- Angle 1-3 ------------------------------------*/
  sTemp = (int16_t)(Ctrl_state.gyrof.x*1000.0f);
  txbuf[data_totalnumber++]  = (uint8_t)((sTemp&0xFF00)>>8);    
  txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
  sTemp = (int16_t)(Ctrl_state.gyrof.y*1000.0f);  
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)(Ctrl_state.gyrof.z*1000.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
  
	//4-6
  sTemp = (int16_t)(Ctrl_state.accf.x*1000.0f);
  txbuf[data_totalnumber++]  = (uint8_t)((sTemp&0xFF00)>>8);    
  txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
  sTemp = (int16_t)(Ctrl_state.accf.y*1000.0f);  
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)(Ctrl_state.accf.z*1000.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
	
	//7-9
	sTemp = (int16_t)(Ctrl_state.vx*1000.0f);
  txbuf[data_totalnumber++]  = (uint8_t)((sTemp&0xFF00)>>8);    
  txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
  sTemp = (int16_t)(Ctrl_state.vy*1000.0f);  
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)(gps_ref[0]*100.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
	
	//10-12
	sTemp = (int16_t)(attitude_roll*1000.0f);
  txbuf[data_totalnumber++]  = (uint8_t)((sTemp&0xFF00)>>8);    
  txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
  sTemp = (int16_t)(attitude_pitch*1000.0f);  
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)(attitude_yaw*1000.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
	
	//13-15
	sTemp = (int16_t)((_gps_position->satellites_used)*1.0f);
  txbuf[data_totalnumber++]  = (uint8_t)((sTemp&0xFF00)>>8);    
  txbuf[data_totalnumber++]  = (uint8_t)(sTemp&0x00FF);
  sTemp = (int16_t)((BARO_Alt)*100.0f);  
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)((barolp)*100.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
	
	//16-18
	sTemp = (int16_t)(_gps_position->vel_n_m_s*1000.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
	sTemp = (int16_t)(_gps_position->vel_e_m_s*1000.0f);  
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)(_gps_position->vel_d_m_s*1000.0f);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);

	//19-20
	sTemp = (int16_t)(gps_ref[1]*100);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);	
  sTemp = (int16_t)((Ctrl_state.vz)*1000);
  txbuf[data_totalnumber++] = (uint8_t)((sTemp&0xFF00)>>8);  
  txbuf[data_totalnumber++] = (uint8_t)(sTemp&0x00FF);
	
	txbuf[3]                  = data_totalnumber - 4;
	
	uint8_t sum = 0;
	for(uint8_t i=0;i<data_totalnumber;i++)
		sum += txbuf[i];
	txbuf[data_totalnumber++]=sum;
	
	return data_totalnumber; 
}










