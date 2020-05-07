#include "ringbuffer.h"
/**
  * @brief  rbInitialize初始化配置，将缓冲区信息填入结构体
  * @param  pRingBuff:ringbuffer结构体
	*         buff：数据缓冲区
	*         length：缓冲区大小
  * @note   
  * @retval void
  * @author Acorss工作室	
  */	
void rbInit(RingBuffer* pRingBuff, uint8_t* buff, uint16_t length)
{
	pRingBuff->pBuff = buff;
	pRingBuff->pEnd  = buff + length;
	pRingBuff->wp = buff;
	pRingBuff->rp = buff;
	pRingBuff->length = length;
	pRingBuff->flagOverflow = 0;
}

