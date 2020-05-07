#ifndef __RINGBUFFER_H
#define __RINGBUFFER_H

#ifdef __cplusplus
 extern "C" {
#endif 

#include "stm32f4xx_hal.h"
	 
#include <stdio.h>

typedef struct {
	uint8_t* pBuff;
	uint8_t* pEnd;  // pBuff + legnth
	uint8_t* wp;    // Write Point
	uint8_t* rp;    // Read Point
	uint16_t length;
	uint8_t  flagOverflow; // set when buffer overflowed
} RingBuffer;


void rbInit(RingBuffer* pRingBuff, uint8_t* buff, uint16_t length);
void rbClear(RingBuffer* pRingBuff);
void rbPush(RingBuffer* pRingBuff, uint8_t value);
uint8_t rbPop(RingBuffer* pRingBuff);
uint16_t rbGetCount(const RingBuffer* pRingBuff);
int8_t rbIsEmpty(const RingBuffer* pRingBuff);
int8_t rbIsFull(const RingBuffer* pRingBuff);

/**
  * @brief  清除ringbuffer结构体的相关信息
  * @param  pRingBuff：待处理的ringbuffer
  * @note   
  * @retval void
  * @author Acorss工作室	
  */	
inline void rbClear(RingBuffer* pRingBuff)
{
 	pRingBuff->wp = pRingBuff->pBuff;
	pRingBuff->rp = pRingBuff->pBuff;
	pRingBuff->flagOverflow = 0;
}

/**
  * @brief  压入单字节到缓冲区
  * @param  pRingBuff：待处理的ringbuffer
  *         value：压入的数据
  * @note   
  * @retval void
  * @author Acorss工作室	
  */	
inline void rbPush(RingBuffer* pRingBuff, uint8_t value)
{
	uint8_t* wp_next = pRingBuff->wp + 1;
	if( wp_next == pRingBuff->pEnd ) {
		wp_next -= pRingBuff->length; // Rewind pointer when exceeds bound
	}
	if( wp_next != pRingBuff->rp ) {
		*pRingBuff->wp = value;
		pRingBuff->wp = wp_next;
	} else {
		pRingBuff->flagOverflow = 1;
	}
}

/**
  * @brief  压出单字节到缓冲区
  * @param  pRingBuff：待处理的ringbuffer   
  * @note   
  * @retval 压出的数据
  * @author Acorss工作室	
  */	
inline uint8_t rbPop(RingBuffer* pRingBuff)
{
	if( pRingBuff->rp == pRingBuff->wp ) 
		return 0; // empty
  
	uint8_t ret = *(pRingBuff->rp++);
	if( pRingBuff->rp == pRingBuff->pEnd ) {
		pRingBuff->rp -= pRingBuff->length; // Rewind pointer when exceeds bound
	}
	return ret;
}

/**
  * @brief  获取缓冲区尚未处理的字节数
  * @param  pRingBuff：待处理的ringbuffer   
  * @note   
  * @retval 待处理的字节数
  * @author Acorss工作室	
  */
inline uint16_t rbGetCount(const RingBuffer* pRingBuff)
{
	return (pRingBuff->wp - pRingBuff->rp + pRingBuff->length) % pRingBuff->length;
}

/**
  * @brief  判断缓冲区是否为空
  * @param  pRingBuff：待处理的ringbuffer   
  * @note   
  * @retval 空为1；否则为0
  * @author Acorss工作室	
  */
inline int8_t rbIsEmpty(const RingBuffer* pRingBuff)
{
	return pRingBuff->wp == pRingBuff->rp; 
}

/**
  * @brief  判断缓冲区是否空
  * @param  pRingBuff：待处理的ringbuffer   
  * @note   
  * @retval 满为1；否则为0
  * @author Acorss工作室	
  */
inline int8_t rbIsFull(const RingBuffer* pRingBuff)
{
 	return (pRingBuff->rp - pRingBuff->wp + pRingBuff->length - 1) % pRingBuff->length == 0;
}


#ifdef __cplusplus
}
#endif

#endif 

