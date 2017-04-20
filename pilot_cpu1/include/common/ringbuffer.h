#ifndef _RINGBUFFER_H_
#define _RINGBUFFER_H_

#ifdef __cplusplus
extern "C"{
#endif

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include "stdint.h"
#include "pilot_print.h"
#include "FreeRTOS.h"

typedef struct xRingBuffer
{
	unsigned			_NumItems;
	size_t		_ItemSize;
	uint8_t				*_ucBuf;
	volatile unsigned	_Head;
	volatile unsigned	_Tail;
}RingBuffer_t;

static inline int iRingBufferInit(RingBuffer_t *pxRingBuf, unsigned xNumItems, unsigned xItemSize)
{
	pxRingBuf->_NumItems = xNumItems;
	pxRingBuf->_ItemSize = xItemSize;
	pxRingBuf->_Head = 0;
	pxRingBuf->_Tail = 0;
	pxRingBuf->_ucBuf = (uint8_t *)pvPortMalloc((xNumItems+1) * xItemSize);
	if(pxRingBuf->_ucBuf == NULL)
	{
		return -1;
	}
    memset(pxRingBuf->_ucBuf, 0, (xNumItems+1) * xItemSize);
	
	return 0;
}

static inline int iRingBufferDeInit(RingBuffer_t *pxRingBuf)
{
	pxRingBuf->_NumItems = 0;
	pxRingBuf->_ItemSize = 0;
	pxRingBuf->_Head = 0;
	pxRingBuf->_Tail = 0;
    
    if(pxRingBuf != NULL)
    	vPortFree(pxRingBuf->_ucBuf);
	
    return 0;
}

static inline unsigned _xNext(RingBuffer_t *pxRingBuf, unsigned xIndex)
{
	return (pxRingBuf->_NumItems - 1 == xIndex) ? 0 : (xIndex + 1);
}

//从buffer读数据，如果vBuf为空，则只移动指针，不拷数据，相当于丢掉最旧的数据
static inline bool xRingBufferGet(RingBuffer_t *pxRingBuf,void *vBuf, size_t xGetLen)
{
	unsigned xCandidate;
	unsigned xNext;

	//buffer非空
	if(pxRingBuf->_Tail != pxRingBuf->_Head)
	{
		if((xGetLen == 0) || (xGetLen > pxRingBuf->_ItemSize))
		{
			xGetLen = pxRingBuf->_ItemSize;
		}

		do
		{
			xCandidate = pxRingBuf->_Tail;
			
			xNext = _xNext(pxRingBuf, xCandidate);

			if(vBuf != NULL)	
			{
				memcpy(vBuf, &pxRingBuf->_ucBuf[xCandidate * pxRingBuf->_ItemSize], xGetLen);
			}

				//GCC提供的原子操作
		}while(!__sync_bool_compare_and_swap(&pxRingBuf->_Tail, xCandidate, xNext));
		//pilot_warn("tail=%d\n", pxRingBuf->_Tail);
		return 0;
	}
	else
	{
		return -1;
	}
}

static inline bool xRingBufferPut(RingBuffer_t *pxRingBuf,const void *vBuf, size_t xPutLen)
{
	unsigned xNext = _xNext(pxRingBuf, pxRingBuf->_Head);

	if(xNext != pxRingBuf->_Tail)
	{
		if((xPutLen == 0) || (xPutLen > pxRingBuf->_ItemSize))
		{
			xPutLen = pxRingBuf->_ItemSize;
		}

		memcpy(&pxRingBuf->_ucBuf[pxRingBuf->_Head * pxRingBuf->_ItemSize], vBuf, xPutLen);
		pxRingBuf->_Head = xNext;
		//pilot_warn("head=%d\n", pxRingBuf->_Head);
		return 0;
	}
	else
	{
		return -1;
	}
}

static inline int iRingBufferGetAvailable(RingBuffer_t *pxRingBuf)
{
	unsigned xTail, xHead;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		xHead = pxRingBuf->_Head;
		xTail = pxRingBuf->_Tail;
	} while (xHead != pxRingBuf->_Head);


	return (xTail <= xHead) ? (xHead - xTail) : (pxRingBuf->_NumItems - xTail + xHead);	
}

static inline int iRingBufferGetSpace(RingBuffer_t *pxRingBuf)
{
	unsigned xTail, xHead;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		xHead = pxRingBuf->_Head;
		xTail = pxRingBuf->_Tail;
	} while (xHead != pxRingBuf->_Head);

	return (xTail > xHead) ? (xTail - xHead - 1) : (pxRingBuf->_NumItems - xHead + xTail - 1);	
}

static inline bool xRingBufferEmpty(RingBuffer_t *pxRingBuf)
{
    return pxRingBuf->_Head == pxRingBuf->_Tail;
}

static inline void vRingBufferFlush(RingBuffer_t *pxRingBuf)
{

	while (!xRingBufferEmpty(pxRingBuf)) {
		xRingBufferGet(pxRingBuf, NULL, 0);
	}
    return ;
}

static inline bool xRingBufferForce(RingBuffer_t *pxRingBuf,const void *vBuf, size_t xPutLen)
{

	bool overwrote = false;

	for (;;) {
		if (xRingBufferPut(pxRingBuf, vBuf, xPutLen) == 0) {
			break;
		}

        //删除buffer中最旧的数据，给新数据腾空间
		xRingBufferGet(pxRingBuf, NULL, 0);
		overwrote = true;
	}

	return overwrote;
}

static inline int iRingBufferSize(RingBuffer_t *pxRingBuf)
{

    return (pxRingBuf->_ucBuf != NULL) ? pxRingBuf->_NumItems : 0;
}

static inline bool xRingBufferResize(RingBuffer_t *pxRingBuf, int iNewSize)
{
	uint8_t *OldBuffer;
	uint8_t *NewBuffer = (uint8_t *)pvPortMalloc((iNewSize + 1) * pxRingBuf->_ItemSize);

	if (NewBuffer == NULL) {
		return false;
	}

    memset(NewBuffer, 0, (iNewSize + 1) * pxRingBuf->_ItemSize);

	OldBuffer = pxRingBuf->_ucBuf;
	pxRingBuf->_ucBuf = NewBuffer;
	pxRingBuf->_NumItems = iNewSize;
	pxRingBuf->_Head = 0;
	pxRingBuf->_Tail = 0;
    vPortFree(OldBuffer);
	return true;
}

static inline void vRingBufferPrintInfo(RingBuffer_t *pxRingBuf, const char *pcName)
{
	pilot_info("%s	%u/%lu (%u/%u @ %p)\n",
	       pcName,
	       pxRingBuf->_NumItems,
	       (unsigned long)pxRingBuf->_NumItems*pxRingBuf->_ItemSize,
	       pxRingBuf->_Head,
	       pxRingBuf->_Tail,
	       pxRingBuf->_ucBuf);
}

#ifdef __cplusplus
}
#endif
#endif
