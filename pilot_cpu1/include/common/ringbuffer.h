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

typedef struct ringbuffer
{
	unsigned			item_num;
	size_t		item_size;
	uint8_t				*_buf;
	volatile unsigned	_head;
	volatile unsigned	_tail;
}ringbuf_t;

static inline int ringbuf_init(ringbuf_t *ringbuf, unsigned item_num, unsigned item_size)
{
	ringbuf->item_num = item_num;
	ringbuf->item_size = item_size;
	ringbuf->_head = 0;
	ringbuf->_tail = 0;
	ringbuf->_buf = (uint8_t *)pvPortMalloc((item_num+1) * item_size);
	if(ringbuf->_buf == NULL)
	{
		return -1;
	}
    memset(ringbuf->_buf, 0, (item_num+1) * item_size);
	
	return 0;
}

static inline int ringbuf_deinit(ringbuf_t *ringbuf)
{
	ringbuf->item_num = 0;
	ringbuf->item_size = 0;
	ringbuf->_head = 0;
	ringbuf->_tail = 0;
    
    if(ringbuf != NULL)
    	vPortFree(ringbuf->_buf);
	
    return 0;
}

static inline unsigned _next(ringbuf_t *ringbuf, unsigned index)
{
	return (ringbuf->item_num - 1 == index) ? 0 : (index + 1);
}

//从buffer读数据，如果buf为空，则只移动指针，不拷数据，相当于丢掉最旧的数据
static inline bool ringbuf_get(ringbuf_t *ringbuf,void *buf, size_t get_len)
{
	unsigned candidate;
	unsigned next;

	//buffer非空
	if(ringbuf->_tail != ringbuf->_head)
	{
		if((get_len == 0) || (get_len > ringbuf->item_size))
		{
			get_len = ringbuf->item_size;
		}

		do
		{
			candidate = ringbuf->_tail;
			
			next = _next(ringbuf, candidate);

			if(buf != NULL)	
			{
				memcpy(buf, &ringbuf->_buf[candidate * ringbuf->item_size], get_len);
			}

				//GCC提供的原子操作
		}while(!__sync_bool_compare_and_swap(&ringbuf->_tail, candidate, next));
		//pilot_warn("tail=%d\n", ringbuf->_tail);
		return 0;
	}
	else
	{
		return -1;
	}
}

static inline bool ringbuf_put(ringbuf_t *ringbuf,const void *buf, size_t put_len)
{
	unsigned next = _next(ringbuf, ringbuf->_head);

	if(next != ringbuf->_tail)
	{
		if((put_len == 0) || (put_len > ringbuf->item_size))
		{
			put_len = ringbuf->item_size;
		}

		memcpy(&ringbuf->_buf[ringbuf->_head * ringbuf->item_size], buf, put_len);
		ringbuf->_head = next;
		//pilot_warn("head=%d\n", ringbuf->_head);
		return 0;
	}
	else
	{
		return -1;
	}
}

static inline int ringbuf_available(ringbuf_t *ringbuf)
{
	unsigned tail, head;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		head = ringbuf->_head;
		tail = ringbuf->_tail;
	} while (head != ringbuf->_head);


	return (tail <= head) ? (head - tail) : (ringbuf->item_num - tail + head);	
}

static inline int ringbuf_space(ringbuf_t *ringbuf)
{
	unsigned tail, head;

	/*
	 * Make a copy of the head/tail pointers in a fashion that
	 * may err on the side of under-estimating the free space
	 * in the buffer in the case that the buffer is being updated
	 * asynchronously with our check.
	 * If the head pointer changes (reducing space) while copying,
	 * re-try the copy.
	 */
	do {
		head = ringbuf->_head;
		tail = ringbuf->_tail;
	} while (head != ringbuf->_head);

	return (tail > head) ? (tail - head - 1) : (ringbuf->item_num - head + tail - 1);	
}

static inline bool ringbuf_empty(ringbuf_t *ringbuf)
{
    return ringbuf->_head == ringbuf->_tail;
}

static inline void ringbuf_flush(ringbuf_t *ringbuf)
{

	while (!ringbuf_empty(ringbuf)) {
		ringbuf_get(ringbuf, NULL, 0);
	}
    return ;
}

static inline bool ringbuf_force(ringbuf_t *ringbuf,const void *buf, size_t put_len)
{

	bool overwrote = false;

	for (;;) {
		if (ringbuf_put(ringbuf, buf, put_len) == 0) {
			break;
		}

        //删除buffer中最旧的数据，给新数据腾空间
		ringbuf_get(ringbuf, NULL, 0);
		overwrote = true;
	}

	return overwrote;
}

static inline int ringbuf_size(ringbuf_t *ringbuf)
{

    return (ringbuf->_buf != NULL) ? ringbuf->item_num : 0;
}

static inline bool ringbuf_resize(ringbuf_t *ringbuf, int iNewSize)
{
	uint8_t *old_buf;
	uint8_t *new_buf = (uint8_t *)pvPortMalloc((iNewSize + 1) * ringbuf->item_size);

	if (new_buf == NULL) {
		return false;
	}

    memset(new_buf, 0, (iNewSize + 1) * ringbuf->item_size);

	old_buf = ringbuf->_buf;
	ringbuf->_buf = new_buf;
	ringbuf->item_num = iNewSize;
	ringbuf->_head = 0;
	ringbuf->_tail = 0;
    vPortFree(old_buf);
	return true;
}

static inline void ringbuf_printinfo(ringbuf_t *ringbuf, const char *name)
{
	pilot_info("%s	%u/%lu (%u/%u @ %p)\n",
	       name,
	       ringbuf->item_num,
	       (unsigned long)ringbuf->item_num*ringbuf->item_size,
	       ringbuf->_head,
	       ringbuf->_tail,
	       ringbuf->_buf);
}

#ifdef __cplusplus
}
#endif
#endif
