#ifndef _ZYNQ_OCM_INTERNAL_H_
#define _ZYNQ_OCM_INTERNAL_H_

#ifndef __linux__
#include "xparameters.h"
#include "xil_cache.h"
#endif
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pilot_print.h"
/*
 * OCM block 0 range: 0xfffc0000 - 0xfffcffff size 64KB
 * We abstract out 15 message channels on ocm
 * The First 1KB space store each channel's ringbuffer class with 64 bytes space:
 * 0xfffc0000 - 0xfffc003f  channel 1  ringbuffer class
 * 0xfffc0040 - 0xfffc007f  channel 2  ringbuffer class
 * 0xfffc0080 - 0xfffc00bf  channel 3  ringbuffer class
 * 0xfffc00c0 - 0xfffc00ff  channel 4  ringbuffer class
 * 0xfffc0100 - 0xfffc013f  channel 5  ringbuffer class
 * 0xfffc0140 - 0xfffc017f  channel 6  ringbuffer class
 * 0xfffc0180 - 0xfffc01bf  channel 7  ringbuffer class
 * 0xfffc01c0 - 0xfffc01ff  channel 8  ringbuffer class
 * 0xfffc0200 - 0xfffc023f  channel 9  ringbuffer class
 * 0xfffc0240 - 0xfffc027f  channel 10 ringbuffer class
 * 0xfffc0280 - 0xfffc02bf  channel 11 ringbuffer class
 * 0xfffc02c0 - 0xfffc02ff  channel 12 ringbuffer class
 * 0xfffc0300 - 0xfffc033f  channel 13 ringbuffer class
 * 0xfffc0340 - 0xfffc037f  channel 14 ringbuffer class
 * 0xfffc0380 - 0xfffc03bf  channel 15 ringbuffer class
 *
 * Each channel has 4KB space as data ringbuffer
 * 0xfffc0400 - 0xfffc13ff  channel 1  ringbuffer
 * 0xfffc1400 - 0xfffc23ff  channel 2  ringbuffer
 * 0xfffc2400 - 0xfffc33ff  channel 3  ringbuffer
 * 0xfffc3400 - 0xfffc43ff  channel 4  ringbuffer
 * 0xfffc4400 - 0xfffc53ff  channel 5  ringbuffer
 * 0xfffc5400 - 0xfffc63ff  channel 6  ringbuffer
 * 0xfffc6400 - 0xfffc73ff  channel 7  ringbuffer
 * 0xfffc7400 - 0xfffc83ff  channel 8  ringbuffer
 * 0xfffc8400 - 0xfffc93ff  channel 9  ringbuffer
 * 0xfffc9400 - 0xfffca3ff  channel 10 ringbuffer
 * 0xfffca400 - 0xfffcb3ff  channel 11 ringbuffer
 * 0xfffcb400 - 0xfffcc3ff  channel 12 ringbuffer
 * 0xfffcc400 - 0xfffcd3ff  channel 13 ringbuffer
 * 0xfffcd400 - 0xfffce3ff  channel 14 ringbuffer
 * 0xfffce400 - 0xfffcf3ff  channel 15 ringbuffer
 */

#define OCM_MSG_CHN_NUM                   (15) //15 channels
#define OCM_MSG_CLASS_BASE                (0xfffc0000)
#define OCM_MSG_CLASS_LEN                 (0x40)  //64 bytes

#define OCM_MSG_BUF_OFFSET                (0x400) //1K
#define OCM_MSG_BUF_BASE(b)                  (b + OCM_MSG_BUF_OFFSET) //base address of ringbuffers 0xfffc0400
#define OCM_MSG_BUF_LEN                   (0x1000)  //one channel with 4KB space ringbuffer

  
/*
 * Cpu0 as master, Cpu1 as slave
 * MOSI(master out slave in) master send data to this buffer, and slave receive data from this buffer
 * MISO(master in slave out) master receive data to this buffer, and slave send data from this buffer
 */
enum{
    BUF_MOSI,
    BUF_MISO,
    BUF_MAX
};

typedef struct {
    volatile uint8_t   *base_addr;
    volatile uint32_t  *buf_len;        //store in ocm
	volatile uint32_t  *head;//read point, store in ocm
	volatile uint32_t  *tail;//write point,store in ocm
    volatile uint32_t  *full_count;//store in ocm
}ocm_ringbuf_t;

typedef struct {
    volatile uint8_t   *base_addr;
    volatile uint32_t  buf_len;
    volatile uint32_t  head;
    volatile uint32_t  tail;
    volatile uint32_t  full_count;
}ringbuf_t;

typedef struct {
    char *owner;
    volatile ocm_ringbuf_t buf[BUF_MAX];//ocm ringbuf class
}ocm_chn_t;

#endif
