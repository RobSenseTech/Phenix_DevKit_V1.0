#include "xparameters.h"
#include <stdio.h>
#include "xil_cache.h"
#include "board_config.h"

#if 0
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
 * Each channel has 4KB space as data ringbuffer, and one ringbuffer split to 8 items, each item has 512 bytes space
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


struct fs_msg{
    char *cmd;
    char *buf;
    int32_t buf_len;
}

#define OCM_MSG_CHN_NUM                   (15) //15 channels
#define OCM_MSG_CLASS_BASE                (0xfffc0000)
#define OCM_MSG_CLASS_LEN                 (0x40)  //64 bytes

#define OCM_MSG_BUF_BASE                  (0xfffc0400) //base address  ringbuffers
#define OCM_MSG_BUF_LEN                   (0x1000)  //one channel with 4KB space ringbuffer
#define OCM_MSG_BUF_ITEM_SIZE             (0x200)   //one item in ringbuffer has 512 bytes space
#define OCM_MSG_BUF_ITEM_NUM              (OCM_MSG_BUF_LEN / OCM_MSG_BUF_ITEM_SIZE) //one ringbuffer combine with these items

typedef struct 
{
	volatile uint32_t  *num_items;
	volatile uint32_t  *item_size;
	volatile uint32_t  *base_addr;
	volatile uint32_t  *head;
	volatile uint32_t  *tail;
    volatile uint32_t  *full_count;
}ocm_ringbuf_t;

ocm_ringbuf_t chn_buf[OCM_MSG_CHN_NUM] = {0}; 

int32_t ocm_msg_init()
{
    int i = 0; 

    for(i=0; i<OCM_MSG_CHN_NUM; i++)
    {
        uint32_t *ptr = &chn_buf[i];
        /*Set ringbuffer class address in OCM*/
        ptr->num_items  = OCM_MSG_CLASS_BASE + i * OCM_MSG_CLASS_LEN;
        ptr->item_size  = ptr->num_items + sizeof(ptr->num_items);
        ptr->base_addr  = ptr->item_size + sizeof(ptr->item_size);
        ptr->head       = ptr->base_addr + sizeof(ptr->base_addr);
        ptr->tail       = ptr->head + sizeof(ptr->head);
        ptr->full_count = ptr->tail + sizeof(ptr->tail);

        /*Init value in ringbuffer class*/
        *ptr->num_items  = OCM_MSG_BUF_ITEM_NUM;
        *ptr->item_size  = OCM_MSG_BUF_ITEM_SIZE;
        memset(ptr->base_addr, 0, OCM_MSG_BUF_LEN);
        *ptr->head       = 0;
        *ptr->tail       = 0;
        *ptr->full_count = 0;
    }

}

ocm_transfer(char *data, int32_t datalen)
{
    
}

int OcmAdd()
{
//	COMM_TX_FLAG ++;
    memset(OCM_TX_MSG_START_ADDR, 0x11111111, sizeof(int));
	Xil_DCacheFlushRange(OCM_TX_MSG_START_ADDR, 4);//必须flush，不然真正的ocm中的值不会变
	return 0;
}
#endif
