#include "ocm_internal.h"
#include "ocm.h"

#if 1
/*
 * Write 4 bytes to ocm
 */
static inline void ocm_write_dword(volatile uint32_t *addr, uint32_t value);
/*
 * Write bytes to ocm (more than 4 bytes)
 */
static inline int32_t ocm_write_bytes(volatile uint8_t *dest, uint8_t *src, uint32_t len);
/*
 * Read 4 bytes from ocm
 */
static inline uint32_t ocm_read_dword(volatile uint32_t *addr);
/*
 * Read bytes from ocm (more than 4 bytes)
 */
static inline uint32_t ocm_read_bytes(uint8_t *dest, volatile uint8_t *src, uint32_t len);

#define OCM_ALIGN(l)     (l % OCM_ALIGN_BYTES != 0)?\
                         l = (l / OCM_ALIGN_BYTES) * OCM_ALIGN_BYTES :\
                         l

#define OCM_BUF_AVAILABLE(buf)             ((buf->head > buf->tail)? \
                                            (buf->buf_len - buf->head) + buf->tail: \
                                            buf->tail - buf->head)

#define OCM_BUF_SPACE(buf)                 ((buf->head > (buf->tail))? \
                                            (buf->head - buf->tail) - 1: \
                                            (buf->buf_len - buf->tail) + buf->head - 1)

#define OCM_BUF_ADVANCETAIL(buf, n)         (buf->tail = (buf->tail+n) % buf->buf_len)
#define OCM_BUF_ADVANCEHEAD(buf, n)         (buf->head = (buf->head+n) % buf->buf_len)

 
static ocm_chn_t ocm_chn[OCM_MSG_CHN_NUM] = {{0}}; 

/*
 * Write 4 bytes to ocm
 */
static inline void ocm_write_dword(volatile uint32_t *addr, uint32_t value)
{
    *addr = value;

#if !defined(DISABLE_CACHE) && !defined(__linux__)
    Xil_DCacheFlushRange((INTPTR)addr, sizeof(uint32_t));
#endif
}

/*
 * Write bytes to ocm (more than 4 bytes)
 */
static inline int32_t ocm_write_bytes(volatile uint8_t *dest, uint8_t *src, uint32_t len)
{
    memcpy((void *)dest, (const void *)src, len);

#if !defined(DISABLE_CACHE) && !defined(__linux__)
    Xil_DCacheFlushRange((INTPTR)dest, len);
#endif

    return 0;
}

/*
 * Read 4 bytes from ocm
 */
static inline uint32_t ocm_read_dword(volatile uint32_t *addr)
{
#if !defined(DISABLE_CACHE) && !defined(__linux__)
    //invalid cahce data, make sure we read the latest value
    Xil_DCacheInvalidateRange((INTPTR)addr, sizeof(uint32_t));
#endif
    
    return *addr;
}

/*
 * Read bytes from ocm (more than 4 bytes)
 */
static inline uint32_t ocm_read_bytes(uint8_t *dest, volatile uint8_t *src, uint32_t len)
{
#if !defined(DISABLE_CACHE) && !defined(__linux__)
    //invalid cahce data, make sure we read the latest value
    Xil_DCacheInvalidateRange((INTPTR)src, len);
#endif
    
    memcpy((void *)dest, (const void *)src, len);

    return 0;
}

/*
 * 
 */
static inline void *ocm_get_class(int32_t chn, ringbuf_t *ringbuf, int32_t type)
{
    ocm_chn_t *p_chn = &ocm_chn[chn];
    volatile ocm_ringbuf_t *p_buf = &p_chn->buf[type];

    ringbuf->base_addr = p_buf->base_addr;
    ringbuf->buf_len = ocm_read_dword(p_buf->buf_len);
    ringbuf->head = ocm_read_dword(p_buf->head);
    ringbuf->tail = ocm_read_dword(p_buf->tail);
    ringbuf->full_count = ocm_read_dword(p_buf->full_count);

    return (void *)p_buf;
}

static inline int32_t ocm_set_class(void *class, ringbuf_t *ringbuf)
{
    volatile ocm_ringbuf_t *p_buf = (ocm_ringbuf_t *)class;

    ocm_write_dword(p_buf->head, ringbuf->head);
    ocm_write_dword(p_buf->tail, ringbuf->tail);
    ocm_write_dword(p_buf->full_count, ringbuf->full_count);
    
    return 0;
}

#if 0
int32_t ocm_msg_send(int32_t chn, char *data, int32_t len)
{
    ocm_chn_t *p_chn = &ocm_chn[chn];
    ocm_ringbuf_t *p_buf = &p_chn->buf[BUF_TX];
    uint32_t tmp_len = len, space = 0, offset = 0;

    while(tmp_len)
    {
        offset = tmp_len;
        space = BUF_SPACE(p_buf);
        if (space == 0) 
        {
            ringbuf->full_count++;
            return -1;
        }

        if(space < tmp_len){
            offset = space;
        }

        if (OCM_READ(p_buf->tail) < OCM_READ(p_buf->head)) {
            // perform as single memcpy
            if(data != NULL)
                memcpy(p_buf->base_addr+OCM_READ(p_buf->tail), data, offset);

            BUF_ADVANCETAIL(p_buf, offset);
            tmp_len -= offset;
            continue;
        }

        // perform as two memcpy calls
        uint32_t n = OCM_READ(p_buf->buf_len) - OCM_READ(p_buf->tail);
        if (n > offset) 
            n = offset;
    
        if(data != NULL)
            memcpy(p_buf->base_addr+OCM_READ(p_buf->tail), data, n);
        BUF_ADVANCETAIL(p_buf, n);
        data += n;
        n = offset - n;
        if (n > 0) {
            if(data != NULL)
                memcpy(p_buf->base_addr+OCM_READ(p_buf->tail), data, n);
            BUF_ADVANCETAIL(p_buf, n);
        }
        tmp_len -= offset;
    }
    
}
#endif

/*
 * len: length of data want to send
 * len < buffer space is defined as channel busy
 */
bool ocm_msg_busy(int32_t chn, int32_t len)
{
    int32_t space = 0;
    ringbuf_t ringbuf;
    ringbuf_t *p_ringbuf = &ringbuf;
     void *class;
#ifdef __linux__
    class = ocm_get_class(chn, p_ringbuf, BUF_MOSI);
#else
    class = ocm_get_class(chn, p_ringbuf, BUF_MISO);
#endif
   
    if(class == NULL)
        return true;

    space = OCM_BUF_SPACE(p_ringbuf);

    return space < len;
}

int32_t ocm_msg_send(int32_t chn, uint8_t *data, int32_t len)
{
    uint32_t tmp_len = len, space = 0;
    ringbuf_t ringbuf;
    ringbuf_t *p_ringbuf = &ringbuf;
    void *class;
#ifdef __linux__
    class = ocm_get_class(chn, p_ringbuf, BUF_MOSI);
#else
    class = ocm_get_class(chn, p_ringbuf, BUF_MISO);
#endif
    if(class == NULL)
    {
        pilot_err("get ocm p_ringbuffer class failed!!\n");
        return 0;
    }

    space = OCM_BUF_SPACE(p_ringbuf);
    //align with 4 bytes
    OCM_ALIGN(space);
    if (space == 0) 
    {
        p_ringbuf->full_count++;
        tmp_len = 0;
        goto set_and_out;
    }

    if(space < tmp_len){
        tmp_len = space;
    }

//    pilot_warn("base=%x tail=%d head=%d\n", p_ringbuf->base_addr, (p_ringbuf->tail), (p_ringbuf->head));
    if (p_ringbuf->tail < p_ringbuf->head) {
        // perform as single memcpy
        if(data != NULL)
            ocm_write_bytes(p_ringbuf->base_addr + p_ringbuf->tail, data, tmp_len);

        OCM_BUF_ADVANCETAIL(p_ringbuf, tmp_len);
        goto set_and_out;
    }

    // perform as two memcpy calls
    uint32_t n = p_ringbuf->buf_len - p_ringbuf->tail;
    if (n > tmp_len) 
        n = tmp_len;

    if(data != NULL)
        ocm_write_bytes(p_ringbuf->base_addr + p_ringbuf->tail, data, n);
    OCM_BUF_ADVANCETAIL(p_ringbuf, n);
    data += n;
    n = tmp_len - n;
    if (n > 0) {
        if(data != NULL)
            ocm_write_bytes(p_ringbuf->base_addr + p_ringbuf->tail, data, n);
        OCM_BUF_ADVANCETAIL(p_ringbuf, n);
    }
    
set_and_out:
    ocm_set_class(class, p_ringbuf);
    return tmp_len;
}

int32_t ocm_msg_recv(int32_t chn, uint8_t *data, int32_t len)
{
    uint32_t available = 0;
    uint32_t tmp_len = len;
    ringbuf_t ringbuf;
    ringbuf_t *p_ringbuf = &ringbuf;
    void *class;
#ifdef __linux__
    class = ocm_get_class(chn, p_ringbuf, BUF_MISO);
#else
    class = ocm_get_class(chn, p_ringbuf, BUF_MOSI);
#endif
    if(class == NULL)
    {
        pilot_err("get ocm ringbuffer class failed!!\n");
        return 0;
    }

//    pilot_info("base=%x tail=%d head=%d len=%d\n", p_ringbuf->base_addr, (p_ringbuf->tail), (p_ringbuf->head), (p_ringbuf->buf_len));
    available = OCM_BUF_AVAILABLE(p_ringbuf);
    //align with 4 bytes
    OCM_ALIGN(available);
    if(available == 0)
    {
        return 0;
    }

    if(available < tmp_len)
    {
        tmp_len = available;
    }

    if(p_ringbuf->tail > p_ringbuf->head)
    {
        ocm_read_bytes(data, p_ringbuf->base_addr + p_ringbuf->head, tmp_len);
        OCM_BUF_ADVANCEHEAD(p_ringbuf, tmp_len);
        goto set_and_out;
    }

    uint32_t n = p_ringbuf->buf_len - p_ringbuf->head;
    if(n > tmp_len) 
        n = tmp_len;

    ocm_read_bytes(data, p_ringbuf->base_addr + p_ringbuf->head, n);
    OCM_BUF_ADVANCEHEAD(p_ringbuf, n);
    data += n;
    n = tmp_len - n;
    if(n > 0)
    {
        ocm_read_bytes(data, p_ringbuf->base_addr + p_ringbuf->head, n);
        OCM_BUF_ADVANCEHEAD(p_ringbuf, n);
    }

set_and_out:
    ocm_set_class(class, p_ringbuf);
    return tmp_len;

}

/*
 * Linux on cpu0 call this api first
 */
int32_t ocm_msg_init()
{
    int i = 0, j = 0; 

    for(i=0; i<OCM_MSG_CHN_NUM; i++)
    {
        ocm_chn_t *p_chn = &ocm_chn[i];
#ifdef __linux__
        extern uint32_t ocm_get_vir_addr(uint32_t phy_addr);
        uint32_t base = 0;
        uint32_t addr = 0;
        if(base == 0)
            base = ocm_get_vir_addr(OCM_MSG_CLASS_BASE);

        addr = base + i * OCM_MSG_CLASS_LEN;
#else
        uint32_t addr = OCM_MSG_CLASS_BASE + i * OCM_MSG_CLASS_LEN;
#endif

        p_chn->owner = NULL;

        for(j=0; j<BUF_MAX; j++)
        {
            volatile ocm_ringbuf_t *p_buf = &p_chn->buf[j];

            /*Set ringbuffer class address in OCM*/
            p_buf->buf_len   = (volatile uint32_t *)addr;
            addr             += sizeof(p_buf->buf_len);

            p_buf->head      = (volatile uint32_t *)addr;
            addr             += sizeof(p_buf->head);

            p_buf->tail      = (volatile uint32_t *)addr;
            addr             += sizeof(p_buf->tail);

            p_buf->full_count= (volatile uint32_t *)addr;
            addr             += sizeof(p_buf->full_count);

            if(addr > OCM_MSG_CLASS_BASE + OCM_MSG_CLASS_LEN*OCM_MSG_CHN_NUM)
            {
                pilot_err("ocm ringbuf class too long:0x%x !!\n", (int)addr);
                return -1;
            }

            /*Init value in ringbuffer class*/
#ifndef __linux__
            p_buf->base_addr = (volatile uint8_t *)(OCM_MSG_BUF_BASE(OCM_MSG_CLASS_BASE)+i*OCM_MSG_BUF_LEN+j*OCM_MSG_BUF_LEN/BUF_MAX);
#else
            p_buf->base_addr = (volatile uint8_t *)(OCM_MSG_BUF_BASE(base)+i*OCM_MSG_BUF_LEN+j*OCM_MSG_BUF_LEN/BUF_MAX);

            ocm_write_dword(p_buf->buf_len,OCM_MSG_BUF_LEN/BUF_MAX);
            ocm_write_dword(p_buf->head, 0);
            ocm_write_dword(p_buf->tail, 0);
            ocm_write_dword(p_buf->full_count, 0);
#endif
        }
       
    }

    return 0;
}

/*
 * Find an available channel for user
 */
int32_t ocm_msg_chn_init(int32_t chn, char *owner)
{

    if(ocm_chn[chn].owner != NULL)
    {
        pilot_err("ocm channel confilct, chn%d has owned by %s\n", (int)chn, ocm_chn[chn].owner);
        return -1;
    }

    ocm_chn[chn].owner = owner;

    return 0;
}


//int OcmAdd()
//{
////	COMM_TX_FLAG ++;
//    memset(OCM_TX_MSG_START_ADDR, 0x11111111, sizeof(int));
//	Xil_DCacheFlushRange(OCM_TX_MSG_START_ADDR, 4);//必须flush，不然真正的ocm中的值不会变
//	return 0;
//}
#endif
