#include "driver.h"
#include "board_config.h"
#include "xil_cache.h"

static off_t ocm_buf_offset = 0;
SemaphoreHandle_t storage_mutex;

int storage_close(Handle_t *handle)
{
    xSemaphoreTake(storage_mutex, portMAX_DELAY);
    ocm_buf_offset = 0;
    xSemaphoreGive(storage_mutex);

    return 0;
}

size_t storage_read(Handle_t *handle, char *buf, size_t len)
{
    size_t read_len = len;

    xSemaphoreTake(storage_mutex, portMAX_DELAY);

    if(ocm_buf_offset+len > HAL_STORAGE_SIZE)
    {
        read_len = HAL_STORAGE_SIZE - ocm_buf_offset;
    }

    memcpy(buf, (const void *)(OCM_STORAGE_START_ADDR+ocm_buf_offset), read_len);
    ocm_buf_offset += read_len;
//    if(len == 4||len==128)
//    Print_Warn("read from ocm:%x %x %x %x ocm_buf_offset=%d %x\n", buf[0], buf[1], buf[2], buf[3], ocm_buf_offset, *OCM_STORAGE_START);
    xSemaphoreGive(storage_mutex);
 
    return read_len;
}

size_t storage_write(Handle_t *handle, const char *buf, size_t len)
{
    size_t write_len = len;

    xSemaphoreTake(storage_mutex, portMAX_DELAY);

    if(ocm_buf_offset+len > HAL_STORAGE_SIZE)
    {
        write_len = HAL_STORAGE_SIZE - ocm_buf_offset;
    }

   /* int i;
    for(i = 0; i < write_len; i++)
        Print_Info("write:%x \n", buf[i]);
*/
    memcpy((void *)(OCM_STORAGE_START_ADDR+ocm_buf_offset), (const void *)buf, write_len);
	Xil_DCacheFlushRange(OCM_STORAGE_START_ADDR+ocm_buf_offset, write_len);//必须flush，不然真正的ocm中的值不会变
   
    COMM_TX_FLAG = COMM_TX_FLAG + 1;
	Xil_DCacheFlushRange(COMM_TX_FLAG_ADDR, 4);//必须flush，不然真正的ocm中的值不会变
    ocm_buf_offset += write_len;
    xSemaphoreGive(storage_mutex);

   return write_len;
}

int storage_lseek(Handle_t *handle, off_t offset, int whence)
{
    xSemaphoreTake(storage_mutex, portMAX_DELAY);

    switch(whence)
    {
        case SEEK_SET:
            ocm_buf_offset = offset;
            break;
        case SEEK_CUR:
        case SEEK_END:
            ocm_buf_offset += offset;
            break;
        default:
            return -1;
    }
    
    if(ocm_buf_offset < 0)
        ocm_buf_offset = 0;
    else if(ocm_buf_offset >= HAL_STORAGE_SIZE)
        ocm_buf_offset = HAL_STORAGE_SIZE - 1;//不允许超出长度

    xSemaphoreGive(storage_mutex);
  
    return ocm_buf_offset;
        
}

const DriverOps_t storage_ops = {
    .close = storage_close,
	.read = storage_read,
	.write = storage_write,
	.lseek = storage_lseek,
};

void storage_init()
{
    storage_mutex = xSemaphoreCreateMutex();
	DriverRegister(MTD_PARAMS_FILE, &storage_ops, NULL);
}
