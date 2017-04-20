/**************************** Include Files *******************************/

#include <stdlib.h>
#include <string.h>
#include "xparameters.h"
#include "xuartps.h"
#include "xil_exception.h"
#include "pilot_print.h"
#include "ringbuffer.h"
#include "gic/zynq_gic.h"
#include <fs/fs.h>
#include <fs/ioctl.h>
#include "driver.h"

/************************** Constant Definitions **************************/

/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */

//串口时钟使能
#define UART_CLK_CTRL		(*(volatile unsigned long *)(0xf8000154))
#define UART0_CLK_ENABLE	(1 << 0)
#define UART1_CLK_ENABLE	(1 << 1)

//外设时钟使能
#define APER_CLK_CTRL		(*(volatile unsigned long *)(0xf800012c)) //AMBA Peripheral Clock Control
#define UART0_CPU_1XCLKACT	(1 << 20)
#define UART1_CPU_1XCLKACT	(1 << 21)

//硬件中断号，来自xparameters_ps.h
static uint32_t uart_intr[XPAR_XUARTPS_NUM_INSTANCES] = {
	XPAR_XUARTPS_0_INTR,//59
	XPAR_XUARTPS_1_INTR,//82
};

typedef struct{
	int32_t uartps_id;	//uart编号
	XUartPs *uartps_inst_ptr;
	int32_t uartps_mode;
	RingBuffer_t rx_ringbuf;
	RingBuffer_t tx_ringbuf;
    SemaphoreHandle_t rx_mutex;
    SemaphoreHandle_t tx_mutex;
}uartps_private_t;

/************************** Function Prototypes *****************************/
static void uartps_isr(void *arg);

static int uartps_open(struct file *filp);

static ssize_t uartps_read(struct file *filp, char *buffer, size_t buflen);

static ssize_t uartps_write(struct file *filp, const char *buffer, size_t buflen);

static int uartps_ioctl(file_t *filp, int cmd, unsigned long arg);

/************************** Variable Definitions ***************************/

static XUartPs uartps[XPAR_XUARTPS_NUM_INSTANCES];		/* Instance of the UART Device */

const struct file_operations uartps_ops = {
	.read = uartps_read,
	.write = uartps_write,
	.ioctl = uartps_ioctl,
};

//串口时钟初始化，虽然fsbl阶段会配置，但是liunx内核启动后会还原这些寄存器，所以在cpu1再配一次
uint32_t uartps_clk_init(int32_t uartps_id)
{
	uint32_t val;

	if(uartps_id == 0)
	{
		//串口时钟
		val = UART_CLK_CTRL;
		val |= UART0_CLK_ENABLE;
		UART_CLK_CTRL = val;

		//外设时钟
		val = APER_CLK_CTRL;
		val |= UART0_CPU_1XCLKACT;
		APER_CLK_CTRL = val;
	}
	else if(uartps_id == 1)
	{
		//串口时钟
		val = UART_CLK_CTRL;
		val |= UART1_CLK_ENABLE;
		UART_CLK_CTRL = val;

		//外设时钟
		val = APER_CLK_CTRL;
		val |= UART1_CPU_1XCLKACT;
		APER_CLK_CTRL = val;
	}

//	pilot_info("Uart%d UART_CLK_CTRL=%x APER_CLK_CTRL=%x\n", uartps_id, UART_CLK_CTRL, APER_CLK_CTRL);

	return 0;
}

//arm核有两个uart，uart1用于console，uart0用于gps，这里只初始化usrt0，uart1由linux初始化
uint32_t uartps_init(int32_t uartps_id)
{
	uint32_t status;
	XUartPs_Config *config;
	uint32_t intr_mask;
	XUartPs *inst_ptr = &uartps[uartps_id];
	uartps_private_t *priv = NULL;
	int intr_id = uart_intr[uartps_id];
    char dev_name[64] = {0};

	if(uartps_id >= XPAR_XUARTPS_NUM_INSTANCES)
	{
		pilot_err("No such uart:%d, check vivado config\n", uartps_id);
		return XST_FAILURE;
	}

	/*
	 * step1: 初始化私有数据，其中包括收发数据缓冲区
	 */
	priv = (uartps_private_t *)pvPortMalloc(sizeof(uartps_private_t));
	if(priv == NULL)
	{
		pilot_err("malloc failed!!\n");
		return XST_FAILURE;
	}
	else
	{
		memset(priv, 0, sizeof(uartps_private_t));
	}
	priv->uartps_id = uartps_id;
	priv->uartps_inst_ptr = inst_ptr;
	iRingBufferInit(&priv->rx_ringbuf, 512, sizeof(char));
	iRingBufferInit(&priv->tx_ringbuf, 512, sizeof(char));
    priv->rx_mutex = xSemaphoreCreateMutex();
    priv->tx_mutex = xSemaphoreCreateMutex();

	/*
	 * step2: 使能串口时钟
	 */
	uartps_clk_init(uartps_id);

	/*
	 * step3: 初始化SDK中的串口配置
	 */
	config = XUartPs_LookupConfig(uartps_id);
	if (NULL == config) {
		pilot_err("There is no config\n");
		return XST_FAILURE;
	}

	status = XUartPs_CfgInitialize(inst_ptr, config, config->BaseAddress);
	if (status != XST_SUCCESS) {
		pilot_err("Uart cfg init failed:%d\n", status);
		return XST_FAILURE;
	}
#if 0
	status = XUartPs_SelfTest(inst_ptr);
	if (status != XST_SUCCESS) {
		pilot_err("Uart self test failed:%d\n", status);
		return XST_FAILURE;
	} 
	else
		pilot_info("self test success\n");
#endif

	/*
	 * step4: 指定uart中断指向哪个CPU
	 */
	gic_bind_interrupt_to_cpu(intr_id, CPU_1_TARGETED);
	
	/*
	 * step5: 注册串口中断处理函数，SDK的代码不通用，这里自己实现
	 */
	gic_isr_register(intr_id,(Xil_ExceptionHandler) uartps_isr,(void *) priv);
	if(status != XST_SUCCESS)
	{
		pilot_err("Set Uart%d interrupt handler failed:%d\n", uartps_id, status);
		return XST_FAILURE;
	}
	/*
	 * step6: 配置串口中断掩码
	 */
	intr_mask =
		XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
		XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
		XUARTPS_IXR_RXOVR;
	XUartPs_SetInterruptMask(inst_ptr, intr_mask);

	/*
	 * setp7: 配置串口模式为普通模式
	 */
	priv->uartps_mode = XUARTPS_OPER_MODE_NORMAL;
	XUartPs_SetOperMode(inst_ptr, XUARTPS_OPER_MODE_NORMAL);

	/*
	 * step8: 注册串口驱动
	 */
    snprintf(dev_name, sizeof(dev_name), "%s%d", "/dev/uart", (int)uartps_id);
    register_driver(dev_name, &uartps_ops, 0666, priv);

	/*
	 * step9: 使能串口中断
	 */
	gic_interrupt_enable(intr_id);
	
	return XST_SUCCESS;
}

ssize_t uartps_read(struct file *filp, char *buffer, size_t buflen)
{
	size_t len = 0;
	uartps_private_t *priv = (uartps_private_t *)filp->f_inode->i_private;

    irqstate_t state = irqsave();
	while(len < buflen)
	{
//        pilot_info("head=%x tail=%x\n", priv->rx_ringbuf._Head, priv->rx_ringbuf._Tail); 
		if(xRingBufferGet(&priv->rx_ringbuf, &buffer[len], priv->rx_ringbuf._ItemSize) == 0)
        {
			len++;
        }
        else
            break;
	}
    irqrestore(state);
	
	return len;
}

ssize_t uartps_write(struct file *filp, const char *buffer, size_t buflen)
{
	size_t len = 0;
	uartps_private_t *priv = (uartps_private_t *)filp->f_inode->i_private;
	XUartPs *inst_ptr = priv->uartps_inst_ptr;
    uint8_t val;

    /*
     * Mutex with interrupt
     */
    irqstate_t state = irqsave();
//    xSemaphoreTake(priv->tx_mutex, portMAX_DELAY);
	while (len < buflen) 
	{
		
        if(xRingBufferPut(&priv->tx_ringbuf, &buffer[len], sizeof(uint8_t)) == 0)
            len++;
        else
            break;
#if 0
		if(!XUartPs_IsTransmitFull(inst_ptr->Config.BaseAddress))
		{
			/*
			 * Fill the FIFO from the buffer
			 */
			XUartPs_WriteReg(inst_ptr->Config.BaseAddress,
					   XUARTPS_FIFO_OFFSET,
					   pcBuf[len]);

			len++;
		}
#endif
	}

    /*
     * If fifo is empty, we need to write one byte to tx fifo, to trigger tx fifo empty interrupt
     * so that interrupt can send the rest data to fifo
     */
    if(XUartPs_IsTransmitEmpty(inst_ptr))
    {
        if(xRingBufferGet(&priv->tx_ringbuf, &val, sizeof(uint8_t)) == 0)
        {
            /*
             * Fill the FIFO from the buffer
             */
            XUartPs_WriteReg(inst_ptr->Config.BaseAddress,
                       XUARTPS_FIFO_OFFSET,
                       val);
        }
        
    }
 //   xSemaphoreGive(priv->tx_mutex);
    irqrestore(state);

	return len;
}

int uartps_ioctl(file_t *filp, int cmd, unsigned long arg)
{
	int32_t ret = 0;
	uartps_private_t *priv = (uartps_private_t *)filp->f_inode->i_private;
	switch(cmd)
	{
		case FIONREAD:
		{
			int available= 0;

            irqstate_t state = irqsave();
			available= iRingBufferGetAvailable(&priv->rx_ringbuf);
			*(int *)arg = available;
            irqrestore(state);
			break;
		}
		case FIONWRITE:
		{
			int space= 0;

            irqstate_t state = irqsave();
			space = iRingBufferGetSpace(&priv->tx_ringbuf);
			*(int *)arg = space;
            irqrestore(state);
			break;
		}
		case UART_IOC_SET_MODE:
		{
            if((uint32_t *)arg == NULL)
            {
                pilot_err("Invald Param!!\n");
                return -1;
            }
		
            uint32_t uartps_mode = *(uint32_t *)(arg);

            if(uartps_mode == UART_MODE_LOOP)
            {
                uartps_mode = XUARTPS_OPER_MODE_LOCAL_LOOP; 
            }
            else
            {
                uartps_mode = XUARTPS_OPER_MODE_NORMAL; 
            }

			priv->uartps_mode = uartps_mode;
			XUartPs_SetOperMode(priv->uartps_inst_ptr, uartps_mode);
			break;
		}	
        case UART_IOC_GET_DATA_FORMAT:
        {
            if((UartDataFormat_t *)arg == NULL)
            {
                pilot_err("Invald Param!!\n");
                return -1;
            }
        
            XUartPsFormat *usr_format = (XUartPsFormat *)arg;

            //For ps uart, XUartPsFormat is equal to UartDataFormat_t
            XUartPs_SetDataFormat(priv->uartps_inst_ptr, usr_format);

            break;
        }
        case UART_IOC_SET_DATA_FORMAT:
        {
            if((XUartPsFormat *)arg == NULL)
            {
                pilot_err("Invald Param!!\n");
                return -1;
            }

            XUartPsFormat *drv_format = (XUartPsFormat *)arg;

            //For ps uart, XUartPsFormat is equal to UartDataFormat_t
            XUartPs_SetDataFormat(priv->uartps_inst_ptr, drv_format);
            break;
        }
		default:
			pilot_err("Unknow Command\n");
            return -1;
	}
	return 0;
}

static void uartps_isr(void *arg)
{
	u32 isr;
	uint8_t  val = 0;
	int32_t  ret = 0;
	uartps_private_t *priv = (uartps_private_t *)arg;
	XUartPs *inst_ptr = priv->uartps_inst_ptr;

	Xil_AssertVoid(inst_ptr != NULL);
	Xil_AssertVoid(inst_ptr->IsReady == XIL_COMPONENT_IS_READY);

    irqstate_t state = irqsave();
	/*
	 * Read the interrupt ID register to determine which
	 * interrupt is active
	 */
	isr = XUartPs_ReadReg(inst_ptr->Config.BaseAddress,
				   XUARTPS_IMR_OFFSET);

	isr &= XUartPs_ReadReg(inst_ptr->Config.BaseAddress,
				   XUARTPS_ISR_OFFSET);

	/*
	 * Handle interrupt 
	 */
	if(0 != (isr & (XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXEMPTY |
				 XUARTPS_IXR_RXFULL))) 
	{
		/* Recieved data interrupt */
		while(XUartPs_IsReceiveData(inst_ptr->Config.BaseAddress))
		{
            if(iRingBufferGetSpace(&priv->rx_ringbuf) > 0) 
            {
            	val = XUartPs_ReadReg(inst_ptr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
//            pilot_warn("isr=%x val=%d\n", isr, val);
			    ret = xRingBufferPut(&priv->rx_ringbuf, &val, sizeof(uint8_t));

            }
            else
            {
                /*
                 * 如果rx fifo和ringbuffer都满了，则强行写一个数据到ringbuffer，防止rx中断再也进不来
                 */
                if(isr & (XUARTPS_IXR_RXFULL)) 
                {
                    val = XUartPs_ReadReg(inst_ptr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
                    ret = xRingBufferForce(&priv->rx_ringbuf, &val, sizeof(uint8_t));
                }
                break;
            }

		}
	}


	if(0 != (isr & (XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_TXFULL))) 
	{
        portBASE_TYPE xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        
//        xSemaphoreTakeFromISR(priv->tx_mutex, &xHigherPriorityTaskWoken);
        /* Transmit data interrupt */
		while ((!XUartPs_IsTransmitFull(inst_ptr->Config.BaseAddress))) 
		{
			ret = xRingBufferGet(&priv->tx_ringbuf, &val, sizeof(uint8_t));
			
			if(ret == 0)
			{
				/*
				 * Fill the FIFO from the buffer
				 */
				XUartPs_WriteReg(inst_ptr->Config.BaseAddress,
						   XUARTPS_FIFO_OFFSET,
						   val);
			}
			else
				break;
			
		}
 //       xSemaphoreGiveFromISR(priv->tx_mutex, &xHigherPriorityTaskWoken);
	}

	if(0 != (isr & (XUARTPS_IXR_OVER | XUARTPS_IXR_FRAMING |
				XUARTPS_IXR_PARITY))) {
		/* Recieved Error status interrupt */
	}

	if(0 != (isr & XUARTPS_IXR_TOUT )) {
		/* Recieved Timeout interrupt */
	}

	if(0 != (isr & XUARTPS_IXR_DMS)) {
		/* Modem status interrupt */
	}

	/*
	 * Clear the interrupt status.
	 */
	XUartPs_WriteReg(inst_ptr->Config.BaseAddress, XUARTPS_ISR_OFFSET, isr);

    irqrestore(state);
}

