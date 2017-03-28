/**************************** Include Files *******************************/

#include <stdlib.h>
#include <string.h>
#include "xparameters.h"
#include "xuartns550.h"
#include "xil_exception.h"
#include "FreeRTOS_Print.h"
#include "ringbuffer.h"
#include "gic/zynq_gic.h"
#include <fs/fs.h>
#include <fs/ioctl.h>
#include "driver.h"

#define XUN_IIR_MODEM           (0)//Modem Status
#define XUN_IIR_THRE            (2)//Transmitter Holding Register Empty
#define XUN_IIR_TIMOUT          (12)//Character Timeout
#define XUN_IIR_RDA             (4)//Received Data Available
#define XUN_IIR_RLS             (6)//Receiver Line Status

//interrupt id, from xparameters_ps.h
static uint32_t uartns_hw_intr[XPAR_XUARTNS550_NUM_INSTANCES] = {
    XPAR_FABRIC_AXI_UART16550_0_IP2INTC_IRPT_INTR,
    XPAR_FABRIC_AXI_UART16550_1_IP2INTC_IRPT_INTR,
    XPAR_FABRIC_AXI_UART16550_2_IP2INTC_IRPT_INTR,
    XPAR_FABRIC_AXI_UART16550_3_IP2INTC_IRPT_INTR,
};

typedef struct{
	int32_t uartns_id;	//uartns port id
	XUartNs550* uartns_inst_ptr;
	int32_t mode;
	RingBuffer_t rx_ringbuf;
	RingBuffer_t tx_ringbuf;
    SemaphoreHandle_t uartns_rxmutex;
    SemaphoreHandle_t uartns_txmutex;
}uartns_private_t;

XUartNs550 uartns_inst[XPAR_XUARTNS550_NUM_INSTANCES];	/* Instance of the UART Device */

void uartns_isr(void *arg);

ssize_t uartns_read(struct file *filp, char *buffer, size_t buflen);

ssize_t uartns_write(struct file *filp, const char *buffer, size_t buflen);

int uartns_ioctl(file_t *filp, int cmd, unsigned long arg);

const struct file_operations uartns_ops = {
	.read =  uartns_read,
	.write = uartns_write,
	.ioctl = uartns_ioctl,
};


uint32_t uartns_init(int32_t uartns_id)
{
	uint32_t status;
	XUartNs550 *uartns_inst_ptr = &uartns_inst[uartns_id];
	uartns_private_t *private = NULL;
	int32_t intr_id= uartns_hw_intr[uartns_id];
    uint16_t options = 0;
    char dev_name[64] = {0};

	if(uartns_id >= XPAR_XUARTNS550_NUM_INSTANCES)
	{
		Print_Err("No such uart lite:%d, check vivado config\n", uartns_id);
		return XST_FAILURE;
	}

	/*
	 * step1: init private data,including ringbuffer 
	 */
	private = (uartns_private_t*)pvPortMalloc(sizeof(uartns_private_t));
	if(private == NULL)
	{
		Print_Err("malloc failed!!\n");
		return XST_FAILURE;
	}
	else
	{
		memset(private, 0, sizeof(uartns_private_t));
	}

    private->uartns_id = uartns_id;
	private->uartns_inst_ptr= uartns_inst_ptr;
	iRingBufferInit(&private->rx_ringbuf, 512, sizeof(char));
	iRingBufferInit(&private->tx_ringbuf, 512, sizeof(char));
    private->uartns_rxmutex = xSemaphoreCreateMutex();
    private->uartns_txmutex = xSemaphoreCreateMutex();


	/*
	 * setp2: init uartns config int SDK
	 */
	status = XUartNs550_Initialize(uartns_inst_ptr, uartns_id);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built correctly.
	 */
	status = XUartNs550_SelfTest(uartns_inst_ptr);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/*
	 * step3: bind uartns interrupt to CPU1
	 */
	GicBindInterruptToCpu(intr_id, CPU_1_TARGETED);

	/*
	 * step4: set interrupt as rising edge triggered
	 */
	GicSetTriggerType(intr_id, INTR_TRIGGER_RAISING_EDGE);

    /*
	 * step5: register interrupt handler
	 */
	GicIsrHandlerRegister(intr_id,(Xil_ExceptionHandler) uartns_isr,(void *)private);
	if(status != XST_SUCCESS)
	{
		Print_Err("Set Uart%d interrupt handler failed:%d\n", uartns_id, status);
		return XST_FAILURE;
	}

    /*
     * step 6:Enable the rx interrupt of the UART so interrupts will occur, and keep the
	 * FIFOs enabled.
	 */
	options = XUN_OPTION_DATA_INTR | XUN_OPTION_FIFOS_ENABLE;
	XUartNs550_SetOptions(uartns_inst_ptr, options);

	/*
	 * step7: register uartns driver
	 */
    snprintf(dev_name, sizeof(dev_name), "%s%d", "/dev/uartns", (int)uartns_id);
    register_driver(dev_name, &uartns_ops, 0666, private);

	/*
	 * step8: Enable interrupt
	 */
	GicInterruptEnable(intr_id);

	return XST_SUCCESS;
}

void uartns_isr(void *arg)
{
	int32_t  ret = 0;
	uint8_t  val = 0;
	uint8_t isr_status;
    uint32_t iir = 0;
    uartns_private_t *private = (uartns_private_t *)arg;
	XUartNs550 *inst_ptr = private->uartns_inst_ptr;

	Xil_AssertVoid(inst_ptr!= NULL);

	/*
	 * Read the interrupt ID register to determine which, only one,
	 * interrupt is active
	 */
	isr_status = (uint8_t)XUartNs550_ReadReg(inst_ptr->BaseAddress, XUN_IIR_OFFSET) & XUN_INT_ID_MASK;

	iir = XUartNs550_ReadReg(inst_ptr->BaseAddress, XUN_IIR_OFFSET);
	/*
	 * Handle interrupt 
	 */
    if(isr_status & XUN_IIR_RDA)
    {
	    uint32_t lsr = 0; 
        uint8_t recv_bytes = 0;

		while(recv_bytes < iRingBufferGetSpace(&private->rx_ringbuf))
        {
            /*
             * Read the Line Status Register to determine if there is any
             * data in the receiver/FIFO
             */
		    lsr = XUartNs550_GetLineStatusReg(inst_ptr->BaseAddress);
            /*
             * If there is a break condition then a zero data byte was put
             * into the receiver, just read it and dump it and update the
             * stats
             */
            if (lsr & XUN_LSR_BREAK_INT) {
                (void)XUartNs550_ReadReg(inst_ptr->BaseAddress, XUN_RBR_OFFSET);
                //XUartNs550_UpdateStats(inst_ptr, (uint8_t)lsr);
            }

            /*
             * If there is data ready to be removed, then put the next byte
             * received into the specified buffer and update the stats to
             * reflect any receive errors for the byte
             */
            else if(lsr & XUN_LSR_DATA_READY) 
            {
                val = XUartNs550_ReadReg(inst_ptr->BaseAddress, XUN_RBR_OFFSET);
			    xRingBufferPut(&private->rx_ringbuf, &val, sizeof(uint8_t));
                //XUartNs550_UpdateStats(inst_ptr, (uint8_t)lsr);
                recv_bytes++;
            }
            /*
             * There's no more data buffered, so exit such that this
             * function does not block waiting for data
             */
            else
            {
                break;
            }
        }
    }

    if(isr_status & XUN_IIR_THRE)
    {
        int i = 0;
        uint8_t send_bytes = 0;
	    uint32_t lsr = XUartNs550_GetLineStatusReg(inst_ptr->BaseAddress);
        uint32_t ier = 0;
        uint16_t fifo_size = 0;
        uint32_t available = 0;

        /*
        * If the transmitter is not empty then don't send any data, the empty
        * room in the FIFO is not available
        */
        if(lsr & XUN_LSR_TX_BUFFER_EMPTY)
        {
            /*
             * Read the interrupt ID register to determine if FIFOs
             * are enabled
             */
            if (iir & XUN_INT_ID_FIFOS_ENABLED) 
            {
                /*
                 * Determine how many bytes can be sent depending on if
                 * the transmitter is empty, a FIFO size of N is really
                 * N - 1 plus the transmitter register
                 */
                if (lsr & XUN_LSR_TX_EMPTY) {
                    fifo_size = XUN_FIFO_SIZE;
                } else {
                    fifo_size = XUN_FIFO_SIZE - 1;
                }

                /*
                 * FIFOs are enabled, if the number of bytes to send
                 * is less than the size of the FIFO, then send all
                 * bytes, otherwise fill the FIFO
                 */
                available = iRingBufferGetAvailable(&private->tx_ringbuf);
                if (available < fifo_size) 
                {
                    send_bytes = available;
                    /*
                     * Disable THRE interrupt for no data to transmit next time
                     */
                    ier = XUartNs550_ReadReg(inst_ptr->BaseAddress, XUN_IER_OFFSET);
                    XUartNs550_WriteReg(inst_ptr->BaseAddress, XUN_IER_OFFSET, ier & (~XUN_IER_TX_EMPTY));

                }
                else
                {
                    send_bytes = fifo_size;

                }

                for(i = 0; i < send_bytes; i++)
                {
                    ret = xRingBufferGet(&private->tx_ringbuf, &val, sizeof(uint8_t));
                    if(ret == 0)
                    {
                        XUartNs550_WriteReg(inst_ptr->BaseAddress, XUN_THR_OFFSET, val);
                    }
                    else
                    {
                        break;
                    }
                }
            }
        }
    }
}

ssize_t uartns_read(struct file *filp, char *buffer, size_t buflen)
{
	size_t read_len= 0;
	uartns_private_t *private = (uartns_private_t *)filp->f_inode->i_private;

	/*
	 * Enter a critical region
     */
    irqstate_t state = irqsave();

	while(read_len < buflen)
	{
//        Print_Info("head=%x tail=%x\n", pxUartDrvPrivate->xUartRxRingBuf._Head, pxUartDrvPrivate->xUartRxRingBuf._Tail); 
		if(xRingBufferGet(&private->rx_ringbuf, &buffer[read_len], sizeof(uint8_t)) == 0)
        {
			read_len++;
        }
        else
            break;
	}
	/*
	 * Restore the interrupt enable register to it's previous value such
	 * that the critical region is exited
	 */
    irqrestore(state);

	return read_len;

}

ssize_t uartns_write(struct file *filp, const char *buffer, size_t buflen)
{
    uint32_t ier = 0;
	size_t write_len = 0;
    uartns_private_t *private  = (uartns_private_t *)filp->f_inode->i_private;
	XUartNs550 *inst_ptr = private->uartns_inst_ptr;

	/*
	 * Enter a critical region by disabling all the UART interrupts to allow
	 * this call to stop a previous operation that may be interrupt driven
	 */
    irqstate_t state = irqsave();

	while (write_len < buflen) 
	{
		
        if(xRingBufferPut(&private->tx_ringbuf, &buffer[write_len], sizeof(uint8_t)) == 0)
            write_len++;
        else
            break;
	}


	/*
	 * Enable THRE interrupt
	 */
	ier = XUartNs550_ReadReg(inst_ptr->BaseAddress, XUN_IER_OFFSET);
	XUartNs550_WriteReg(inst_ptr->BaseAddress, XUN_IER_OFFSET, ier | XUN_IER_TX_EMPTY);
    irqrestore(state);

    return write_len;
}

static void structure_trans_to_drv(UartDataFormat_t *usr_format, XUartNs550Format *drv_format)
{
   drv_format->BaudRate = usr_format->iBaudRate;

    switch(usr_format->iDataBits)
    {
        case UART_DATA_5_BIT:
            drv_format->DataBits = XUN_FORMAT_5_BITS;
            break;
        case UART_DATA_6_BIT:
            drv_format->DataBits = XUN_FORMAT_6_BITS;
            break;
        case UART_DATA_7_BIT:
            drv_format->DataBits = XUN_FORMAT_7_BITS;
            break;
        case UART_DATA_8_BIT:
            drv_format->DataBits = XUN_FORMAT_8_BITS;
            break;
        default:
            drv_format->DataBits = XUN_FORMAT_8_BITS;
            break;
    }

    switch(usr_format->iParity)
    {
        case UART_EVEN_PARITY:
            drv_format->Parity = XUN_FORMAT_EVEN_PARITY;
            break;
        case UART_ODD_PARITY:
            drv_format->Parity = XUN_FORMAT_ODD_PARITY;
            break;
        case UART_NO_PARITY:
            drv_format->Parity = XUN_FORMAT_NO_PARITY;
            break;
        default:
            drv_format->Parity = XUN_FORMAT_NO_PARITY;
            break;
    }


    switch(usr_format->iStopBits)
    {
        case UART_1_STOP_BIT:
            drv_format->StopBits = XUN_FORMAT_1_STOP_BIT;
            break;
        case UART_2_STOP_BIT:
            drv_format->StopBits = XUN_FORMAT_1_STOP_BIT;
            break;
        default:
            drv_format->StopBits = XUN_FORMAT_1_STOP_BIT;
            break;
    }

}

static void structure_trans_to_user(UartDataFormat_t *usr_format, XUartNs550Format *drv_format)
{
    usr_format->iBaudRate = drv_format->BaudRate;

    switch(drv_format->DataBits)
    {
        case XUN_FORMAT_5_BITS:
            usr_format->iDataBits = UART_DATA_5_BIT;
            break;
        case XUN_FORMAT_6_BITS:
            usr_format->iDataBits = UART_DATA_6_BIT;
            break;
        case XUN_FORMAT_7_BITS:
            usr_format->iDataBits = UART_DATA_7_BIT;
            break;
        case XUN_FORMAT_8_BITS:
            usr_format->iDataBits = UART_DATA_8_BIT;
            break;
        default:
            usr_format->iDataBits = UART_DATA_8_BIT;
            break;
    }

    switch(drv_format->Parity)
    {
        case XUN_FORMAT_EVEN_PARITY:
            usr_format->iParity = UART_EVEN_PARITY;
            break;
        case XUN_FORMAT_ODD_PARITY:
            usr_format->iParity = UART_ODD_PARITY;
            break;
        case XUN_FORMAT_NO_PARITY:
            usr_format->iParity = UART_NO_PARITY;
            break;
        default:
            usr_format->iParity = UART_NO_PARITY;
            break;
    }


    switch(drv_format->StopBits)
    {
        case XUN_FORMAT_1_STOP_BIT:
            usr_format->iStopBits = UART_1_STOP_BIT;
            break;
        case XUN_FORMAT_2_STOP_BIT:
            usr_format->iStopBits = UART_2_STOP_BIT;
            break;
        default:
            usr_format->iStopBits = UART_1_STOP_BIT;
            break;
    }

}

int uartns_ioctl(file_t *filp, int cmd, unsigned long arg)
{
    uint32_t ier = 0;
	uartns_private_t *private = (uartns_private_t *)filp->f_inode->i_private;
	XUartNs550 *inst_ptr = private->uartns_inst_ptr;

	irqstate_t state = irqsave();

	switch(cmd)
    {
        case FIONREAD:
		{
			int available = 0;

			available = iRingBufferGetAvailable(&private->rx_ringbuf);
			*(int *)arg = available;
			break;
		}   
        case FIONWRITE:
		{
			int space = 0;

			space = iRingBufferGetSpace(&private->tx_ringbuf);
			*(int *)arg = space;
			break;
		}
		case UART_IOC_SET_MODE:
		{
            uint16_t curr_mode = XUartNs550_GetOptions(inst_ptr);

            if((uint32_t *)arg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }
		
            uint32_t mode = *(uint32_t *)(arg);

            if(mode == UART_MODE_LOOP)
            {
                mode = curr_mode | XUN_OPTION_LOOPBACK; 
            }
            else
            {
                mode = curr_mode & (~XUN_OPTION_LOOPBACK); 
            }

			private->mode = mode;
			XUartNs550_SetOptions(inst_ptr, mode);
			break;
		}
        case UART_IOC_GET_DATA_FORMAT:
        {
            if((UartDataFormat_t *)arg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }

            UartDataFormat_t *data_format = (UartDataFormat_t *)arg;
            XUartNs550Format xil_format = {0};

            XUartNs550_GetDataFormat(inst_ptr, &xil_format);

            structure_trans_to_user(data_format, &xil_format);

            break;
        }
        case UART_IOC_SET_DATA_FORMAT:
        {
            if((UartDataFormat_t *)arg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }

            UartDataFormat_t *data_format = (UartDataFormat_t *)arg;
            XUartNs550Format xil_format = {0};

            structure_trans_to_drv(data_format, &xil_format);
            Print_Info("baud=%d databits=%d parity=%d stop=%d\n", xil_format.BaudRate, xil_format.DataBits, xil_format.Parity, xil_format.StopBits);

            XUartNs550_SetDataFormat(inst_ptr, &xil_format);
            break;
        }
    }

    irqrestore(state);

}

