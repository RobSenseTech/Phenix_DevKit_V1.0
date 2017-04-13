/**************************** Include Files *******************************/

#include <stdlib.h>
#include <string.h>
#include "xparameters.h"
#include "xuartps.h"
#include "xil_exception.h"
#include "FreeRTOS_Print.h"
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
static uint32_t UartHwIntID[XPAR_XUARTPS_NUM_INSTANCES] = {
	XPAR_XUARTPS_0_INTR,//59
	XPAR_XUARTPS_1_INTR,//82
};

typedef struct{
	int32_t iUartID;	//uart编号
	XUartPs *pxUartInstPtr;
	int32_t iUartOperMode;
	RingBuffer_t xUartRxRingBuf;
	RingBuffer_t xUartTxRingBuf;
    SemaphoreHandle_t UartRxMutex;
    SemaphoreHandle_t UartTxMutex;
}UartDrvPrivate_t;

/************************** Function Prototypes *****************************/
void vUartPs_InterruptHandler(void *pvArg);

int Uart_Open(struct file *filp);
	
ssize_t Uart_Read(struct file *filp, char *buffer, size_t buflen);

ssize_t Uart_Write(struct file *filp, const char *buffer, size_t buflen);

int Uart_Ioctl(file_t *filp, int iCmd, unsigned long ulArg);

/************************** Variable Definitions ***************************/

static XUartPs UartPs[XPAR_XUARTPS_NUM_INSTANCES];		/* Instance of the UART Device */

const struct file_operations xUart_ops = {
	.read = Uart_Read,
	.write = Uart_Write,
	.ioctl = Uart_Ioctl,
};

//串口时钟初始化，虽然fsbl阶段会配置，但是liunx内核启动后会还原这些寄存器，所以在cpu1再配一次
uint32_t UartPsClkInit(int32_t iUartId)
{
	u32 uiVal;

	if(iUartId == 0)
	{
		//串口时钟
		uiVal = UART_CLK_CTRL;
		uiVal |= UART0_CLK_ENABLE;
		UART_CLK_CTRL = uiVal;

		//外设时钟
		uiVal = APER_CLK_CTRL;
		uiVal |= UART0_CPU_1XCLKACT;
		APER_CLK_CTRL = uiVal;
	}
	else if(iUartId == 1)
	{
		//串口时钟
		uiVal = UART_CLK_CTRL;
		uiVal |= UART1_CLK_ENABLE;
		UART_CLK_CTRL = uiVal;

		//外设时钟
		uiVal = APER_CLK_CTRL;
		uiVal |= UART1_CPU_1XCLKACT;
		APER_CLK_CTRL = uiVal;
	}

//	Print_Info("Uart%d UART_CLK_CTRL=%x APER_CLK_CTRL=%x\n", iUartId, UART_CLK_CTRL, APER_CLK_CTRL);

	return 0;
}

//arm核有两个uart，uart1用于console，uart0用于gps，这里只初始化usrt0，uart1由linux初始化
uint32_t UartPsInit(int32_t iUartId)
{
	uint32_t Status;
	XUartPs_Config *Config;
	u32 IntrMask;
	XUartPs *UartInstPtr = &UartPs[iUartId];
	UartDrvPrivate_t *pxUartDrvPrivate = NULL;
	int UartIntrId = UartHwIntID[iUartId];
    char DevName[64] = {0};

	if(iUartId >= XPAR_XUARTPS_NUM_INSTANCES)
	{
		Print_Err("No such uart:%d, check vivado config\n", iUartId);
		return XST_FAILURE;
	}

	/*
	 * step1: 初始化私有数据，其中包括收发数据缓冲区
	 */
	pxUartDrvPrivate = (UartDrvPrivate_t *)pvPortMalloc(sizeof(UartDrvPrivate_t));
	if(pxUartDrvPrivate == NULL)
	{
		Print_Err("malloc failed!!\n");
		return XST_FAILURE;
	}
	else
	{
		memset(pxUartDrvPrivate, 0, sizeof(UartDrvPrivate_t));
	}
	pxUartDrvPrivate->iUartID = iUartId;
	pxUartDrvPrivate->pxUartInstPtr = UartInstPtr;
	iRingBufferInit(&pxUartDrvPrivate->xUartRxRingBuf, 512, sizeof(char));
	iRingBufferInit(&pxUartDrvPrivate->xUartTxRingBuf, 512, sizeof(char));
    pxUartDrvPrivate->UartRxMutex = xSemaphoreCreateMutex();
    pxUartDrvPrivate->UartTxMutex = xSemaphoreCreateMutex();

	/*
	 * step2: 使能串口时钟
	 */
	UartPsClkInit(iUartId);

	/*
	 * step3: 初始化SDK中的串口配置
	 */
	Config = XUartPs_LookupConfig(iUartId);
	if (NULL == Config) {
		Print_Err("There is no config\n");
		return XST_FAILURE;
	}

	Status = XUartPs_CfgInitialize(UartInstPtr, Config, Config->BaseAddress);
	if (Status != XST_SUCCESS) {
		Print_Err("Uart cfg init failed:%d\n", Status);
		return XST_FAILURE;
	}
#if 0
	Status = XUartPs_SelfTest(UartInstPtr);
	if (Status != XST_SUCCESS) {
		Print_Err("Uart self test failed:%d\n", Status);
		return XST_FAILURE;
	} 
	else
		Print_Info("self test success\n");
#endif

	/*
	 * step4: 指定uart中断指向哪个CPU
	 */
	GicBindInterruptToCpu(UartIntrId, CPU_1_TARGETED);
	
	/*
	 * step5: 注册串口中断处理函数，SDK的代码不通用，这里自己实现
	 */
	GicIsrHandlerRegister(UartIntrId,(Xil_ExceptionHandler) vUartPs_InterruptHandler,(void *) pxUartDrvPrivate);
	if(Status != XST_SUCCESS)
	{
		Print_Err("Set Uart%d interrupt handler failed:%d\n", iUartId, Status);
		return XST_FAILURE;
	}
	/*
	 * step6: 配置串口中断掩码
	 */
	IntrMask =
		XUARTPS_IXR_TOUT | XUARTPS_IXR_PARITY | XUARTPS_IXR_FRAMING |
		XUARTPS_IXR_OVER | XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_RXFULL |
		XUARTPS_IXR_RXOVR;
	XUartPs_SetInterruptMask(UartInstPtr, IntrMask);

	/*
	 * setp7: 配置串口模式为普通模式
	 */
	pxUartDrvPrivate->iUartOperMode = XUARTPS_OPER_MODE_NORMAL;
	XUartPs_SetOperMode(UartInstPtr, XUARTPS_OPER_MODE_NORMAL);

	/*
	 * step8: 注册串口驱动
	 */
    snprintf(DevName, sizeof(DevName), "%s%d", "/dev/uart", (int)iUartId);
//	DriverRegister(DevName, &xUart_ops, pxUartDrvPrivate);
    register_driver(DevName, &xUart_ops, 0666, pxUartDrvPrivate);

	/*
	 * step9: 使能串口中断
	 */
	GicInterruptEnable(UartIntrId);
	
	return XST_SUCCESS;
}

ssize_t Uart_Read(struct file *filp, char *buffer, size_t buflen)
{
	size_t xLen = 0;
	UartDrvPrivate_t *pxUartDrvPrivate = (UartDrvPrivate_t *)filp->f_inode->i_private;

    irqstate_t state = irqsave();
	while(xLen < buflen)
	{
//        Print_Info("head=%x tail=%x\n", pxUartDrvPrivate->xUartRxRingBuf._Head, pxUartDrvPrivate->xUartRxRingBuf._Tail); 
		if(xRingBufferGet(&pxUartDrvPrivate->xUartRxRingBuf, &buffer[xLen], pxUartDrvPrivate->xUartRxRingBuf._ItemSize) == 0)
        {
			xLen++;
        }
        else
            break;
	}
    irqrestore(state);
	
	return xLen;
}

ssize_t Uart_Write(struct file *filp, const char *buffer, size_t buflen)
{
	size_t xLen = 0;
	UartDrvPrivate_t *pxUartDrvPrivate = (UartDrvPrivate_t *)filp->f_inode->i_private;
	XUartPs *InstancePtr = pxUartDrvPrivate->pxUartInstPtr;
    uint8_t ucVal;

    /*
     * Mutex with interrupt
     */
    irqstate_t state = irqsave();
//    xSemaphoreTake(pxUartDrvPrivate->UartTxMutex, portMAX_DELAY);
	while (xLen < buflen) 
	{
		
        if(xRingBufferPut(&pxUartDrvPrivate->xUartTxRingBuf, &buffer[xLen], sizeof(uint8_t)) == 0)
            xLen++;
        else
            break;
#if 0
		if(!XUartPs_IsTransmitFull(InstancePtr->Config.BaseAddress))
		{
			/*
			 * Fill the FIFO from the buffer
			 */
			XUartPs_WriteReg(InstancePtr->Config.BaseAddress,
					   XUARTPS_FIFO_OFFSET,
					   pcBuf[xLen]);

			xLen++;
		}
#endif
	}

    /*
     * If fifo is empty, we need to write one byte to tx fifo, to trigger tx fifo empty interrupt
     * so that interrupt can send the rest data to fifo
     */
    if(XUartPs_IsTransmitEmpty(InstancePtr))
    {
        if(xRingBufferGet(&pxUartDrvPrivate->xUartTxRingBuf, &ucVal, sizeof(uint8_t)) == 0)
        {
            /*
             * Fill the FIFO from the buffer
             */
            XUartPs_WriteReg(InstancePtr->Config.BaseAddress,
                       XUARTPS_FIFO_OFFSET,
                       ucVal);
        }
        
    }
 //   xSemaphoreGive(pxUartDrvPrivate->UartTxMutex);
    irqrestore(state);

	return xLen;
}

int Uart_Ioctl(file_t *filp, int iCmd, unsigned long ulArg)
{
	int32_t iRet = 0;
	UartDrvPrivate_t *pxUartDrvPrivate = (UartDrvPrivate_t *)filp->f_inode->i_private;
	switch(iCmd)
	{
		case FIONREAD:
		{
			int iFilledCount = 0;

            irqstate_t state = irqsave();
			iFilledCount = iRingBufferGetAvailable(&pxUartDrvPrivate->xUartRxRingBuf);
			*(int *)ulArg = iFilledCount;
            irqrestore(state);
			break;
		}
		case FIONWRITE:
		{
			int iEmptyCount = 0;

            irqstate_t state = irqsave();
			iEmptyCount = iRingBufferGetSpace(&pxUartDrvPrivate->xUartTxRingBuf);
			*(int *)ulArg = iEmptyCount;
            irqrestore(state);
			break;
		}
		case UART_IOC_SET_MODE:
		{
            if((uint32_t *)ulArg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }
		
            uint32_t uiUartOperMode = *(uint32_t *)(ulArg);

            if(uiUartOperMode == UART_MODE_LOOP)
            {
                uiUartOperMode = XUARTPS_OPER_MODE_LOCAL_LOOP; 
            }
            else
            {
                uiUartOperMode = XUARTPS_OPER_MODE_NORMAL; 
            }

			pxUartDrvPrivate->iUartOperMode = uiUartOperMode;
			XUartPs_SetOperMode(pxUartDrvPrivate->pxUartInstPtr, uiUartOperMode);
			break;
		}	
        case UART_IOC_GET_DATA_FORMAT:
        {
            if((UartDataFormat_t *)ulArg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }
        
            XUartPsFormat *usr_format = (XUartPsFormat *)ulArg;

            //For ps uart, XUartPsFormat is equal to UartDataFormat_t
            XUartPs_SetDataFormat(pxUartDrvPrivate->pxUartInstPtr, usr_format);

            break;
        }
        case UART_IOC_SET_DATA_FORMAT:
        {
            if((XUartPsFormat *)ulArg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }

            XUartPsFormat *drv_format = (XUartPsFormat *)ulArg;

            //For ps uart, XUartPsFormat is equal to UartDataFormat_t
            XUartPs_SetDataFormat(pxUartDrvPrivate->pxUartInstPtr, drv_format);
            break;
        }
		default:
			Print_Err("Unknow Command\n");
            return -1;
	}
	return 0;
}


void vUartPs_InterruptHandler(void *pvArg)
{
	u32 IsrStatus;
	uint8_t  ucVal = 0;
	int32_t  iRet = 0;
	UartDrvPrivate_t *pxUartDrvPrivate = (UartDrvPrivate_t *)pvArg;
	XUartPs *InstancePtr = pxUartDrvPrivate->pxUartInstPtr;

	Xil_AssertVoid(InstancePtr != NULL);
	Xil_AssertVoid(InstancePtr->IsReady == XIL_COMPONENT_IS_READY);

    irqstate_t state = irqsave();
	/*
	 * Read the interrupt ID register to determine which
	 * interrupt is active
	 */
	IsrStatus = XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
				   XUARTPS_IMR_OFFSET);

	IsrStatus &= XUartPs_ReadReg(InstancePtr->Config.BaseAddress,
				   XUARTPS_ISR_OFFSET);

	/*
	 * Handle interrupt 
	 */
	if(0 != (IsrStatus & (XUARTPS_IXR_RXOVR | XUARTPS_IXR_RXEMPTY |
				 XUARTPS_IXR_RXFULL))) 
	{
		/* Recieved data interrupt */
		while(XUartPs_IsReceiveData(InstancePtr->Config.BaseAddress))
		{
            if(iRingBufferGetSpace(&pxUartDrvPrivate->xUartRxRingBuf) > 0) 
            {
            	ucVal = XUartPs_ReadReg(InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
//            Print_Warn("IsrStatus=%x val=%d\n", IsrStatus, ucVal);
			    iRet = xRingBufferPut(&pxUartDrvPrivate->xUartRxRingBuf, &ucVal, sizeof(uint8_t));

            }
            else
            {
                /*
                 * 如果rx fifo和ringbuffer都满了，则强行写一个数据到ringbuffer，防止rx中断再也进不来
                 */
                if(IsrStatus & (XUARTPS_IXR_RXFULL)) 
                {
                    ucVal = XUartPs_ReadReg(InstancePtr->Config.BaseAddress, XUARTPS_FIFO_OFFSET);
                    iRet = xRingBufferForce(&pxUartDrvPrivate->xUartRxRingBuf, &ucVal, sizeof(uint8_t));
                }
                break;
            }

		}
	}


	if(0 != (IsrStatus & (XUARTPS_IXR_TXEMPTY | XUARTPS_IXR_TXFULL))) 
	{
        portBASE_TYPE xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        
//        xSemaphoreTakeFromISR(pxUartDrvPrivate->UartTxMutex, &xHigherPriorityTaskWoken);
        /* Transmit data interrupt */
		while ((!XUartPs_IsTransmitFull(InstancePtr->Config.BaseAddress))) 
		{
			iRet = xRingBufferGet(&pxUartDrvPrivate->xUartTxRingBuf, &ucVal, sizeof(uint8_t));
			
			if(iRet == 0)
			{
				/*
				 * Fill the FIFO from the buffer
				 */
				XUartPs_WriteReg(InstancePtr->Config.BaseAddress,
						   XUARTPS_FIFO_OFFSET,
						   ucVal);
			}
			else
				break;
			
		}
 //       xSemaphoreGiveFromISR(pxUartDrvPrivate->UartTxMutex, &xHigherPriorityTaskWoken);
	}

	if(0 != (IsrStatus & (XUARTPS_IXR_OVER | XUARTPS_IXR_FRAMING |
				XUARTPS_IXR_PARITY))) {
		/* Recieved Error Status interrupt */
	}

	if(0 != (IsrStatus & XUARTPS_IXR_TOUT )) {
		/* Recieved Timeout interrupt */
	}

	if(0 != (IsrStatus & XUARTPS_IXR_DMS)) {
		/* Modem status interrupt */
	}

	/*
	 * Clear the interrupt status.
	 */
	XUartPs_WriteReg(InstancePtr->Config.BaseAddress, XUARTPS_ISR_OFFSET,
		IsrStatus);

    irqrestore(state);
}

