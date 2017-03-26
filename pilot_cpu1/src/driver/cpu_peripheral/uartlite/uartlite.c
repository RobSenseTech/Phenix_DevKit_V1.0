#include "FreeRTOS_Print.h"
#include "xuartlite.h"
#include "xuartlite_i.h"
#include "xuartlite_l.h"
#include "xparameters.h"
#include "driver.h"
#include "ringbuffer.h"

typedef struct{
	int32_t iUartID;	//uart编号
	XUartLite *pxUartInstPtr;
	int32_t iUartOperMode;
	RingBuffer_t xUartRxRingBuf;
	RingBuffer_t xUartTxRingBuf;
    SemaphoreHandle_t UartLiteRxMutex;
    SemaphoreHandle_t UartLiteTxMutex;
}UartLitePrivate_t;


XUartLite UartLite[XPAR_XUARTLITE_NUM_INSTANCES];
int32_t UartLiteHwIntID[XPAR_XUARTLITE_NUM_INSTANCES]={
	XPAR_FABRIC_AXI_UARTLITE_0_INTERRUPT_INTR, 
	XPAR_FABRIC_AXI_UARTLITE_1_INTERRUPT_INTR, 
	XPAR_FABRIC_AXI_UARTLITE_2_INTERRUPT_INTR, 
	XPAR_FABRIC_AXI_UARTLITE_3_INTERRUPT_INTR, 
};

void vUartLite_InterruptHandler(void *pvArg);

size_t UartLite_Read(Handle_t *pHandle, char *pcBuf, size_t xReadLen);

size_t UartLite_Write(Handle_t *pHandle, const char *pcBuf, size_t xWriteLen);

int UartLite_Ioctl(Handle_t *pHandle, int iCmd, void *pvArg);

DriverOps_t xUartLite_ops = {
	.read = UartLite_Read,
	.write = UartLite_Write,
	.ioctl = UartLite_Ioctl,
};

uint32_t UartLiteInit(int32_t iUartLiteId)
{
	uint32_t Status;
	XUartLite *UartLiteInstPtr = &UartLite[iUartLiteId];
	UartLitePrivate_t *pxUartLitePrivate = NULL;
	int UartLiteIntrId = UartLiteHwIntID[iUartLiteId];
    char DevName[MAX_DRIVER_NAME_LEN] = {0};

	if(iUartLiteId >= XPAR_XUARTLITE_NUM_INSTANCES)
	{
		Print_Err("No such uart lite:%d, check vivado config\n", iUartLiteId);
		return XST_FAILURE;
	}

	/*
	 * step1: 初始化私有数据，其中包括收发数据缓冲区
	 */
	pxUartLitePrivate = (UartLitePrivate_t *)pvPortMalloc(sizeof(UartLitePrivate_t));
	if(pxUartLitePrivate == NULL)
	{
		Print_Err("malloc failed!!\n");
		return XST_FAILURE;
	}
	else
	{
		memset(pxUartLitePrivate, 0, sizeof(UartLitePrivate_t));
	}
	pxUartLitePrivate->iUartID = iUartLiteId;
	pxUartLitePrivate->pxUartInstPtr = UartLiteInstPtr;
	iRingBufferInit(&pxUartLitePrivate->xUartRxRingBuf, 512, sizeof(char));
	iRingBufferInit(&pxUartLitePrivate->xUartTxRingBuf, 512, sizeof(char));
    pxUartLitePrivate->UartLiteRxMutex = xSemaphoreCreateMutex();
    pxUartLitePrivate->UartLiteTxMutex = xSemaphoreCreateMutex();

	/*
	 * setp2: 初始化SDK中的串口配置
	 */
	Status = XUartLite_Initialize(UartLiteInstPtr, iUartLiteId);
	if(Status != XST_SUCCESS)
	{
		Print_Err("UartLite%d initialize failed\n", iUartLiteId);
		return XST_FAILURE;
	}

	Status = XUartLite_SelfTest(UartLiteInstPtr);
	if(Status != XST_SUCCESS)
	{
		Print_Err("UartLite%d self test failed\n", iUartLiteId);
		return XST_FAILURE;
	}

	/*
	 * step3: 指定uart中断指向哪个CPU
	 */
	GicBindInterruptToCpu(UartLiteIntrId, CPU_1_TARGETED);
	
	/*
	 * step4: 设置中断触发方式为上升沿触发
	 */
	GicSetTriggerType(UartLiteIntrId, INTR_TRIGGER_RAISING_EDGE);
	/*
	 * step5: 注册串口中断处理函数，SDK的代码不通用，这里自己实现
	 */
	GicIsrHandlerRegister(UartLiteIntrId,(Xil_ExceptionHandler) vUartLite_InterruptHandler,(void *) pxUartLitePrivate);
	if(Status != XST_SUCCESS)
	{
		Print_Err("Set Uart%d interrupt handler failed:%d\n", iUartLiteId, Status);
		return XST_FAILURE;
	}

	/*
	 * step6: 注册串口驱动
	 */
    snprintf(DevName, sizeof(DevName), "%s%d", "/dev/uartlite", iUartLiteId);
	DriverRegister(DevName, &xUartLite_ops, pxUartLitePrivate);

	/*
	 * step7:
	 */
	XUartLite_EnableInterrupt(UartLiteInstPtr);

	/*
	 * step8: 使能串口中断
	 */
	GicInterruptEnable(UartLiteIntrId);

	return XST_SUCCESS;
}

void vUartLite_InterruptHandler(void *pvArg)
{
	int32_t iRet = 0;
	UartLitePrivate_t *pxUartLitePrivate = (UartLitePrivate_t *)pvArg;
	XUartLite *InstancePtr = pxUartLitePrivate->pxUartInstPtr;
	uint32_t IsrStatus;
	uint8_t  ucVal = 0;

	Xil_AssertVoid(InstancePtr != NULL);

    irqstate_t state = irqsave();
	IsrStatus = XUartLite_GetStatusReg(InstancePtr->RegBaseAddress); 

     //   Print_Info("uartlite%d BaseAddr=%x IsrStatus=%x\n", pxUartLitePrivate->iUartID, InstancePtr->RegBaseAddress, IsrStatus);

	if ((IsrStatus & (XUL_SR_RX_FIFO_FULL |	XUL_SR_RX_FIFO_VALID_DATA)) != 0) 
	{
		while(XUartLite_GetStatusReg(InstancePtr->RegBaseAddress) & XUL_SR_RX_FIFO_VALID_DATA)
		{
            if(iRingBufferGetEmptyCount(&pxUartLitePrivate->xUartRxRingBuf) > 0)
            {
			    ucVal = XUartLite_ReadReg(InstancePtr->RegBaseAddress, XUL_RX_FIFO_OFFSET);
			    iRet = xRingBufferPut(&pxUartLitePrivate->xUartRxRingBuf, &ucVal, sizeof(uint8_t));
            }
            else
            {
                /*
                 * 如果rx fifo和ringbuffer都满了，则强行写一个数据到ringbuffer，防止rx中断再也进不来
                 */
                if(IsrStatus & XUL_SR_RX_FIFO_FULL != 0)
                {
			        ucVal = XUartLite_ReadReg(InstancePtr->RegBaseAddress, XUL_RX_FIFO_OFFSET);
			        iRet = xRingBufferForce(&pxUartLitePrivate->xUartRxRingBuf, &ucVal, sizeof(uint8_t));
                }
                break;
            }
		}

	}

    if((IsrStatus & XUL_SR_TX_FIFO_EMPTY) != 0)
    {
        portBASE_TYPE xHigherPriorityTaskWoken;
        xHigherPriorityTaskWoken = pdFALSE;
        
//        xSemaphoreTakeFromISR(pxUartLitePrivate->UartLiteTxMutex, &xHigherPriorityTaskWoken);
        while(!(XUartLite_GetStatusReg(InstancePtr->RegBaseAddress) & XUL_SR_TX_FIFO_FULL))
        {
            if(xRingBufferGet(&pxUartLitePrivate->xUartTxRingBuf, &ucVal, sizeof(uint8_t)) == 0)
            {
                /*
                 * Fill the FIFO from the buffer
                 */
                XUartLite_WriteReg(InstancePtr->RegBaseAddress,
                           XUL_TX_FIFO_OFFSET,
                           ucVal);
            }
            else
            {
                break;
            }
        }

  //      xSemaphoreGiveFromISR(pxUartLitePrivate->UartLiteTxMutex, &xHigherPriorityTaskWoken);
    }
    irqrestore(state);

}

size_t UartLite_Read(Handle_t *pHandle, char *pcBuf, size_t xReadLen)
{
	size_t xLen = 0;
	UartLitePrivate_t *pxUartLitePrivate = (UartLitePrivate_t *)pHandle->xInode.pvPrivate;

    irqstate_t state = irqsave();
	while(xLen < xReadLen)
	{
		if(xRingBufferGet(&pxUartLitePrivate->xUartRxRingBuf, &pcBuf[xLen], pxUartLitePrivate->xUartRxRingBuf._ItemSize) == 0)
			xLen++;
        else
            break;
	}
    irqrestore(state);
	
	return xLen;
}

size_t UartLite_Write(Handle_t *pHandle, const char *pcBuf, size_t xWriteLen)
{
	size_t xLen = 0;
	UartLitePrivate_t *pxUartLitePrivate = (UartLitePrivate_t *)pHandle->xInode.pvPrivate;
	XUartLite *InstancePtr = pxUartLitePrivate->pxUartInstPtr;
    uint8_t ucVal;

    irqstate_t state = irqsave();
    //xSemaphoreTake(pxUartLitePrivate->UartLiteTxMutex, portMAX_DELAY);
	while (xLen < xWriteLen) 
	{
        if(xRingBufferPut(&pxUartLitePrivate->xUartTxRingBuf, &pcBuf[xLen], sizeof(uint8_t)) == 0)
            xLen++;
        else
            break;
#if 0
		/* FIFO Full? */
		if(!(XUartLite_GetStatusReg(InstancePtr->RegBaseAddress) & XUL_SR_TX_FIFO_FULL))
		{
			/*
			 * Fill the FIFO from the buffer
			 */
			XUartLite_WriteReg(InstancePtr->RegBaseAddress,
					   XUL_TX_FIFO_OFFSET,
					   pcBuf[xLen]);

			xLen++;
		}
#endif
	}

    /*
     * If fifo is empty, we need to write one byte to tx fifo, to trigger tx fifo empty interrupt
     * so that interrupt can send the rest data to fifo
     */

    if(XUartLite_GetStatusReg(InstancePtr->RegBaseAddress) & (XUL_SR_TX_FIFO_EMPTY))
    {
        if(xRingBufferGet(&pxUartLitePrivate->xUartTxRingBuf, &ucVal, sizeof(uint8_t)) == 0)
        {
            /*
             * Fill the FIFO from the buffer
             */
            XUartLite_WriteReg(InstancePtr->RegBaseAddress,
                       XUL_TX_FIFO_OFFSET,
                       ucVal);
        }
        
    }
    
  //  xSemaphoreGive(pxUartLitePrivate->UartLiteTxMutex);
    irqrestore(state);
	return xLen;
}

int UartLite_Ioctl(Handle_t *pHandle, int iCmd, void *pvArg)
{
	UartLitePrivate_t *pxUartLitePrivate = (UartLitePrivate_t *)pHandle->xInode.pvPrivate;
	switch(iCmd)
	{
		case UART_IOC_NREAD:
		{
			int iFilledCount = 0;

            if(pvArg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }
			
            irqstate_t state = irqsave();
			iFilledCount = iRingBufferGetFilledCount(&pxUartLitePrivate->xUartRxRingBuf);
			*(int *)pvArg = iFilledCount;
            irqrestore(state);
			break;
		}
		case UART_IOC_NWRITE:
		{
			int iEmptyCount = 0;

            if(pvArg == NULL)
            {
                Print_Err("Invald Param!!\n");
                return -1;
            }
	
            irqstate_t state = irqsave();
			iEmptyCount = iRingBufferGetEmptyCount(&pxUartLitePrivate->xUartTxRingBuf);
			*(int *)pvArg = iEmptyCount;
            irqrestore(state);
			break;
		}
        case UART_IOC_SET_BAUDRATE:
            break;
		default:
			Print_Err("Unknow Command\n");
	}
	return 0;
}



