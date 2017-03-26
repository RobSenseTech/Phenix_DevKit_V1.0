#include "xparameters.h"
#include "xstatus.h"
#include "xttcps.h"
#include "FreeRTOS_Print.h"
#include "driver.h"

//根据中断号算出中断在GIC_SPI_TARGET寄存器中的位偏移
#define SPI_TARGET_BIT_CALC(InterruptId) ((InterruptId%4)*8)

/* The interrupt controller is initialised in this file, and made available to
other modules. */
XScuGic xInterruptController;
XScuGic *xIntcInstPtr = &xInterruptController;

void GicBindInterruptToCpu(int32_t iIntrId, int32_t iCpuId)
{
	uint32_t RegVal = 0;

	RegVal = XScuGic_DistReadReg(xIntcInstPtr, XSCUGIC_SPI_TARGET_OFFSET_CALC(iIntrId));
	RegVal &= ~(3 << SPI_TARGET_BIT_CALC(iIntrId));
	RegVal |= (iCpuId << SPI_TARGET_BIT_CALC(iIntrId));
	XScuGic_DistWriteReg(xIntcInstPtr, XSCUGIC_SPI_TARGET_OFFSET_CALC(iIntrId), RegVal);
}

int32_t GicIsrHandlerRegister(int32_t iIntrId, Xil_InterruptHandler Handler, void *CallBackRef)
{
	int iStatus = 0;

	iStatus = XScuGic_Connect(xIntcInstPtr, iIntrId,(Xil_ExceptionHandler) Handler,CallBackRef);
	if (iStatus != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

void GicInterruptEnable(int32_t iIntrId)
{
	 XScuGic_Enable(xIntcInstPtr, iIntrId);
}

void GicSetTriggerType(uint32_t iIntrId, uint8_t Trigger)
{
	uint32_t RegValue;
	/*
	 * Determine the register to write to using the iIntrId.
	 */
	RegValue = XScuGic_DistReadReg(xIntcInstPtr, XSCUGIC_INT_CFG_OFFSET_CALC (iIntrId));

	/*
	 * Shift and Mask the correct bits for the priority and trigger in the
	 * register
	 */
	RegValue &= ~(XSCUGIC_INT_CFG_MASK << ((iIntrId%16)*2));
	RegValue |= Trigger << ((iIntrId%16)*2);

	/*
	 * Write the value back to the register.
	 */
	XScuGic_DistWriteReg(xIntcInstPtr, XSCUGIC_INT_CFG_OFFSET_CALC(iIntrId),
				RegValue);


}

void GicInit()
{
	BaseType_t xStatus;
	XScuGic_Config *pxGICConfig;

	/* Ensure no interrupts execute while the scheduler is in an inconsistent
	state.  Interrupts are automatically enabled when the scheduler is
	started. */
	portDISABLE_INTERRUPTS();

	/* Obtain the configuration of the GIC. */
	pxGICConfig = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );

	/* Sanity check the FreeRTOSConfig.h settings are correct for the
	hardware. */
	configASSERT( pxGICConfig );
	configASSERT( pxGICConfig->CpuBaseAddress == ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET ) );
	configASSERT( pxGICConfig->DistBaseAddress == configINTERRUPT_CONTROLLER_BASE_ADDRESS );

	/* Install a default handler for each GIC interrupt. */
	//注意，整个系统必须用同一个xInterruptController！！！！！
	xStatus = XScuGic_CfgInitialize(xIntcInstPtr, pxGICConfig, pxGICConfig->CpuBaseAddress );
	if(xStatus != XST_SUCCESS)
	{
		Print_Err("Gic cfg init failed:%d! Assert now\n", xStatus);
	}
	configASSERT( xStatus == XST_SUCCESS );
	( void ) xStatus; /* Remove compiler warning if configASSERT() is not defined. */

	/* The Xilinx projects use a BSP that do not allow the start up code to be
	altered easily.  Therefore the vector table used by FreeRTOS is defined in
	FreeRTOS_asm_vectors.S, which is part of this project.  Switch to use the
	FreeRTOS vector table. */
	vPortInstallFreeRTOSVectorTable();
}
