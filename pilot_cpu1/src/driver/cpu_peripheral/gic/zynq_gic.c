#include "xparameters.h"
#include "xstatus.h"
#include "xttcps.h"
#include "pilot_print.h"
#include "driver.h"

//根据中断号算出中断在GIC_SPI_TARGET寄存器中的位偏移
#define SPI_TARGET_BIT_CALC(InterruptId) ((InterruptId%4)*8)

/* The interrupt controller is initialised in this file, and made available to
other modules. */
XScuGic xInterruptController;
XScuGic *gic_inst_ptr = &xInterruptController;

void gic_bind_interrupt_to_cpu(int32_t intr_id, int32_t cpu_id)
{
	uint32_t reg_val = 0;

	reg_val = XScuGic_DistReadReg(gic_inst_ptr, XSCUGIC_SPI_TARGET_OFFSET_CALC(intr_id));
	reg_val &= ~(3 << SPI_TARGET_BIT_CALC(intr_id));
	reg_val |= (cpu_id << SPI_TARGET_BIT_CALC(intr_id));
	XScuGic_DistWriteReg(gic_inst_ptr, XSCUGIC_SPI_TARGET_OFFSET_CALC(intr_id), reg_val);
}

int32_t gic_isr_register(int32_t intr_id, Xil_InterruptHandler Handler, void *CallBackRef)
{
	int status = 0;

	status = XScuGic_Connect(gic_inst_ptr, intr_id,(Xil_ExceptionHandler) Handler,CallBackRef);
	if (status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

void gic_interrupt_enable(int32_t intr_id)
{
	 XScuGic_Enable(gic_inst_ptr, intr_id);
}

void gic_set_trigger_type(uint32_t intr_id, uint8_t Trigger)
{
	uint32_t reg_val;
	/*
	 * Determine the register to write to using the intr_id.
	 */
	reg_val = XScuGic_DistReadReg(gic_inst_ptr, XSCUGIC_INT_CFG_OFFSET_CALC (intr_id));

	/*
	 * Shift and Mask the correct bits for the priority and trigger in the
	 * register
	 */
	reg_val &= ~(XSCUGIC_INT_CFG_MASK << ((intr_id%16)*2));
	reg_val |= Trigger << ((intr_id%16)*2);

	/*
	 * Write the value back to the register.
	 */
	XScuGic_DistWriteReg(gic_inst_ptr, XSCUGIC_INT_CFG_OFFSET_CALC(intr_id),
				reg_val);


}

void gic_init()
{
	BaseType_t status;
	XScuGic_Config *config;

	/* Ensure no interrupts execute while the scheduler is in an inconsistent
	state.  Interrupts are automatically enabled when the scheduler is
	started. */
	portDISABLE_INTERRUPTS();

	/* Obtain the configuration of the GIC. */
	config = XScuGic_LookupConfig( XPAR_SCUGIC_SINGLE_DEVICE_ID );

	/* Sanity check the FreeRTOSConfig.h settings are correct for the
	hardware. */
	configASSERT( config );
	configASSERT( config->CpuBaseAddress == ( configINTERRUPT_CONTROLLER_BASE_ADDRESS + configINTERRUPT_CONTROLLER_CPU_INTERFACE_OFFSET ) );
	configASSERT( config->DistBaseAddress == configINTERRUPT_CONTROLLER_BASE_ADDRESS );

	/* Install a default handler for each GIC interrupt. */
	//注意，整个系统必须用同一个xInterruptController！！！！！
	status = XScuGic_CfgInitialize(gic_inst_ptr, config, config->CpuBaseAddress );
	if(status != XST_SUCCESS)
	{
		pilot_err("Gic cfg init failed:%d! Assert now\n", status);
	}
	configASSERT( status == XST_SUCCESS );
	( void ) status; /* Remove compiler warning if configASSERT() is not defined. */

	/* The Xilinx projects use a BSP that do not allow the start up code to be
	altered easily.  Therefore the vector table used by FreeRTOS is defined in
	FreeRTOS_asm_vectors.S, which is part of this project.  Switch to use the
	FreeRTOS vector table. */
	vPortInstallFreeRTOSVectorTable();
}
