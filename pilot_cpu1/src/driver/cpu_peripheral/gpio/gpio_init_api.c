/**
*******************************************************************************
* @file                  gpio_init_api.c
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/

/***************************** Include Files *********************************/
#include "gpio_init_api.h"
#include "xstatus.h"


/************************** Constant Definitions *****************************/

/************************** Variable Definitions *****************************/
XGpioPs GpioInstance;
XGpioPs *spi_gpio;

/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/


/*****************************************************************************/
/**
* This function contains what function.
*
* @param   N/A.
*
* @return   N/A.
*
* @note     None.
*
******************************************************************************/
s32 gpio_clock_active_enable() {
	u32 Register;
	s32 Status = XST_SUCCESS;
	u32 Return;

	Register = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);

	Register |= (u32) 1 << XSLCR_GPIO_CPU_1XCLKACT_SHIFT;

	Xil_Out32(XSLCR_APER_CLK_CTRL_ADDR, Register);

	Return = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);
	if (Register != Return) {
		xil_printf("gpio_clock_active_enable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
		Status = XST_FAILURE;
	}

	return Status;
}

void gpio_port_output_config(XGpioPs *InstancePtr, u32 port_id) {

	/*
	 * Set the direction for the pin to be output
	 */
	XGpioPs_SetDirectionPin(InstancePtr, port_id, 1);
	XGpioPs_SetOutputEnablePin(InstancePtr, port_id, 1);
}

void gpio_port_input_config(XGpioPs *InstancePtr, u32 port_id) {

	/*
	 * Set the direction for the pin to be input
	 */
	XGpioPs_SetDirectionPin(InstancePtr, port_id, 0x0);
}

int GpioPsInit() {
	int Status;
	XGpioPs_Config *ConfigPtr;

	Status = gpio_clock_active_enable();
	if (Status != XST_SUCCESS) {
		xil_printf("GPIO Interrupt clock enable failed \r\n");
		return XST_FAILURE;
	}

	/*
	 * Initialize the GPIO driver.
	 */
	ConfigPtr = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
	Status = XGpioPs_CfgInitialize(&GpioInstance, ConfigPtr,
					ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		xil_printf("XGpioPs_CfgInitialize failed %d\r\n", Status);
		return XST_FAILURE;
	}

	spi_gpio = &GpioInstance;

	return XST_SUCCESS;
}