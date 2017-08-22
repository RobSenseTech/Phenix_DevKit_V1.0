/**
*******************************************************************************
* @file                  gpio_init_api.c
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/

/***************************** Include Files *********************************/
#include "gpio.h"
#include "xstatus.h"
#include "pilot_print.h"


/************************** Constant Definitions *****************************/

/************************** Variable Definitions *****************************/
XGpioPs gpio_ps;
xSemaphoreHandle gpio_mutex;

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
int32_t gpio_clock_active_enable() 
{
	uint32_t Register;
	int32_t Status = XST_SUCCESS;
	uint32_t Return;

	Register = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);

	Register |= (uint32_t) 1 << XSLCR_GPIO_CPU_1XCLKACT_SHIFT;

	Xil_Out32(XSLCR_APER_CLK_CTRL_ADDR, Register);

	Return = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);
	if (Register != Return) {
		pilot_err("gpio_clock_active_enable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
		Status = XST_FAILURE;
	}

	return Status;
}

void gpio_pin_config_output(uint32_t pin) 
{

    xSemaphoreTake(gpio_mutex, portMAX_DELAY);
	/*
	 * Set the direction for the pin to be output
	 */
	XGpioPs_SetDirectionPin(&gpio_ps, pin, 1);
	XGpioPs_SetOutputEnablePin(&gpio_ps, pin, 1);

    xSemaphoreGive(gpio_mutex);
}

void gpio_pin_config_input(uint32_t pin) 
{

    xSemaphoreTake(gpio_mutex, portMAX_DELAY);
	/*
	 * Set the direction for the pin to be input
	 */
	XGpioPs_SetDirectionPin(&gpio_ps, pin, 0x0);
    xSemaphoreGive(gpio_mutex);
}

void gpio_set(uint32_t pin)
{
    xSemaphoreTake(gpio_mutex, portMAX_DELAY);
    XGpioPs_WritePin(&gpio_ps, pin, 0x1);
    xSemaphoreGive(gpio_mutex);
}

void gpio_clear(uint32_t pin)
{
    xSemaphoreTake(gpio_mutex, portMAX_DELAY);
    XGpioPs_WritePin(&gpio_ps, pin, 0x0);
    xSemaphoreGive(gpio_mutex);
}

int32_t gpiops_init() 
{
	int status;
	XGpioPs_Config *config;

	status = gpio_clock_active_enable();
	if (status != XST_SUCCESS) {
		pilot_err("GPIO Interrupt clock enable failed \r\n");
		return XST_FAILURE;
	}

	/*
	 * Initialize the GPIO driver.
	 */
	config = XGpioPs_LookupConfig(GPIO_DEVICE_ID);
    if (NULL == config) {
		pilot_err(" gpio lookup configuration failed.\r\n");
		return XST_FAILURE;
	}

	status = XGpioPs_CfgInitialize(&gpio_ps, config,
					config->BaseAddr);
	if (status != XST_SUCCESS) {
		pilot_err("XGpioPs_CfgInitialize failed %d\r\n", status);
		return XST_FAILURE;
	}

    gpio_mutex = xSemaphoreCreateMutex();

	return XST_SUCCESS;
}
