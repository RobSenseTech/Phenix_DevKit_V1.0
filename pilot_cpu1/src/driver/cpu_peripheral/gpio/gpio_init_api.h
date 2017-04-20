/**
*******************************************************************************
* @file                  gpio_init_api.h
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/
#ifndef _GPIO_INIT_API_H
#define _GPIO_INIT_API_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "xil_types.h"
#include "xil_io.h"
#include "xparameters.h"
#include "xgpiops.h"

/************************** Constant Definitions *****************************/
/**< AMBA Peripheral Clock Control */
#define XSLCR_APER_CLK_CTRL_ADDR				(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x0000012CU)
/**< GPIO clock enable shift */
#define XSLCR_GPIO_CPU_1XCLKACT_SHIFT	22U
/**< GPIO clock enable MASK */
#define XSLCR_GPIO_CPU_1XCLKACT_MASK	0x00400000U

#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID
/************************** Variable Definitions ******************************/

/************************** Function Prototypes ******************************/
/*
 * spi clock active enable interface
 */
 s32 gpio_clock_active_enable();
 
 void gpio_port_output_config(XGpioPs *InstancePtr, u32 port_id);
 
 void gpio_port_input_config(XGpioPs *InstancePtr, u32 port_id);
 
 int gpiops_init();


#ifdef __cplusplus
}
#endif

#endif /* _GPIO_INIT_API_H */
