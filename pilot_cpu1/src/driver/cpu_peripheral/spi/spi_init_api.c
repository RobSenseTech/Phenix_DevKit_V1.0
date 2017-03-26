/**
*******************************************************************************
* @file                  spi_init_api.c
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/

/***************************** Include Files *********************************/
#include "spi_init_api.h"
#include "xstatus.h"

/************************** Constant Definitions *****************************/

/************************** Variable Definitions *****************************/


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
s32 spi_clock_active_enable(u32 spi_id) {
	u32 Register;
	s32 Status = XST_SUCCESS;
	u32 Return;

	Register = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);
	if (spi_id == 0) {
		Register |= (u32) 1 << XSLCR_SPI_0_CPU_1XCLKACT_SHIFT;
	}
	else if (spi_id == 1) {
		Register |= (u32) 1 << XSLCR_SPI_1_CPU_1XCLKACT_SHIFT;
	}
	else {
		Status = XST_FAILURE;
	}

	if (Status == XST_SUCCESS) {
		Xil_Out32(XSLCR_APER_CLK_CTRL_ADDR, Register);

		Return = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);
		if (Register != Return) {
			xil_printf("spi_clock_active_enable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
			Status = XST_FAILURE;
		}
	}
	return Status;
}

s32 spi_clock_active_disable(u32 spi_id) {
	u32 Register;
	s32 Status = XST_SUCCESS;
	u32 Return;

	Register = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);
	if (spi_id == 0) {
		Register &= (u32)(~XSLCR_SPI_0_CPU_1XCLKACT_MASK);
	}
	else if (spi_id == 1) {
		Register &= (u32)(~XSLCR_SPI_1_CPU_1XCLKACT_MASK);
	}
	else {
		Status = XST_FAILURE;
	}

	if (Status == XST_SUCCESS) {
		Xil_Out32(XSLCR_APER_CLK_CTRL_ADDR, Register);

		Return = Xil_In32(XSLCR_APER_CLK_CTRL_ADDR);
		if (Register != Return) {
			xil_printf("spi_clock_active_disable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
			Status = XST_FAILURE;
		}
	}
	return Status;
}

s32 spi_reference_clock_enable(u32 spi_id) {
	u32 Register;
	s32 Status = XST_SUCCESS;
	u32 Return;

	Register = Xil_In32(XSLCR_SPI_CLK_CTRL_ADDR);
	if (spi_id == 0) {
		Register |= (u32) 1 << XSLCR_SPI_0_CLKACT1_SHIFT;
	}
	else if (spi_id == 1) {
		Register |= (u32) 1 << XSLCR_SPI_1_CLKACT1_SHIFT;
	}
	else {
		Status = XST_FAILURE;
	}

	if (Status == XST_SUCCESS) {
		Xil_Out32(XSLCR_SPI_CLK_CTRL_ADDR, Register);

		Return = Xil_In32(XSLCR_SPI_CLK_CTRL_ADDR);
		if (Register != Return) {
			xil_printf("spi_reference_clock_enable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
			Status = XST_FAILURE;
		}
	}
	return Status;
}

s32 spi_clock_source_select(enum clock_source clock) {
	u32 Register;
	s32 Status = XST_SUCCESS;
	u32 clockSource = (u32) clock;
	u32 Return;

	Register = Xil_In32(XSLCR_SPI_CLK_CTRL_ADDR);
	Register &= (u32)(~XSLCR_SPI_CLK_SOURCE_SELECT_MASK);
	Register |= (u32) clockSource << XSLCR_SPI_CLK_SOURCE_SELECT_SHIFT;
	Xil_Out32(XSLCR_SPI_CLK_CTRL_ADDR, Register);

	Return = Xil_In32(XSLCR_SPI_CLK_CTRL_ADDR);
	if (Register != Return) {
		xil_printf("spi_reference_clock_enable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
		Status = XST_FAILURE;
	}

	return Status;
}

s32 spi_clock_source_divisor(u32 divisor) {
	u32 Register;
	s32 Status = XST_SUCCESS;
	u32 Return;

	if (divisor < 64) {
		Register = Xil_In32(XSLCR_SPI_CLK_CTRL_ADDR);
		Register &= (u32)(~XSLCR_SPI_CLK_DIVISOR_MASK);
		Register |= (u32) divisor << XSLCR_SPI_CLK_DIVISOR_SHIFT;
		Xil_Out32(XSLCR_SPI_CLK_CTRL_ADDR, Register);

		Return = Xil_In32(XSLCR_SPI_CLK_CTRL_ADDR);
		if (Register != Return) {
			xil_printf("spi_reference_clock_enable failed, return value is 0x%08x  (wanted: 0x%08x)\r\n", Return, Register);
			Status = XST_FAILURE;
		}
	} else {
		xil_printf("spi_clock_source_divisor failed, illegal value %d  (wanted: 0 <= divisor <= 63)\r\n",divisor);
		Status = XST_FAILURE;
	}
	return Status;
}

 s32 spi_clock_init(u32 spi_id, enum clock_source clock, u32 clock_divisor) {

	s32 Status = XST_SUCCESS;

	if (spi_clock_active_enable(spi_id) != XST_SUCCESS) {
		xil_printf("spi_clock_active_enable failed\r\n");
		Status = XST_FAILURE;
	} else if (spi_clock_source_select(clock) != XST_SUCCESS) {
		xil_printf("spi_clock_source_select failed\r\n");
		Status = XST_FAILURE;
	} else if (spi_clock_source_divisor(clock_divisor) != XST_SUCCESS) {
		xil_printf("spi_clock_source_divisor failed\r\n");
		Status = XST_FAILURE;
	} else if (spi_reference_clock_enable(spi_id) != XST_SUCCESS) {
		xil_printf("spi_reference_clock_enable failed\r\n");
		Status = XST_FAILURE;
	} else {
		xil_printf("SPI clock enable successful\r\n");
	}
	spi_SCLK_init(spi_id);
	spi_MOSI_init(spi_id);
	spi_MISO_init(spi_id);

	return Status;
 }

 s32 spi_SCLK_init(u32 spi_id) {
	u32 Register = 0x22A0;
	if(spi_id == 1)
	{
		Xil_Out32(XSLCR_SCLK_SPI_1_ADDR, Register);
		Register = Xil_In32(XSLCR_SCLK_SPI_1_ADDR);
	 	xil_printf("address 0x%08x : 0x%08x\r\n", XSLCR_SCLK_SPI_1_ADDR, Register);
		return XST_SUCCESS;
	}
	else if (spi_id == 0)
	{
		Xil_Out32(XSLCR_SCLK_SPI_0_ADDR, Register);
		Register = Xil_In32(XSLCR_SCLK_SPI_0_ADDR);
	 	xil_printf("address 0x%08x : 0x%08x\r\n", XSLCR_SCLK_SPI_0_ADDR, Register);
		return XST_SUCCESS;
 	}
 else
 	{
	 xil_printf("spi_id error!\r\n");
	 return XST_FAILURE;
 	}
}

 s32 spi_MOSI_init(u32 spi_id) {
	u32 Register = 0x22A0;
	if(spi_id == 1)
		{
			Xil_Out32(XSLCR_MOSI_SPI_1_ADDR, Register);
			Register = Xil_In32(XSLCR_MOSI_SPI_1_ADDR);
		 	xil_printf("address 0x%08x : 0x%08x\r\n", XSLCR_MOSI_SPI_1_ADDR, Register);
			return XST_SUCCESS;
		}
		else if (spi_id == 0)
		{
			Xil_Out32(XSLCR_MOSI_SPI_0_ADDR, Register);
			Register = Xil_In32(XSLCR_MOSI_SPI_0_ADDR);
		 	xil_printf("address 0x%08x : 0x%08x\r\n", XSLCR_MOSI_SPI_0_ADDR, Register);
			return XST_SUCCESS;
		}
		else
	  {
	 	 xil_printf("spi_id error!\r\n");
	 	 return XST_FAILURE;
	  }
 }

 s32 spi_MISO_init(u32 spi_id) {
	u32 Register = 0x22A1;
	if(spi_id == 1)
		{
			Xil_Out32(XSLCR_MISO_SPI_1_ADDR, Register);
			Register = Xil_In32(XSLCR_MISO_SPI_1_ADDR);
		 	xil_printf("address 0x%08x : 0x%08x\r\n", XSLCR_MISO_SPI_1_ADDR, Register);
			return XST_SUCCESS;
		}
		else if (spi_id == 0)
		{
			Xil_Out32(XSLCR_MISO_SPI_0_ADDR, Register);
			Register = Xil_In32(XSLCR_MISO_SPI_0_ADDR);
		 	xil_printf("address 0x%08x : 0x%08x\r\n", XSLCR_MISO_SPI_0_ADDR, Register);
			return XST_SUCCESS;
		}
		else
		{
	 	 xil_printf("spi_id error!\r\n");
	 	 return XST_FAILURE;
	  }
	}

 s32 spi_SS_init(u8 slave_id) {
	return XST_SUCCESS;
 }
