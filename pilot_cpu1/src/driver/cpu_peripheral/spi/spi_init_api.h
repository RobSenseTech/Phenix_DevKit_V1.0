/**
*******************************************************************************
* @file                  spi_init_api.h
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/
#ifndef _SPI_INIT_API_H
#define _SPI_INIT_API_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "xil_types.h"
#include "xil_io.h"
#include "xparameters.h"

/************************** Constant Definitions *****************************/
/**< AMBA Peripheral Clock Control */
#define XSLCR_APER_CLK_CTRL_ADDR				(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x0000012CU)
/**< SPI0 clock enable shift */
#define XSLCR_SPI_0_CPU_1XCLKACT_SHIFT	14U
/**< SPI0 clock enable MASK */
#define XSLCR_SPI_0_CPU_1XCLKACT_MASK	0x00004000U
/**< SPI1 clock enable shift */
#define XSLCR_SPI_1_CPU_1XCLKACT_SHIFT	15U
/**< SPI1 clock enable MASK */
#define XSLCR_SPI_1_CPU_1XCLKACT_MASK	0x00008000U

/**< SPI Ref Clock Control */
#define XSLCR_SPI_CLK_CTRL_ADDR					(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x00000158U)
/**< SPI 0 reference clock source enable/disable shift */
#define XSLCR_SPI_0_CLKACT1_SHIFT	0U
/**< SPI 0 reference clock source enable/disable MASK */
#define XSLCR_SPI_0_CLKACT1_MASK	0x00000001U
/**< SPI 1 reference clock source enable/disable shift */
#define XSLCR_SPI_1_CLKACT1_SHIFT	1U
/**< SPI 0 reference clock source enable/disable MASK */
#define XSLCR_SPI_1_CLKACT1_MASK	0x00000002U
/**< SPI clock source select shift */
#define XSLCR_SPI_CLK_SOURCE_SELECT_SHIFT	4U
/**< SPI clock source select MASK */
#define XSLCR_SPI_CLK_SOURCE_SELECT_MASK	0x00000030U
/**< SPI clock divisor shift */
#define XSLCR_SPI_CLK_DIVISOR_SHIFT	8U
/**< SPI clock divisor MASK  with 6 bits*/
#define XSLCR_SPI_CLK_DIVISOR_MASK	0x00003F00U

/**< SPI Ref Clock Control */
//#define XSLCR_SPI_RST_CTRL_ADDR					(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x0000021CU)
/**< SPI 0 AMBA software reset shift*/
#define XSLCR_SPI_0_CPU1X_RST_SHIFT	0U
/**< SPI 0 AMBA software reset mask */
#define XSLCR_SPI_0_CPU1X_RST_MASK	0x00000001U
/**< SPI 1 AMBA software reset shift*/
#define XSLCR_SPI_1_CPU1X_RST_SHIFT	1U
/**< SPI 1 AMBA software reset mask */
#define XSLCR_SPI_1_CPU1X_RST_MASK	0x00000002U
/**< SPI 0 Reference software reset shift*/
#define XSLCR_SPI_0_REF_RST_SHIFT 2U
/**< SPI 0 Reference software reset mask */
#define XSLCR_SPI_0_REF_RST_MASK	0x00000003U
/**< SPI 1 Reference software reset */
#define XSLCR_SPI_1_REF_RST_SHIFT 3U
/**< SPI 0 Reference software reset mask */
#define XSLCR_SPI_1_REF_RST_MASK	0x00000004U

/**< SPI SCLK MOSI MISO MIO configure */
#define XSLCR_SCLK_SPI_MIO_16_ADDR			(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x00000740U)
#define XSLCR_MOSI_SPI_MIO_21_ADDR			(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x00000754U)
#define XSLCR_MISO_SPI_MIO_17_ADDR			(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x00000744U)

#define XSLCR_SCLK_SPI_MIO_48_ADDR			(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x000007C0U)
#define XSLCR_MOSI_SPI_MIO_46_ADDR			(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x000007B8U)
#define XSLCR_MISO_SPI_MIO_47_ADDR			(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x000007BCU)

/**< SPI 1 SCLK MOSI MISO MIO configure */
#define XSLCR_SCLK_SPI_0_ADDR			XSLCR_SCLK_SPI_MIO_16_ADDR
#define XSLCR_MOSI_SPI_0_ADDR			XSLCR_MOSI_SPI_MIO_21_ADDR
#define XSLCR_MISO_SPI_0_ADDR			XSLCR_MISO_SPI_MIO_17_ADDR

#define XSLCR_SCLK_SPI_1_ADDR			XSLCR_SCLK_SPI_MIO_48_ADDR
#define XSLCR_MOSI_SPI_1_ADDR			XSLCR_MOSI_SPI_MIO_46_ADDR
#define XSLCR_MISO_SPI_1_ADDR			XSLCR_MISO_SPI_MIO_47_ADDR

enum clock_source
{
	CLOCK_SOURCE_IO_PLL     = 0,
	CLOCK_SOURCE_ARM_PLL    = 2,
	CLOCK_SOURCE_DDR_PLL    = 3
};

/************************** Function Prototypes ******************************/
/*
 * spi clock active enable interface
 */
 s32 spi_clock_active_enable(u32 spi_id);
 /*
 * spi clock active disable interface
 */
 s32 spi_clock_active_disable(u32 spi_id);


 s32 spi_reference_clock_enable(u32 spi_id);

 s32 spi_clock_source_select(enum clock_source clock);

 s32 spi_clock_source_divisor(u32 divisor);

 s32 spi_clock_init(u32 spi_id, enum clock_source clock, u32 clock_divisor);

 s32 spi_SCLK_init(u32 spi_id);

 s32 spi_MOSI_init(u32 spi_id);

 s32 spi_MISO_init(u32 spi_id);

 s32 spi_SS_init(u8 slave_id);

#ifdef __cplusplus
}
#endif

#endif /* _SPI_INIT_API_H */
