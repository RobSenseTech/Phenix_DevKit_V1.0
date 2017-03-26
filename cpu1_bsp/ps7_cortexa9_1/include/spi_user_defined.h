/**
*******************************************************************************
* @file                  spi_user_defined.h
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/
#ifndef _SPI_USER_DEFINED_H
#define _SPI_USER_DEFINED_H

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C" {
#else
#define EXTERN extern
#endif
/***************************** Include Files *********************************/
#include "xspips.h"

/************************** Constant Definitions *****************************/

/************************** Variable Definitions *****************************/

/***************** Macros (Inline Functions) Definitions *********************/

/************************** Function Prototypes ******************************/
EXTERN u32 spi_disable_slave_via_gpio(XSpiPs *InstancePtr);

#undef EXTERN
#if defined(__cplusplus)
}
#endif
#endif /* _SPI_USER_DEFINED_H */