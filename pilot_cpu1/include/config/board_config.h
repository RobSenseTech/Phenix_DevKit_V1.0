/**
*******************************************************************************
* @file                  board_config.h
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/
#ifndef _PILOT_BOARD_CONFIG_H
#define _PILOT_BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "xparameters.h"
#include "ocm_config.h"

#define MIO_22            22
#define MIO_23            23
#define MIO_24            24
#define MIO_25            25
//#define MIO_UNKOWN        12

/************************** Constant Definitions *****************************/
/* SPI chip selects */
#define GPIO_SPI_CS_GYRO                   MIO_22
#define GPIO_SPI_CS_ACCEL                  MIO_24
#define GPIO_SPI_CS_BARO                   MIO_25
//#define GPIO_SPI_CS_FRAM                   MIO_UNKOWN
#define GPIO_SPI_CS_MAG                    MIO_23


#define PX4_SPI_BUS_SENSORS	1
#define PX4_SPI_BUS_RAMTRON	2
#define PX4_SPI_BUS_EXT		4

#define SPI_DEVICE_ID_FOR_SENSOR		XPAR_PS7_SPI_0_DEVICE_ID


/* I2C busses */
#define PX4_I2C_BUS_EXPANSION	0
#define PX4_I2C_BUS_ONBOARD	1
#define PX4_I2C_BUS_LED		PX4_I2C_BUS_ONBOARD

#define PX4_I2C_OBDEV_LED	0x55
#define PX4_I2C_OBDEV_HMC5883	0x1e

/*****************************SUBS Serial port********************************/
#define RC_SERIAL_PORT      "/dev/uart0"

#define SKETCHNAME "zynq_pilot" 
#ifdef __cplusplus
}
#endif

#endif /* _PILOT_BOARD_CONFIG_H */
