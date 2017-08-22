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

#define MIO_34            34
#define MIO_36            36
#define MIO_37            37
#define MIO_35            35
#define MIO_49            49
//#define MIO_UNKOWN        12

/************************** Constant Definitions *****************************/
/* SPI chip selects */
#define GPIO_SPI_CS_GYRO                   MIO_34
#define GPIO_SPI_CS_ACCEL                  MIO_36
#define GPIO_SPI_CS_BARO                   MIO_37
//#define GPIO_SPI_CS_FRAM                   MIO_UNKOWN
#define GPIO_SPI_CS_MAG                    MIO_35

#define GPIO_SPI_CS_EXT                    MIO_49


#define PX4_SPI_BUS_SENSORS	0
#define PX4_SPI_BUS_EXT		1

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
