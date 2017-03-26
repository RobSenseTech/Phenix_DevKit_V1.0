/**
*******************************************************************************
* @file                  spi_backend.h
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/
#ifndef _DEVICE_SPI_BACKEND_H
#define _DEVICE_SPI_BACKEND_H

#ifdef __cplusplus
extern "C" {
#endif

/***************************** Include Files *********************************/
#include "spi_init_api.h"

#include "xspips.h"
#include "xscugic.h"
#include "xstatus.h"
#include "xgpiops.h"

#include "xil_printf.h"
#include "FreeRTOS.h"
#include "semphr.h"

/************************** Constant Definitions *****************************/

	/**
	 * Constructor
	 *
	 * @param Spi           SPI Driver pointer
	 * @param Interrupt     Interrupt pointer
	 * @param name		    Driver name
	 * @param devname       Device node name
	 * @param device	    Device id (used by SPI_SELECT)
	 * @param mode		    SPI clock/data mode
	 * @param frequency	    SPI clock frequency
	 * 
	 */

enum ESpi_device_id {
	ESPI_DEVICE_TYPE_NONE         = 0,
	ESPI_DEVICE_TYPE_GYRO         = 1,
	ESPI_DEVICE_TYPE_ACCEL        = 2,
	ESPI_DEVICE_TYPE_BARO         = 3,
	ESPI_DEVICE_TYPE_MAG          = 4,
	ESPI_DEVICE_TYPE_FRAM         = 5,
	
	ESPI_DEVICE_TYPE_INTERNAL_CS_0  = 100,
	ESPI_DEVICE_TYPE_INTERNAL_CS_1  = 101,
	ESPI_DEVICE_TYPE_INTERNAL_CS_2  = 102,
	ESPI_DEVICE_TYPE_END
};

enum ESpi_clock_mode {
  ESPI_CLOCK_MODE_0 = 0,       /* CPOL=0 CHPHA=0 */
  ESPI_CLOCK_MODE_1 = 1,       /* CPOL=0 CHPHA=1 */
  ESPI_CLOCK_MODE_2 = 2,       /* CPOL=1 CHPHA=0 */
  ESPI_CLOCK_MODE_3 = 3        /* CPOL=1 CHPHA=1 */
};

struct SSpiDevice {
	XSpiPs* Spi;
	XScuGic* Interrupt;
	XGpioPs* Gpio;
	const char* busname;
	const char* devname;
	enum ESpi_device_id device;
	enum ESpi_clock_mode mode;
	u32 frequency;
    xSemaphoreHandle spi_mutex; 
};

struct SDeviceViaSpi {
	u8 spi_id;
	struct SSpiDevice device;
};

#define SPI_DEVICE_ID		XPAR_XSPIPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define SPI_INTR_ID		XPAR_XSPIPS_1_INTR

#define SPI_NO_INTERNAL_SLAVE_SELECTED      0xFU

#define SPI_REFERENCE_CLOCK                (u32)((1000*1000*1000)/20)

#define SPI_REFERENCE_CLOCK_DIVIDED_BY_4         (u32)(SPI_REFERENCE_CLOCK/4)
#define SPI_REFERENCE_CLOCK_DIVIDED_BY_8         (u32)(SPI_REFERENCE_CLOCK/8)
#define SPI_REFERENCE_CLOCK_DIVIDED_BY_16        (u32)(SPI_REFERENCE_CLOCK/16)
#define SPI_REFERENCE_CLOCK_DIVIDED_BY_32        (u32)(SPI_REFERENCE_CLOCK/32)
#define SPI_REFERENCE_CLOCK_DIVIDED_BY_64        (u32)(SPI_REFERENCE_CLOCK/64)
#define SPI_REFERENCE_CLOCK_DIVIDED_BY_128       (u32)(SPI_REFERENCE_CLOCK/128)
#define SPI_REFERENCE_CLOCK_DIVIDED_BY_256       (u32)(SPI_REFERENCE_CLOCK/256)

/** @name SPI Clock Prescaler options
 * The SPI Clock Prescaler Configuration bits are used to program master mode
 * bit rate. The bit rate can be programmed in divide-by-two decrements from
 * pclk/4 to pclk/256.
 *
 * @{
 */
// this part defined in xspips.h
//#define XSPIPS_CLK_PRESCALE_4         0x01U  /**< PCLK/4 Prescaler */
//#define XSPIPS_CLK_PRESCALE_8         0x02U  /**< PCLK/8 Prescaler */
//#define XSPIPS_CLK_PRESCALE_16        0x03U  /**< PCLK/16 Prescaler */
//#define XSPIPS_CLK_PRESCALE_32        0x04U  /**< PCLK/32 Prescaler */
//#define XSPIPS_CLK_PRESCALE_64        0x05U  /**< PCLK/64 Prescaler */
//#define XSPIPS_CLK_PRESCALE_128       0x06U  /**< PCLK/128 Prescaler */
//#define XSPIPS_CLK_PRESCALE_256       0x07U  /**< PCLK/256 Prescaler */

/************************** Variable Definitions *****************************/

/***************** Macros (Inline Functions) Definitions *********************/


/************************** Function Prototypes ******************************/
	/**
	 * initialize spi device
	 */
int spi_init(struct SSpiDevice *InstancePtr, u16 SpiDeviceId, u16 IntcDeviceId, u16 SpiIntrId, u16 GpioDeviceId);

	/**
	 * destory spi device
	 */
void spi_destroy(struct SSpiDevice *InstancePtr, u16 SpiIntrId);

	/**
	 * Perform a SPI transfer.
	 *
	 * If called from interrupt context, this interface does not lock
	 * the bus and may interfere with non-interrupt-context callers.
	 *
	 * Clients in a mixed interrupt/non-interrupt configuration must
	 * ensure appropriate interlocking.
	 *
	 * At least one of send or recv must be non-null.
	 *
	 * @param send		Bytes to send to the device, or nullptr if
	 *			no data is to be sent.
	 * @param recv		Buffer for receiving bytes from the device,
	 *			or nullptr if no bytes are to be received.
	 * @param len		Number of bytes to transfer.
	 * @return		OK if the exchange was successful, -errno
	 *			otherwise.
	 */
int SpiTransfer(struct SDeviceViaSpi *instanceptr, u8 *send, u8 *recv, u32 len);
int spi_transfer(struct SSpiDevice *InstancePtr, u8 *send, u8 *recv, u32 len);

	/**
	 * Set the SPI bus frequency
	 * This is used to change frequency on the fly. Some sensors
	 * (such as the MPU6000) need a lower frequency for setup
	 * registers and can handle higher frequency for sensor
	 * value registers
	 *
	 * @param frequency	Frequency to set (Hz)
	 */
void spi_set_frequency(struct SSpiDevice *InstancePtr, u32 frequency);
void SpiSetFrequency(struct SDeviceViaSpi *instanceptr, u32 frequency);

int spi_set_clock_prescaler(struct SSpiDevice *InstancePtr);

u8 spi_get_clock_prescaler(struct SSpiDevice *InstancePtr);

int spi_slave_select(struct SSpiDevice *InstancePtr, u8 selected);

int spi_disable_slave_via_gpio(XSpiPs *InstancePtr);

int SpiPsInit(u16 Spi_Id, u8 isIntcOn);

int DeviceViaSpiCfgInitialize(struct SDeviceViaSpi *instanceptr, 
		enum ESpi_device_id device_id,
		const char* deviceName,
		enum ESpi_clock_mode mode,
		u32 frequency);

#ifdef __cplusplus
}
#endif

#endif /* _DEVICE_SPI_BACKEND_H */
