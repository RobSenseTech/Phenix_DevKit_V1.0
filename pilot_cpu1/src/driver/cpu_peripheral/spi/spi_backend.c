/**
*******************************************************************************
* @file                  spi_backend.c
* @author                $Author: Frank Wang <wang990099@163.com> $
*
* Base class for devices connected via SPI.
* Copyright 2016 RobSense. All rights reserved.
*******************************************************************************/

/***************************** Include Files *********************************/
#include "spi_init_api.h"
#include "spi_backend.h"
#include  "gpio/gpio_init_api.h"

#include "xil_exception.h"
#include "board_config.h"
#include "sleep.h"
#include "FreeRTOS_Print.h"

/************************** Constant Definitions *****************************/
#define SPI_0_DEVICE_ID                 0
#define SPI_1_DEVICE_ID                 1
#define SPI_CLOCK_SOURCE_DIVISOR        0x14

#define GPIO_DEVICE_ID  	XPAR_XGPIOPS_0_DEVICE_ID
#define INTC_DEVICE_ID		XPAR_SCUGIC_SINGLE_DEVICE_ID
#define SPI_INTR_ID			XPAR_XSPIPS_1_INTR

/************************** Variable Definitions *****************************/

static u32 Error = 0;

XSpiPs Spi0Instance;			/* The instance of the SPI device */
XSpiPs Spi1Instance;			/* The instance of the SPI device */
SemaphoreHandle_t spi_sem[2] = {NULL};
extern XGpioPs *spi_gpio;
extern XScuGic xInterruptController;
extern XGpioPs GpioInstance;

static u8 isInterruptOn = 0;
/**************************** Type Definitions *******************************/


/***************** Macros (Inline Functions) Definitions *********************/
static int spi_0_slave_slected(struct SSpiDevice *InstancePtr, u8 isSelected);

static int spi_1_slave_slected(struct SSpiDevice *InstancePtr, u8 isSelected);

static int spi_driver_init(XSpiPs *InstancePtr, u16 SpiDeviceId);

static int spi_interrupt_init(XScuGic *IntcInstancePtr,
		XSpiPs *SpiInstancePtr, u16 IntcDeviceId, u16 SpiIntrId);

static int spi_gpio_init(XGpioPs *Gpio, u16 DeviceId);

static void SpiPsHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount);

static int spi_0_disable_slave_select_via_gpio();

static int spi_1_disable_slave_select_via_gpio();

static void spi_interrupt_enable(XScuGic *IntcInstancePtr, u16 SpiIntrId);

static int spi_init_under_poll_mode(struct SSpiDevice *InstancePtr, u16 SpiDeviceId,
		u16 GpioDeviceId);

static int spi_init_under_interrupt_mode(struct SSpiDevice *InstancePtr, u16 SpiDeviceId,
		u16 IntcDeviceId, u16 SpiIntrId, u16 GpioDeviceId);
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
int spi_init(struct SSpiDevice *InstancePtr, u16 SpiDeviceId, u16 IntcDeviceId, u16 SpiIntrId, u16 GpioDeviceId)  {
	if (InstancePtr->Interrupt == NULL) {
		Print_Info("spi init from poll mode\r\n");
		return spi_init_under_poll_mode(InstancePtr, SpiDeviceId, GpioDeviceId);
	} else {
		Print_Info("spi init from interrupt mode\r\n");
		return spi_init_under_interrupt_mode(InstancePtr, SpiDeviceId, IntcDeviceId, SpiIntrId, GpioDeviceId);
	}
}

int DeviceViaSpiCfgInitialize(struct SDeviceViaSpi *instanceptr,
		enum ESpi_device_id device_id,
		char* deviceName,
		enum ESpi_clock_mode mode,
		u32 frequency) {

	if (instanceptr->spi_id == 0) {
		instanceptr->device.Spi = &Spi0Instance;
		instanceptr->device.busname = "SPI 0";
		instanceptr->device.spi_mutex = spi_sem[0];
        Print_Info("sem[0]=%x\n", spi_sem[0]);
	} else if (instanceptr->spi_id == 1) {
		instanceptr->device.Spi = &Spi1Instance;
		instanceptr->device.busname = "SPI 1";
		instanceptr->device.spi_mutex = spi_sem[1];
        Print_Info("sem[1]=%x\n", spi_sem[1]);
	} else {
		Print_Err(" SPI %d Unsupport.\r\n", instanceptr->spi_id);
		return XST_FAILURE;
	}

	instanceptr->device.Gpio = &GpioInstance;
	instanceptr->device.devname = deviceName;
	instanceptr->device.device = device_id;
	instanceptr->device.mode = mode;
	instanceptr->device.frequency = frequency;

	if (isInterruptOn != 0) {
		instanceptr->device.Interrupt = &xInterruptController;
	} else {
		instanceptr->device.Interrupt = NULL;
	}

    return XST_SUCCESS;
}

int spi_init_under_interrupt_mode(struct SSpiDevice *InstancePtr, u16 SpiDeviceId, u16 IntcDeviceId, u16 SpiIntrId, u16 GpioDeviceId) {
	int Status;

	/* SPI bus device init */
	Status = spi_driver_init(InstancePtr->Spi, SpiDeviceId);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI device initialization failed.\r\n");
		return XST_FAILURE;
	}

	/* SPI interrupt init */
	Status = spi_interrupt_init(InstancePtr->Interrupt, InstancePtr->Spi, IntcDeviceId, SpiIntrId);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI interrupt initialization failed.\r\n");
		return XST_FAILURE;
	}

	Status = spi_gpio_init(InstancePtr->Gpio, GpioDeviceId);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI gpio initialization failed.\r\n");
		return XST_FAILURE;
	}

	/*
	 * Setup the handler for the SPI that will be called from the
	 * interrupt context when an SPI status occurs, specify a pointer to
	 * the SPI driver instance as the callback reference so the handler is
	 * able to access the instance data
	 */
	XSpiPs_SetStatusHandler(InstancePtr->Spi, InstancePtr->Spi,
				 (XSpiPs_StatusHandler) SpiPsHandler);

	return XST_SUCCESS;
}

int spi_init_under_poll_mode(struct SSpiDevice *InstancePtr, u16 SpiDeviceId, u16 GpioDeviceId) {
	int Status;

	if (InstancePtr->Interrupt != NULL) {
		Print_Err(" SPI interrupt pointer should be NULL under poll mode.\r\n");
		return XST_FAILURE;
	}

	/* SPI bus device init */
	Status = spi_driver_init(InstancePtr->Spi, SpiDeviceId);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI device initialization failed.\r\n");
		return XST_FAILURE;
	}

	Status = spi_gpio_init(InstancePtr->Gpio, GpioDeviceId);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI gpio initialization failed.\r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

static int spi_driver_init(XSpiPs *SpiInstancePtr, u16 SpiDeviceId) {
	int Status;
	XSpiPs_Config *SpiConfig;

	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
	Status = spi_clock_init((u32)SpiDeviceId, CLOCK_SOURCE_IO_PLL, SPI_CLOCK_SOURCE_DIVISOR);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI clock initialization failed.\r\n");
		return XST_FAILURE;
	}

	/*
	 * Initialize the SPI driver so that it's ready to use
	 */
	SpiConfig = XSpiPs_LookupConfig(SpiDeviceId);
	if (NULL == SpiConfig) {
		Print_Err(" SPI lookup configuration failed.\r\n");
		return XST_FAILURE;
	}

	Status = XSpiPs_CfgInitialize(SpiInstancePtr, SpiConfig,
					SpiConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI configuration initialization failed.\r\n");
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build
	 */
	Status = XSpiPs_SelfTest(SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI self test failed.\r\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}

static int spi_interrupt_init(XScuGic *IntcInstancePtr,
		XSpiPs *SpiInstancePtr, u16 IntcDeviceId, u16 SpiIntrId) {
	int Status;
	XScuGic_Config *IntcConfig;

	/*
	 * Initialize the interrupt controller driver so that it is ready to
	 * use.
	 */
	IntcConfig = XScuGic_LookupConfig(IntcDeviceId);
	if (NULL == IntcConfig) {
		Print_Err(" Interrupt lookup configuration failed.\r\n");
		return XST_FAILURE;
	}

	Status = XScuGic_CfgInitialize(IntcInstancePtr, IntcConfig,
					IntcConfig->CpuBaseAddress);
	if (Status != XST_SUCCESS) {
		Print_Err(" Interrupt configuration initialization failed.\r\n");
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to ensure that the hardware was built
	 * correctly
	 */
	Status = XScuGic_SelfTest(IntcInstancePtr);
	if (Status != XST_SUCCESS) {
		return XST_FAILURE;
	}

	/* enable spi #81 interrupt */
	spi_interrupt_enable(IntcInstancePtr, SpiIntrId);

	/*
	 * Connect the interrupt controller interrupt handler to the hardware
	 * interrupt handling logic in the processor.
	 */
	Xil_ExceptionRegisterHandler(XIL_EXCEPTION_ID_INT,
				(Xil_ExceptionHandler)XScuGic_InterruptHandler,
				IntcInstancePtr);

	/*
	 * Connect the device driver handler that will be called when an
	 * interrupt for the device occurs, the handler defined above performs
	 * the specific interrupt processing for the device.
	 */
	Status = XScuGic_Connect(IntcInstancePtr, SpiIntrId,
				(Xil_ExceptionHandler)XSpiPs_InterruptHandler,
				(void *)SpiInstancePtr);
	if (Status != XST_SUCCESS) {
		Print_Err(" Interrupt connect failed.\r\n");
		return Status;
	}

	/*
	 * Enable the interrupt for the Spi device.
	 */
	XScuGic_Enable(IntcInstancePtr, SpiIntrId);

	/*
	 * Enable interrupts in the Processor.
	 */
	Xil_ExceptionEnable();

	return XST_SUCCESS;
}

static int spi_gpio_init(XGpioPs *Gpio, u16 DeviceId) {
	int Status;
	XGpioPs_Config *ConfigPtr;

	Status = gpio_clock_active_enable();
	if (Status != XST_SUCCESS) {
		Print_Err("GPIO Interrupt clock enable failed \r\n");
		return XST_FAILURE;
	}

	/*
	 * Initialize the GPIO driver.
	 */
	ConfigPtr = XGpioPs_LookupConfig(DeviceId);
	Status = XGpioPs_CfgInitialize(Gpio, ConfigPtr,
					ConfigPtr->BaseAddr);
	if (Status != XST_SUCCESS) {
		Print_Err("XGpioPs_CfgInitialize failed %d\r\n", Status);
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


static int  xSpiPs_Exchange(XSpiPs *InstancePtr, uint8_t *SendBufPtr,uint8_t *RecvBufPtr, uint32_t ByteCount)
{
    uint8_t ucWord;
    uint8_t *pcSrc = SendBufPtr;
    uint8_t *pcDest = RecvBufPtr;
    uint32_t uiRegVal;

	XSpiPs_Enable(InstancePtr);
    while(ByteCount-- > 0)
    {
       if(pcSrc)
          ucWord = *pcSrc++;
       else
          ucWord = 0xff;


	    /*等待tx fifo空*/
        uiRegVal = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);
        while((uiRegVal & XSPIPS_IXR_TXOW_MASK) == 0)
        {
            uiRegVal = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);
        }
        XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_TXD_OFFSET, ucWord);

         uiRegVal = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);
        while((uiRegVal & XSPIPS_IXR_RXNEMPTY_MASK) == 0)
        {
            uiRegVal = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);
        }
        ucWord = (uint8_t)XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_RXD_OFFSET);

        if(pcDest)
            *pcDest++ = ucWord;

        /*清空标志位*/
        uiRegVal = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);
        XSpiPs_WriteReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET, uiRegVal);
        uiRegVal = XSpiPs_ReadReg(InstancePtr->Config.BaseAddress, XSPIPS_SR_OFFSET);

    }

	XSpiPs_Disable(InstancePtr);
    spi_disable_slave_via_gpio(InstancePtr);
    return 0;
}

int spi_transfer(struct SSpiDevice *InstancePtr, u8 *SendBufPtr,
				u8 *RecvBufPtr, u32 ByteCount) {
	int Status;

	Status = spi_slave_select(InstancePtr, 0x1);
	if (Status != XST_SUCCESS) {
		Print_Err("SPI set slave select Failed %d\r\n", Status);
		return XST_FAILURE;
	}

	if (InstancePtr->Interrupt != NULL) {
		return XSpiPs_Transfer(InstancePtr->Spi, SendBufPtr, RecvBufPtr, ByteCount);
	} else {
    #if 1
		return XSpiPs_PolledTransfer(InstancePtr->Spi, SendBufPtr, RecvBufPtr, ByteCount);
    #else
        return xSpiPs_Exchange(InstancePtr->Spi, SendBufPtr, RecvBufPtr, ByteCount);
    #endif
	}
}

void spi_destroy(struct SSpiDevice *InstancePtr, u16 SpiIntrId) {
	/*
	 * Disable the interrupt for the SPI device.
	 */
	XScuGic_Disable(InstancePtr->Interrupt, SpiIntrId);
	/*
	 * Disconnect and disable the interrupt for the Spi device.
	 */
	XScuGic_Disconnect(InstancePtr->Interrupt, SpiIntrId);
}

/******************************************************************************
*
* This function is the handler which performs processing for the SPI driver.
* It is called from an interrupt context such that the amount of processing
* performed should be minimized.  It is called when a transfer of SPI data
* completes or an error occurs.
*
* This handler provides an example of how to handle SPI interrupts but is
* application specific.
*
* @param	CallBackRef is a reference passed to the handler.
* @param	StatusEvent is the status of the SPI .
* @param	ByteCount is the number of bytes transferred.
*
* @return	None
*
* @note		None.
*
******************************************************************************/
void SpiPsHandler(void *CallBackRef, u32 StatusEvent, unsigned int ByteCount)
{
	/*
	 * Indicate the transfer on the SPI bus is no longer in progress
	 * regardless of the status event
	 */
	//TransferInProgress = FALSE;

	/*
	 * If the event was not transfer done, then track it as an error
	 */
	if (StatusEvent != XST_SPI_TRANSFER_DONE) {
		Error++;
		Print_Err(" Error number %d.\r\n", Error);
	}
	else {
		Print_Info(" tranfer done.\r\n");
	}
}

int spi_slave_select(struct SSpiDevice *InstancePtr, u8 isSelected){

	int Status = XST_SUCCESS;

	if (InstancePtr->device == ESPI_DEVICE_TYPE_NONE)
	{
		Print_Err(" No Device mount on this instance. \r\n");
		return XST_FAILURE;
	}

	if (InstancePtr->Spi->Config.DeviceId == 0) {
		Status = spi_0_slave_slected(InstancePtr, isSelected);
	} else if (InstancePtr->Spi->Config.DeviceId == 1) {
		Status = spi_1_slave_slected(InstancePtr, isSelected);
	} else {
		Print_Err(" Unsupport SPI device id %d. \r\n", InstancePtr->Spi->Config.DeviceId);
		Status = XST_FAILURE;
	}
	return Status;
}

static int spi_0_slave_slected(struct SSpiDevice *InstancePtr, u8 isSelected) {

	int Status = XST_SUCCESS;

	if ((InstancePtr->device >= ESPI_DEVICE_TYPE_INTERNAL_CS_0)
			&& (InstancePtr->device < ESPI_DEVICE_TYPE_END)) {
		Print_Info(" Device use SPI 0 internal CS %d selected. \r\n", (InstancePtr->device % 100));

		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
		//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);

		Status = XSpiPs_SetSlaveSelect(InstancePtr->Spi, (u8)(InstancePtr->device % 100));
	} else {
		switch (InstancePtr->device) {
			case ESPI_DEVICE_TYPE_GYRO:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, ((~isSelected) & 0x01));
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_ACCEL:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, ((~isSelected) & 0x01));
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_BARO:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, ((~isSelected) & 0x01));
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_FRAM:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, ((~isSelected) & 0x01));
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_MAG:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, ((~isSelected) & 0x01));
				break;

			default:
				Print_Err(" Unknown slave, CS %d selected. \r\n", InstancePtr->device);
				Status = XST_FAILURE;
				break;
		}

		if (Status != XST_FAILURE) {
			Status = XSpiPs_SetSlaveSelect(InstancePtr->Spi, (u8)SPI_NO_INTERNAL_SLAVE_SELECTED);
		}
	}
	return Status;
}

static int spi_1_slave_slected(struct SSpiDevice *InstancePtr, u8 isSelected) {

	int Status = XST_SUCCESS;

	if ((InstancePtr->device >= ESPI_DEVICE_TYPE_INTERNAL_CS_0)
			&& (InstancePtr->device < ESPI_DEVICE_TYPE_END)) {
		Print_Info(" Device use SPI 0 internal CS %d selected. \r\n", (InstancePtr->device % 100));

		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
		//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
		XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);

		Status = XSpiPs_SetSlaveSelect(InstancePtr->Spi, (u8)(InstancePtr->device % 100));
	} else {
		switch (InstancePtr->device) {
			case ESPI_DEVICE_TYPE_GYRO:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, ((~isSelected) & 0x01));
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_ACCEL:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, ((~isSelected) & 0x01));
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_BARO:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, ((~isSelected) & 0x01));
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_FRAM:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, ((~isSelected) & 0x01));
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, 0x1);
				break;

			case ESPI_DEVICE_TYPE_MAG:
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_GYRO, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_ACCEL, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_BARO, 0x1);
				//XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_FRAM, 0x1);
				XGpioPs_WritePin(InstancePtr->Gpio, GPIO_SPI_CS_MAG, ((~isSelected) & 0x01));
				break;

			default:
				Print_Err(" Unknown slave, CS %d selected. \r\n", InstancePtr->device);
				Status = XST_FAILURE;
				break;
		}

		if (Status != XST_FAILURE) {
			Status = XSpiPs_SetSlaveSelect(InstancePtr->Spi, (u8)SPI_NO_INTERNAL_SLAVE_SELECTED);
		}
	}
	return Status;
}

void spi_set_frequency(struct SSpiDevice *InstancePtr, u32 frequency) {
	InstancePtr->frequency = frequency;
	Print_Info(" %s slave frequency set to %d Hz.\r\n", InstancePtr->devname, InstancePtr->frequency);
}

int spi_set_clock_prescaler(struct SSpiDevice *InstancePtr) {
	// SPI reference clock have been set (u32)((1000*1000*1000)/13) 76MHz.
	int Status = XST_SUCCESS;
	if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_4) {
		Print_Err(" Unsupport frequency %d Hz, %s frequency should less than %d Hz.\r\n", InstancePtr->frequency, InstancePtr->busname, SPI_REFERENCE_CLOCK_DIVIDED_BY_4);
		Status = XST_FAILURE;
	} else if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_8) {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_4);
	} else if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_16) {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_8);
	} else if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_32) {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_16);
	} else if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_64) {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_32);
	} else if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_128) {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_64);
	} else if (InstancePtr->frequency > SPI_REFERENCE_CLOCK_DIVIDED_BY_256) {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_128);
	} else {
		Status = XSpiPs_SetClkPrescaler(InstancePtr->Spi, XSPIPS_CLK_PRESCALE_256);
	}

	return 	Status;
}

u8 spi_get_clock_prescaler(struct SSpiDevice *InstancePtr) {
	return XSpiPs_GetClkPrescaler(InstancePtr->Spi);
}

int spi_disable_slave_via_gpio(XSpiPs *InstancePtr) {
	int Status = XST_SUCCESS;

	if (InstancePtr->Config.DeviceId == 0) {
		Status = spi_0_disable_slave_select_via_gpio();
	} else if (InstancePtr->Config.DeviceId == 1) {
		Status = spi_1_disable_slave_select_via_gpio();
	} else {
		Print_Err(" Unsupport SPI device id %d. \r\n", InstancePtr->Config.DeviceId);
		Status = XST_FAILURE;
	}
	return Status;
}

static int spi_0_disable_slave_select_via_gpio() {
	int Status = XST_SUCCESS;

	{
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_GYRO, 0x1);
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_ACCEL, 0x1);
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_BARO, 0x1);
		//XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_FRAM, 0x1);
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_MAG, 0x1);
	}
	return Status;
}

static int spi_1_disable_slave_select_via_gpio() {
	int Status = XST_SUCCESS;

	{
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_GYRO, 0x1);
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_ACCEL, 0x1);
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_BARO, 0x1);
		//XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_FRAM, 0x1);
		XGpioPs_WritePin(spi_gpio, GPIO_SPI_CS_MAG, 0x1);
	}
	return Status;
}

static void spi_interrupt_enable(XScuGic *IntcInstancePtr, u16 SpiIntrId) {
	u32 RegValue;
	/*
	 * 3. The CPU interface in the spi_target register
	 * Only write to the SPI interrupts, so start at 32
	 */
	RegValue = XScuGic_DistReadReg(IntcInstancePtr, XSCUGIC_SPI_TARGET_OFFSET_CALC(SpiIntrId));
	RegValue &= 0xFFFF00FF;
	RegValue |= 0x00000200;
	XScuGic_DistWriteReg(IntcInstancePtr, XSCUGIC_SPI_TARGET_OFFSET_CALC(SpiIntrId), RegValue);


	/*
	 * 4. Enable the SPI using the enable_set register.
	 */
	XScuGic_EnableIntr((IntcInstancePtr)->Config->DistBaseAddress, (u32)SpiIntrId);
}

int SpiPsInit(u16 Spi_Id, u8 isIntcOn) {
	int Status = XST_SUCCESS;
	XSpiPs *SpiInstancePtr = NULL;

	if (Spi_Id == 0) {
		SpiInstancePtr = &Spi0Instance;
	} else if (Spi_Id == 1) {
		SpiInstancePtr = &Spi1Instance;
	} else {
		Print_Err(" %d SPI unsupport.\r\n", Spi_Id);
		return XST_FAILURE;
	}

    spi_sem[Spi_Id] = xSemaphoreCreateMutex();
    Print_Info("sem[0]=%x sem[1]=%x\n", spi_sem[0], spi_sem[1]);

	Status = spi_driver_init(SpiInstancePtr, Spi_Id);
	if (Status != XST_SUCCESS) {
		Print_Err(" SPI device initialization failed.\r\n");
		return XST_FAILURE;
	}

	if (isIntcOn != 0) {

		/*
		* Enable interrupts in the Processor.
		*/
		Xil_ExceptionDisable();

		/* enable spi #81 interrupt */
		spi_interrupt_enable(&xInterruptController, SPI_INTR_ID);

		/*
		* Connect the device driver handler that will be called when an
		* interrupt for the device occurs, the handler defined above performs
		* the specific interrupt processing for the device.
		*/
		Status = XScuGic_Connect(&xInterruptController, SPI_INTR_ID,
					(Xil_ExceptionHandler)XSpiPs_InterruptHandler,
					(void *)SpiInstancePtr);
		if (Status != XST_SUCCESS) {
			Print_Err(" Interrupt connect failed.\r\n");
			return Status;
		}
		/*
		* Enable the interrupt for the Spi device.
		*/
		XScuGic_Enable(&xInterruptController, SPI_INTR_ID);
		/*
		* Enable interrupts in the Processor.
		*/
		Xil_ExceptionEnable();

		/*
		 * Setup the handler for the SPI that will be called from the
		 * interrupt context when an SPI status occurs, specify a pointer to
		 * the SPI driver instance as the callback reference so the handler is
		 * able to access the instance data
		 */
		XSpiPs_SetStatusHandler(SpiInstancePtr, SpiInstancePtr,
					 (XSpiPs_StatusHandler) SpiPsHandler);

		isInterruptOn = 0x1;
	} else {
		isInterruptOn = 0x0;
	}

	if (spi_gpio != NULL) {
		gpio_port_output_config(spi_gpio, GPIO_SPI_CS_GYRO);
		gpio_port_output_config(spi_gpio, GPIO_SPI_CS_ACCEL);
		gpio_port_output_config(spi_gpio, GPIO_SPI_CS_BARO);
		gpio_port_output_config(spi_gpio, GPIO_SPI_CS_MAG);
	} else {
		Print_Err("Sensor GPIO didn't config %d\r\n", Status);
	}
	spi_disable_slave_via_gpio(SpiInstancePtr);

	XSpiPs_SetOptions(SpiInstancePtr, /*XSPIPS_MANUAL_START_OPTION |*/ \
			XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION | XSPIPS_CLK_PHASE_1_OPTION | XSPIPS_CLK_ACTIVE_LOW_OPTION);

	XSpiPs_Enable(SpiInstancePtr);
	return Status;
}

int SpiTransfer(struct SDeviceViaSpi *instanceptr, u8 *send, u8 *recv, u32 len) {
	int Status;
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;

//    Print_Warn("devname-%s instanceptr=%x device=%x mutex=%x\n", instanceptr->device.devname, instanceptr, &instanceptr->device, instanceptr->device.spi_mutex);
    xSemaphoreTake(instanceptr->device.spi_mutex, portMAX_DELAY);
	Status = spi_set_clock_prescaler(&(instanceptr->device));
	if (Status != XST_SUCCESS) {
		Print_Err("SPI set clock prescaler Failed %d\r\n", Status);
		return XST_FAILURE;
	}
    Status =  spi_transfer(&(instanceptr->device), send, recv, len);
    xSemaphoreGive(instanceptr->device.spi_mutex);
	return Status;
}

void SpiSetFrequency(struct SDeviceViaSpi *instanceptr, u32 frequency) {
	spi_set_frequency(&(instanceptr->device), frequency);
}
