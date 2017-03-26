/***************************** Include Files *******************************/
#include "iic.h"

/************************** Constant Definitions *****************************/
#define APER_CLK_CTRL		(volatile unsigned long *)(0xf800012c)    //AMBA Peripheral Clock Control
#define IIC0_CPU_1XCLKACT	(1 << 18)
#define IIC1_CPU_1XCLKACT	(1 << 19)

#define IIC_RST_CTRL		(volatile unsigned long *)(0xF8000224) //IIC _RST_CTRL
#define IIC0_CPU1x_RST	(1 << 0)
#define IIC1_CPU1x_RST	(1 << 1)


#define IIC_ID_0       0
#define IIC_ID_1       1

/************************* Variable Definitions *****************************/
XIicPs Iic0Instance;
XIicPs Iic1Instance;
SemaphoreHandle_t IicSem[2];
//iic_priv_s iic_priv;

/***************** Macros (Inline Functions) Definitions *********************/
static void  set_bit(volatile unsigned long *addr , u32 bits);
static void  reset_bit(volatile unsigned long *addr , u32 bits);
static int IicPsClkEnable(u16 iIicId);
static int Iic_Reset_Software(u16 iIicId);

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

static void  set_bit(volatile unsigned long *addr , u32 bits)
{
		u32 uiVal;
		uiVal = *addr;
		uiVal |= bits;
		*addr = uiVal;
}

static void  reset_bit(volatile unsigned long *addr , u32 bits)
{
	u32 uiVal;
	uiVal = *addr;
	uiVal &= ~bits;
	*addr = uiVal;
}


static int IicPsClkEnable(u16 iIicId)
{
	if(iIicId == 0)
	{
		set_bit(APER_CLK_CTRL , IIC0_CPU_1XCLKACT);
	}
	else if(iIicId == 1)
	{		
		set_bit(APER_CLK_CTRL , IIC1_CPU_1XCLKACT);
	}

	return XST_SUCCESS;
}	

static int Iic_Reset_Software(u16 iIicId)
{
	if(iIicId == 0)
	{
		set_bit(IIC_RST_CTRL,IIC0_CPU1x_RST);	
	}
	else if(iIicId == 1)
	{	
		set_bit(IIC_RST_CTRL,IIC1_CPU1x_RST);
	}


	if(iIicId == 0)
	{
		reset_bit(IIC_RST_CTRL,IIC0_CPU1x_RST);	
	}
	else if(iIicId == 1)
	{	
		reset_bit(IIC_RST_CTRL,IIC1_CPU1x_RST);
	}
	return XST_SUCCESS;
}

iic_priv_s * Iic_GetPriv(u8 iIicId, u8 DevAddr, u32 FsclHz)
{
	XIicPs *IicInstancePtr = NULL;
    iic_priv_s *iic_priv = NULL;

    iic_priv = (iic_priv_s *)pvPortMalloc(sizeof(iic_priv_s));
    if(iic_priv == NULL)
    {
        Print_Err("malloc failed!!\n");
        return NULL;
    }

	if(iIicId == IIC_ID_0) 
    {
		IicInstancePtr = &Iic0Instance;
    }
	else if(iIicId == IIC_ID_1)
    {
		IicInstancePtr = &Iic1Instance;		
    }
    else
    {
        Print_Err("No such iic id:%d, init failed\n", iIicId);
        return NULL;
    }
	
    iic_priv->iic_mutex = IicSem[iIicId];
	iic_priv -> iicbus    = IicInstancePtr;
	iic_priv -> devaddr   = DevAddr;
	iic_priv -> frequency = FsclHz;
	iic_priv -> regaddr = 0;	

    return iic_priv;
}

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
int Iic_Init(u8 iIicId, u32 FsclHz)
{
	int Status;
	XIicPs_Config *IicConfig = NULL;
	XIicPs *IicInstancePtr = NULL;
	
	if(iIicId == IIC_ID_0) 
		IicInstancePtr = &Iic0Instance;
	else if(iIicId == IIC_ID_1)
		IicInstancePtr = &Iic1Instance;		
    else
    {
        Print_Err("No such iic id:%d, init failed\n", iIicId);
        return -1;
    }

    IicSem[iIicId] = xSemaphoreCreateMutex();
	/*
	* Initialize the IIC driver so that it's ready to use
	*/
	IicConfig = XIicPs_LookupConfig(iIicId);
	if (NULL == IicConfig) {
		Print_Err(" IIC lookup configuration failed.\n");
		return XST_FAILURE;
	}
		
	Status = XIicPs_CfgInitialize(IicInstancePtr, IicConfig,
								  IicConfig->BaseAddress);
	if (Status != XST_SUCCESS) {
		Print_Err(" IIC configuration initialization failed.\n");
		return XST_FAILURE;
	}
		
	Iic_Reset_Software(iIicId);
	IicPsClkEnable(iIicId);	
	
	/*
	* Perform a self-test to check hardware build
	*/
	Status = XIicPs_SelfTest(IicInstancePtr);
	if (Status != XST_SUCCESS) {
//		xil_printf(" IIC[%d] self test failed.\r\n",IicConfig->DeviceId);
		return XST_FAILURE;
	}
		
	Status = XIicPs_SetSClk(IicInstancePtr, FsclHz);     //configure clocks
	if (Status != XST_SUCCESS) {
		Print_Err(" IIC set clk failed.\n");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


int Iic_transfer(iic_priv_s *iic_priv, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len, unsigned regaddrflag)
{
	XIicPs *InstancePtr = iic_priv -> iicbus;
	u8 dev = iic_priv -> devaddr;
	u8 reg = iic_priv -> regaddr;		
	u8 buf[32];
	u32 clk = iic_priv -> frequency;
	u32 actclk = XIicPs_GetSClk(InstancePtr);
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
	

	int Status = -1;	

//	Print_Info("actclk is %dKHz\r\n",actclk/1000);
    xSemaphoreTake(iic_priv->iic_mutex, portMAX_DELAY);

	if(clk == 100000)
	{
		if(actclk > 300000)
        {
            Status = XIicPs_SetSClk(InstancePtr, clk);
            if (Status == XST_SUCCESS)
            {
    //		Print_Info("clock is different, set to %dKHz\r\n",clk/1000);
                xSemaphoreGive(iic_priv->iic_mutex);
                return XST_FAILURE;
            }
        }
	
	}
	else if(clk == 400000)
	{
		if(actclk < 100000)
        {
    		Status = XIicPs_SetSClk(InstancePtr, clk);
            if (Status == XST_SUCCESS)
            {
    //			Print_Info("clock is different, set to %dKHz\r\n",clk/1000);
                xSemaphoreGive(iic_priv->iic_mutex);
                return XST_FAILURE;
            }    
        }
	
	}

	
	if(send_len > 0)
	{
		Xil_AssertNonvoid(send != NULL);	
		if(regaddrflag == 1)
		{
			buf[0] = reg;
			memcpy(&buf[1], send, send_len);
			XIicPs_MasterSendPolled(InstancePtr, buf,
								send_len+1, dev);
		}
		else 
			XIicPs_MasterSendPolled(InstancePtr, (u8 *)send,
									send_len, dev);
	}
	if(recv_len > 0)
	{		
		Xil_AssertNonvoid(recv != NULL);	
		if(regaddrflag == 1)
		{
			XIicPs_MasterSendPolled(InstancePtr, &reg,
								1, dev);		
			XIicPs_MasterRecvPolled(InstancePtr, recv,
								recv_len, dev);
		}
		else 
			XIicPs_MasterRecvPolled(InstancePtr, recv,
									recv_len, dev);
	}
    
    xSemaphoreGive(iic_priv->iic_mutex);

	return XST_SUCCESS;
}

void Iic_set_address(iic_priv_s *iic_priv, u8 reg_addr)
{
	iic_priv -> regaddr = reg_addr;
}

void debug_print(iic_priv_s *iic_priv)
{
    Print_Info("iic_priv.frequency = %dKHz\r\n",(int)iic_priv -> frequency/1000);
	Print_Info("iic_priv.devaddr = %x\r\n",(int)iic_priv -> devaddr);	
}




