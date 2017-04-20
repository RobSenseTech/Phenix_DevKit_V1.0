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
XIicPs xil_iic0;
XIicPs xil_iic1;
SemaphoreHandle_t iic_sem[2];
//iic_priv_s iic_priv;

/***************** Macros (Inline Functions) Definitions *********************/
static void  set_bit(volatile unsigned long *addr , u32 bits);
static void  reset_bit(volatile unsigned long *addr , u32 bits);
static int iicps_clk_enable(u16 iic_id);
static int iicps_soft_reset(u16 iic_id);

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


static int iicps_clk_enable(u16 iic_id)
{
	if(iic_id == 0)
	{
		set_bit(APER_CLK_CTRL , IIC0_CPU_1XCLKACT);
	}
	else if(iic_id == 1)
	{		
		set_bit(APER_CLK_CTRL , IIC1_CPU_1XCLKACT);
	}

	return XST_SUCCESS;
}	

static int iicps_soft_reset(u16 iic_id)
{
	if(iic_id == 0)
	{
		set_bit(IIC_RST_CTRL,IIC0_CPU1x_RST);	
	}
	else if(iic_id == 1)
	{	
		set_bit(IIC_RST_CTRL,IIC1_CPU1x_RST);
	}


	if(iic_id == 0)
	{
		reset_bit(IIC_RST_CTRL,IIC0_CPU1x_RST);	
	}
	else if(iic_id == 1)
	{	
		reset_bit(IIC_RST_CTRL,IIC1_CPU1x_RST);
	}
	return XST_SUCCESS;
}

iic_priv_s * iic_get_priv(u8 iic_id, u8 DevAddr, u32 FsclHz)
{
	XIicPs *inst_ptr = NULL;
    iic_priv_s *iic_priv = NULL;

    iic_priv = (iic_priv_s *)pvPortMalloc(sizeof(iic_priv_s));
    if(iic_priv == NULL)
    {
        pilot_err("malloc failed!!\n");
        return NULL;
    }

	if(iic_id == IIC_ID_0) 
    {
		inst_ptr = &xil_iic0;
    }
	else if(iic_id == IIC_ID_1)
    {
		inst_ptr = &xil_iic1;		
    }
    else
    {
        pilot_err("No such iic id:%d, init failed\n", iic_id);
        return NULL;
    }
	
    iic_priv->iic_mutex = iic_sem[iic_id];
	iic_priv -> iicbus    = inst_ptr;
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
int iic_init(u8 iic_id, u32 FsclHz)
{
	int status;
	XIicPs_Config *xil_config = NULL;
	XIicPs *inst_ptr = NULL;
	
	if(iic_id == IIC_ID_0) 
		inst_ptr = &xil_iic0;
	else if(iic_id == IIC_ID_1)
		inst_ptr = &xil_iic1;		
    else
    {
        pilot_err("No such iic id:%d, init failed\n", iic_id);
        return -1;
    }

    iic_sem[iic_id] = xSemaphoreCreateMutex();
	/*
	* Initialize the IIC driver so that it's ready to use
	*/
	xil_config = XIicPs_LookupConfig(iic_id);
	if (NULL == xil_config) {
		pilot_err(" IIC lookup configuration failed.\n");
		return XST_FAILURE;
	}
		
	status = XIicPs_CfgInitialize(inst_ptr, xil_config,
								  xil_config->BaseAddress);
	if (status != XST_SUCCESS) {
		pilot_err(" IIC configuration initialization failed.\n");
		return XST_FAILURE;
	}
		
	iicps_soft_reset(iic_id);
	iicps_clk_enable(iic_id);	
	
	/*
	* Perform a self-test to check hardware build
	*/
	status = XIicPs_SelfTest(inst_ptr);
	if (status != XST_SUCCESS) {
//		pilot_err(" IIC[%d] self test failed.\r\n",xil_config->DeviceId);
		return XST_FAILURE;
	}
		
	status = XIicPs_SetSClk(inst_ptr, FsclHz);     //configure clocks
	if (status != XST_SUCCESS) {
		pilot_err(" IIC set clk failed.\n");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


int iic_transfer(iic_priv_s *iic_priv, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len, unsigned regaddrflag)
{
	XIicPs *InstancePtr = iic_priv -> iicbus;
	u8 dev = iic_priv -> devaddr;
	u8 reg = iic_priv -> regaddr;		
	u8 buf[32];
	u32 clk = iic_priv -> frequency;
	u32 actclk = XIicPs_GetSClk(InstancePtr);
    portBASE_TYPE xHigherPriorityTaskWoken;
    xHigherPriorityTaskWoken = pdFALSE;
	int status = -1;	

    if((sizeof(buf) < send_len + 1) && (regaddrflag == 1))
    {
        pilot_err("iic send buf too long:%d\n", send_len);
        return XST_FAILURE;
    }
//	pilot_info("actclk is %dKHz\r\n",actclk/1000);
    xSemaphoreTake(iic_priv->iic_mutex, portMAX_DELAY);

	if(clk == 100000)
	{
		if(actclk > 300000)
        {
            status = XIicPs_SetSClk(InstancePtr, clk);
            if (status == XST_SUCCESS)
            {
    //		pilot_info("clock is different, set to %dKHz\r\n",clk/1000);
                xSemaphoreGive(iic_priv->iic_mutex);
                return XST_FAILURE;
            }
        }
	
	}
	else if(clk == 400000)
	{
		if(actclk < 100000)
        {
    		status = XIicPs_SetSClk(InstancePtr, clk);
            if (status == XST_SUCCESS)
            {
    //			pilot_info("clock is different, set to %dKHz\r\n",clk/1000);
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
    pilot_info("iic_priv.frequency = %dKHz\r\n",(int)iic_priv -> frequency/1000);
	pilot_info("iic_priv.devaddr = %x\r\n",(int)iic_priv -> devaddr);	
}




