/***************************** Include Files *******************************/
#include "iic.h"

/* I2C Device Private Data */

typedef struct{
    uint8_t bus_id;
	XIicPs* iicps; 
	uint32_t clk;
	uint32_t act_clk;
	uint8_t devaddr;
    xSemaphoreHandle iic_mutex;
}iic_priv_s;

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

void *iic_register(uint8_t bus_id, uint8_t dev_addr, uint32_t clk)
{
	XIicPs *iicps = NULL;
    iic_priv_s *iic_priv = NULL;
    int32_t status = -1;

    iic_priv = (iic_priv_s *)pvPortMalloc(sizeof(iic_priv_s));
    if(iic_priv == NULL)
    {
        pilot_err("malloc failed!!\n");
        return NULL;
    }

	if(bus_id == IIC_ID_0) 
    {
		iicps = &xil_iic0;
    }
	else if(bus_id == IIC_ID_1)
    {
		iicps = &xil_iic1;		
    }
    else
    {
        pilot_err("No such iic id:%d, init failed\n", bus_id);
        return NULL;
    }
	
    iic_priv->iic_mutex = iic_sem[bus_id];
	iic_priv->iicps     = iicps;
	iic_priv->devaddr   = dev_addr;
	iic_priv->clk       = clk;

    status = XIicPs_SetSClk(iicps, clk);
    if (status != XST_SUCCESS)
    {
        return XST_FAILURE;
    }

    /*
     * The actual value may not be exact to integer math rounding errors
     * For example, we set clock HZ as 400000, the actual value in IIC controller is 388500
     */
    iic_priv->act_clk = XIicPs_GetSClk(iicps);

    return (void *)iic_priv;
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
int32_t iic_init(uint8_t bus_id, uint32_t clk)
{
	int status;
	XIicPs_Config *xil_config = NULL;
	XIicPs *iicps = NULL;
	
	if(bus_id == IIC_ID_0) 
		iicps = &xil_iic0;
	else if(bus_id == IIC_ID_1)
		iicps = &xil_iic1;		
    else
    {
        pilot_err("No such iic id:%d, init failed\n", bus_id);
        return -1;
    }

    iic_sem[bus_id] = xSemaphoreCreateMutex();
	/*
	* Initialize the IIC driver so that it's ready to use
	*/
	xil_config = XIicPs_LookupConfig(bus_id);
	if (NULL == xil_config) {
		pilot_err(" IIC lookup configuration failed.\n");
		return XST_FAILURE;
	}
		
	status = XIicPs_CfgInitialize(iicps, xil_config,
								  xil_config->BaseAddress);
	if (status != XST_SUCCESS) {
		pilot_err(" IIC configuration initialization failed.\n");
		return XST_FAILURE;
	}
		
	iicps_soft_reset(bus_id);
	iicps_clk_enable(bus_id);	
	
	/*
	* Perform a self-test to check hardware build
	*/
	status = XIicPs_SelfTest(iicps);
	if (status != XST_SUCCESS) {
//		pilot_err(" IIC[%d] self test failed.\r\n",xil_config->DeviceId);
		return XST_FAILURE;
	}
		
	status = XIicPs_SetSClk(iicps, clk);     //configure clocks
	if (status != XST_SUCCESS) {
		pilot_err(" IIC set clk failed.\n");
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


int32_t iic_transfer(void *iic_priv, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
    iic_priv_s *priv = (iic_priv_s *)iic_priv;
	XIicPs *iicps = priv->iicps;
	uint8_t dev = priv->devaddr;
	uint32_t curr_clk = XIicPs_GetSClk(iicps);
	int status = -1;	
    
//	pilot_info("actclk is %dKHz\r\n",actclk/1000);
    xSemaphoreTake(priv->iic_mutex, portMAX_DELAY);

    if(curr_clk != priv->act_clk)
    {
        status = XIicPs_SetSClk(iicps, priv->clk);
        if (status != XST_SUCCESS)
        {
//		pilot_info("clock is different, set to %dKHz\r\n",clk/1000);
            xSemaphoreGive(priv->iic_mutex);
            return XST_FAILURE;
        }
    }
	
	if(send_len > 0)
	{
		Xil_AssertNonvoid(send != NULL);	
		XIicPs_MasterSendPolled(iicps, (uint8_t *)send, send_len, dev);
	}

	if(recv_len > 0)
	{		
		Xil_AssertNonvoid(recv != NULL);	
        XIicPs_MasterRecvPolled(iicps, recv, recv_len, dev);
	}
    
    xSemaphoreGive(priv->iic_mutex);

	return XST_SUCCESS;
}



