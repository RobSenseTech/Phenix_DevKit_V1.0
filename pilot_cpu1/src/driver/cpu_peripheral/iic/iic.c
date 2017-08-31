/***************************** Include Files *******************************/
#include "iic.h"
#include "gic/zynq_gic.h"
#include "Phx_define.h"

/************************** Constant Definitions *****************************/
#define APER_CLK_CTRL		(volatile unsigned long *)(0xf800012c)    //AMBA Peripheral Clock Control
#define IIC0_CPU_1XCLKACT	(1 << 18)
#define IIC1_CPU_1XCLKACT	(1 << 19)

#define IIC_RST_CTRL		(volatile unsigned long *)(0xF8000224) //IIC _RST_CTRL
#define IIC0_CPU1x_RST	(1 << 0)
#define IIC1_CPU1x_RST	(1 << 1)


#define IIC_ID_0       (0)
#define IIC_ID_1       (1)
#define ZYNQ_MAX_IIC_BUS_NUM  (2)

#define IIC_DRV_TIMEOUT_US     (10000)
/************************* Variable Definitions *****************************/
static XIicPs xil_iic0;
static XIicPs xil_iic1;

struct iic_describe{
    uint8_t bus_id;
    uint32_t intr_id;
    XIicPs *iic_ps;
    SemaphoreHandle_t trans_sem;
    volatile uint8_t total_err;	/**< Total Error Flag */
#if 0
    volatile uint8_t  send_complete;	/**< Flag to check completion of Transmission */
    volatile uint8_t  recv_complete;	/**< Flag to check completion of Reception */
    volatile uint32_t total_err_count;	/**< Total Error Count Flag */
    volatile uint32_t slave_response;		/**< Slave Response Flag */
#endif
    SemaphoreHandle_t iic_mutex;
};

/* I2C Device Private Data */

typedef struct{
    struct iic_describe *iic_desc;
	uint32_t clk;
	uint32_t act_clk;
	uint8_t devaddr;
}iic_node_s;


static struct iic_describe iic0_desc = {
    .bus_id = 0,
    .intr_id = XPAR_XIICPS_0_INTR,
    .iic_ps = &xil_iic0,
#if 0
    .send_complete = 0,	
    .recv_complete = 0,	
    .total_err_count = 0,
    .slave_response = 0,	
#endif
};

static struct iic_describe iic1_desc = {
    .bus_id = 1,
    .intr_id = XPAR_XIICPS_1_INTR,
    .iic_ps = &xil_iic1,
#if 0
    .send_complete = 0,	
    .recv_complete = 0,	
    .total_err_count = 0,
    .slave_response = 0,	
#endif
};

static struct iic_describe *iic_descs[ZYNQ_MAX_IIC_BUS_NUM] = {&iic0_desc, &iic1_desc};
/*
XIicPs xil_iic0;
XIicPs xil_iic1;
SemaphoreHandle_t iic_mutex[2];
*/
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

/*
 * 如果发送/接收失败,调这个函数清空Transfer_size_reg0,否则配clk会失败
 */
static inline void iicps_clear_transmit(XIicPs* iicps)
{
     *(volatile uint32_t *)(iicps->Config.BaseAddress + XIICPS_TRANS_SIZE_OFFSET) = 0;
}

void *iic_register(uint8_t bus_id, uint8_t dev_addr, uint32_t clk)
{
    struct iic_describe *iic_desc = NULL;
    iic_node_s *iic_node = NULL;
    int32_t status = -1;

    iic_node = (iic_node_s *)pvPortMalloc(sizeof(iic_node_s));
    if(iic_node == NULL)
    {
        pilot_err("malloc failed!!\n");
        return NULL;
    }

    if(bus_id >= ZYNQ_MAX_IIC_BUS_NUM)
    {
        pilot_err("no such iic bus:%d\n", bus_id);
        return NULL;
    }

    iic_desc = iic_descs[bus_id];

    iic_node->iic_desc  = iic_desc;
	iic_node->devaddr   = dev_addr;
	iic_node->clk       = clk;

    xSemaphoreTake(iic_desc->iic_mutex, portMAX_DELAY);

    status = XIicPs_SetSClk(iic_desc->iic_ps, clk);
    if (status != XST_SUCCESS)
    {
        xSemaphoreGive(iic_desc->iic_mutex);
        return NULL;
    }

    /*
     * The actual value may not be exact to integer math rounding errors
     * For example, we set clock HZ as 400000, the actual value in IIC controller is 388500
     */
    iic_node->act_clk = XIicPs_GetSClk(iic_desc->iic_ps);

    xSemaphoreGive(iic_desc->iic_mutex);

    return (void *)iic_node;
}

void iic_deregister(void *iic_node)
{
    iic_node_s *node = (iic_node_s *)iic_node;
    struct iic_describe *iic_desc = node->iic_desc;

    if(iic_node == NULL)
        return;

    xSemaphoreTake(iic_desc->iic_mutex, portMAX_DELAY);
    vPortFree(iic_node);
    xSemaphoreGive(iic_desc->iic_mutex);
}

void iicps_status_hander(void *param, uint32_t event)
{
    struct iic_describe *iic_desc = (struct iic_describe *)param;
    portBASE_TYPE higher_priority_woken;
    higher_priority_woken= pdFALSE;
	/*
	 * All of the data transfer has been finished.
	 */
	if (0 != (event & XIICPS_EVENT_COMPLETE_SEND) || 0 != (event & XIICPS_EVENT_COMPLETE_RECV)) 
    {
        xSemaphoreGiveFromISR(iic_desc->trans_sem, &higher_priority_woken);
    }
    else if (0 != (event & XIICPS_EVENT_ERROR))
    {
		iic_desc->total_err = 1;
	}
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
    struct iic_describe *iic_desc = NULL;
	
	if(bus_id >= ZYNQ_MAX_IIC_BUS_NUM)
    {
        pilot_err("no such iic bus:%d\n", bus_id);
        return -1;
    }

    iic_desc = iic_descs[bus_id];

    iic_desc->iic_mutex = xSemaphoreCreateMutex();
    iic_desc->trans_sem = xSemaphoreCreateBinary();
	/*
	* Initialize the IIC driver so that it's ready to use
	*/
	xil_config = XIicPs_LookupConfig(bus_id);
	if (NULL == xil_config) {
		pilot_err(" IIC lookup configuration failed.\n");
		return XST_FAILURE;
	}
		
	status = XIicPs_CfgInitialize(iic_desc->iic_ps, xil_config,
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
	status = XIicPs_SelfTest(iic_desc->iic_ps);
	if (status != XST_SUCCESS) {
		pilot_err(" IIC[%d] self test failed.\n",bus_id);
		return -1;
	}

//	XIicPs_WriteReg(iic_desc->iic_ps->Config.BaseAddress, XIICPS_TIME_OUT_OFFSET, 255);

	gic_bind_interrupt_to_cpu(iic_desc->intr_id, CPU_1_TARGETED);

	gic_isr_register(iic_desc->intr_id,(Xil_ExceptionHandler) XIicPs_MasterInterruptHandler,(void *)iic_desc->iic_ps);

	gic_interrupt_enable(iic_desc->intr_id);

	XIicPs_SetStatusHandler(iic_desc->iic_ps, (void *)iic_desc, iicps_status_hander);

	status = XIicPs_SetSClk(iic_desc->iic_ps, clk);     //configure clocks
	if (status != XST_SUCCESS) {
		pilot_err(" IIC set clk failed.\n");
		return XST_FAILURE;
	}

	return XST_SUCCESS;
}


#if 0
int32_t iic_transfer(void *iic_node, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
    iic_node_s *node = (iic_node_s *)iic_node;
	int32_t status = -1;	
    int32_t ret = -1;
    
    if(node == NULL)
    {
        pilot_err("invalid iic node!!\n");
        return -1;
    }

    struct iic_describe *iic_desc = node->iic_desc;
	XIicPs *iicps = node->iicps;
	uint8_t dev = node->devaddr;
	uint32_t curr_clk = XIicPs_GetSClk(iicps);
//	pilot_info("actclk is %dKHz\r\n",actclk/1000);
    xSemaphoreTake(iic_desc->iic_mutex, portMAX_DELAY);

    if(curr_clk != node->act_clk)
    {
        status = XIicPs_SetSClk(iicps, node->clk);
        if (status != XST_SUCCESS)
        {
            //pilot_err("set to %dKHz failed transmit size=%d\r\n",node->clk/1000, *(volatile uint32_t *)0xE0005014);
            xSemaphoreGive(iic_desc->iic_mutex);
            return -1;
        }
    }
	
	if(send_len > 0)
	{
		Xil_AssertNonvoid(send != NULL);	
		ret = XIicPs_MasterSendPolled(iicps, (uint8_t *)send, send_len, dev);
        if(ret != XST_SUCCESS)
        {
            iicps_clear_transmit(iicps);
            ret = -1;
        }
	}

	if(recv_len > 0)
	{		
		Xil_AssertNonvoid(recv != NULL);	
        ret = XIicPs_MasterRecvPolled(iicps, recv, recv_len, dev);
        if(ret != XST_SUCCESS)
        {
            iicps_clear_transmit(iicps);
            ret = -1;
        }
	}
    
    xSemaphoreGive(iic_desc->iic_mutex);

	return ret;
}
#else
int32_t iic_transfer(void *iic_node, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len)
{
    iic_node_s *node = (iic_node_s *)iic_node;
    int32_t ret = 0;

    if(node == NULL)
    {
        pilot_err("invalid iic node!!\n");
        return -1;
    }
    
    struct iic_describe *iic_desc = node->iic_desc;
	XIicPs *iicps = iic_desc->iic_ps;
	uint8_t dev = node->devaddr;
	uint32_t curr_clk = XIicPs_GetSClk(iicps);

//	pilot_info("actclk is %dKHz\r\n",actclk/1000);
    xSemaphoreTake(iic_desc->iic_mutex, portMAX_DELAY);

    if(curr_clk != node->act_clk)
    {
        ret = XIicPs_SetSClk(iicps, node->clk);
        if (ret != XST_SUCCESS)
        {
            pilot_err("set to %dKHz failed\n",node->clk/1000);
            ret -1;
            goto out;
        }
    }
	
	if(send_len > 0)
	{
		Xil_AssertNonvoid(send != NULL);	

        /*
         * Send the Data.
         */
		XIicPs_MasterSend(iicps, (uint8_t *)send, send_len, dev);

        /*
         * Wait for the entire buffer to be sent, letting the interrupt
         * processing work in the background.
         */
        ret = xSemaphoreTake(iic_desc->trans_sem, USEC2TICK(IIC_DRV_TIMEOUT_US));
        if (0 != iic_desc->total_err || ret != pdPASS) 
        {
            iicps_clear_transmit(iicps);
            iic_desc->total_err = 0;
            ret = -1;
            goto out;
        }
        else
        {
            ret = 0;
        }

	}

	if(recv_len > 0)
	{		
		Xil_AssertNonvoid(recv != NULL);	

        /*
         * Receive the Data.
         */
        XIicPs_MasterRecv(iicps, recv, recv_len, dev);

        ret = xSemaphoreTake(iic_desc->trans_sem, USEC2TICK(IIC_DRV_TIMEOUT_US));
        if (0 != iic_desc->total_err || ret != pdPASS) 
        {
            iicps_clear_transmit(iicps);
            iic_desc->total_err = 0;
            ret = -1;
            goto out;
        }
        else
        {
            ret = 0;
        }

	}

out:
    
    xSemaphoreGive(iic_desc->iic_mutex);
	return ret;
}

#endif


