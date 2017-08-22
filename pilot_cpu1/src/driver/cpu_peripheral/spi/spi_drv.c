/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    spi_drv.c
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-08-12 14:03:49
 *Last Modify: 2017-08-12 19:29:09
 *Description: 
**********************************************************************************/

#include "spi_drv.h"

#define ZYNQ_MAX_SPI_BUS_NUM     (2)

//SPI时钟使能
#define SPI_CLK_CTRL		(volatile unsigned long *)(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x158)
#define SPI_CLOCK_SOURCE_DIVISOR    (0x14)
#define SPI_REFERENCE_CLOCK                ((1000*1000*1000)/SPI_CLOCK_SOURCE_DIVISOR)

//外设时钟使能
#define APER_CLK_CTRL		(volatile unsigned long *)(XPAR_PS7_SLCR_0_S_AXI_BASEADDR + 0x12c) //AMBA Peripheral Clock Control

struct spi_describe {
    uint8_t bus_id;
    XSpiPs *spi_ps;
    volatile unsigned long *slcr_spi_base;     //SPI_CLK_CTRL register base address
    uint32_t slcr_spi_clk_bit;  //clock active bit in SPI_CLK_CTRL register
    uint8_t  slcr_spi_divisor;
    volatile unsigned long *aper_clk_base;     //APER_CLK_CTRL register base address
    uint32_t aper_spi_clk_bit;  //AMBA Clock control bit in APER_CLK_CTRL register
    uint32_t curr_freq;         //current spi baud frequency
    xSemaphoreHandle bus_mutex;
    struct list_head node_list_head; /*head of spi node list*/
};

static XSpiPs spi0_instance;			/* The instance of the SPI device */
static XSpiPs spi1_instance;			/* The instance of the SPI device */

static struct spi_describe spi0_desc = {
    .bus_id = 0,
    .spi_ps = &spi0_instance,
    .slcr_spi_base = SPI_CLK_CTRL,
    .slcr_spi_clk_bit = 0,
    .slcr_spi_divisor = SPI_CLOCK_SOURCE_DIVISOR,
    .aper_clk_base = APER_CLK_CTRL,
    .aper_spi_clk_bit = 14,
    .curr_freq = 0,
};

static struct spi_describe spi1_desc = {
    .bus_id = 1,
    .spi_ps = &spi1_instance,
    .slcr_spi_base = SPI_CLK_CTRL,
    .slcr_spi_clk_bit = 1,
    .slcr_spi_divisor = SPI_CLOCK_SOURCE_DIVISOR,
    .aper_clk_base = APER_CLK_CTRL,
    .aper_spi_clk_bit = 15,
    .curr_freq = 0,
};

static struct spi_describe *spi_descs[ZYNQ_MAX_SPI_BUS_NUM] = {&spi0_desc, &spi1_desc};

static inline int32_t spi_drv_clock_init(struct spi_describe *spi_desc)
{
	uint32_t val;

    if(spi_desc->slcr_spi_divisor > 0x3f)
    {
        return -1;
    }

    val = *spi_desc->slcr_spi_base;

    //enable spi clock 
    val |= (1 << spi_desc->slcr_spi_clk_bit);
    //set divisor
    val |= (spi_desc->slcr_spi_divisor << 8);

    *spi_desc->slcr_spi_base = val;
    
    //enable amba clock
    val = *spi_desc->aper_clk_base;
    val |= (1 << spi_desc->aper_spi_clk_bit);
    *spi_desc->aper_clk_base = val;

    return 0;

}

/*
 * this api must be called first
 */
int32_t spi_drv_init(uint8_t bus_id)
{
    struct spi_describe *spi_desc = NULL;
	int32_t status;
	XSpiPs_Config *spi_config;

    if(bus_id >= ZYNQ_MAX_SPI_BUS_NUM)
    {
        pilot_err("invalid spi bus:%d\n", bus_id);
        return -1;
    }

    spi_desc = spi_descs[bus_id];

    spi_desc->bus_mutex  = xSemaphoreCreateMutex();

    /*
     * init spi clock
     */
    spi_drv_clock_init(spi_desc);

    /*
	 * Initialize the SPI driver so that it's ready to use
	 */
	spi_config = XSpiPs_LookupConfig(bus_id);
	if (NULL == spi_config) {
		pilot_err(" SPI lookup configuration failed.\r\n");
		return XST_FAILURE;
	}

    status = XSpiPs_CfgInitialize(spi_desc->spi_ps, spi_config, spi_config->BaseAddress);
	if (status != XST_SUCCESS) {
		pilot_err(" SPI configuration initialization failed.\r\n");
		return XST_FAILURE;
	}

	/*
	 * Perform a self-test to check hardware build
	 */
	status = XSpiPs_SelfTest(spi_desc->spi_ps);
	if (status != XST_SUCCESS) {
		pilot_err(" SPI self test failed.\r\n");
		return XST_FAILURE;
	}

    XSpiPs_SetOptions(spi_desc->spi_ps, /*XSPIPS_MANUAL_START_OPTION |*/ \
			XSPIPS_MASTER_OPTION | XSPIPS_FORCE_SSELECT_OPTION | XSPIPS_CLK_PHASE_1_OPTION | XSPIPS_CLK_ACTIVE_LOW_OPTION);

	XSpiPs_Enable(spi_desc->spi_ps);

    INIT_LIST_HEAD(&spi_desc->node_list_head);

	return XST_SUCCESS;

}

/*
 * register a spi slave to spi bus
 */
void spi_register_node(struct spi_node *new_node)
{
	struct spi_describe *desc = NULL;

    if(new_node == NULL || new_node->bus_id >= ZYNQ_MAX_SPI_BUS_NUM)
    {
        pilot_err("no such spi bus:%d\n", new_node->bus_id);
        return -1;
    }

    desc = spi_descs[new_node->bus_id];

    xSemaphoreTake(desc->bus_mutex, portMAX_DELAY);

	list_add_tail(&new_node->spi_list, &desc->node_list_head);
    
    xSemaphoreGive(desc->bus_mutex);
}

/*
 * delete a registered spi node
 */
void spi_deregister_node(struct spi_node *node)
{
	struct spi_describe *desc = NULL;

    if(node == NULL || node->bus_id >= ZYNQ_MAX_SPI_BUS_NUM)
    {
        pilot_err("no such spi bus:%d\n", node->bus_id);
        return -1;
    }

    desc = spi_descs[node->bus_id];


    xSemaphoreTake(desc->bus_mutex, portMAX_DELAY);

	list_del(&node->spi_list);

    xSemaphoreGive(desc->bus_mutex);
}

/*
 * init chip select gpio
 */
void spi_cs_init(struct spi_node *node)
{
	gpio_pin_config_output(node->cs_pin);
}

static inline void spi_cs_write(struct spi_node *node, bool selected)
{
    if(selected == true)
        gpio_clear(node->cs_pin);
    else
        gpio_set(node->cs_pin);
}

static void spi_select(struct spi_describe *desc, struct spi_node *node, bool selected)
{
	struct spi_node * tmp_node = NULL;

	list_for_each_entry(tmp_node, &desc->node_list_head, spi_list)
    {
        if(tmp_node == node)
        {
            spi_cs_write(node, selected);
        }
        else
        {
            spi_cs_write(tmp_node, false);
        }
    }
}

static inline uint8_t spi_xfer(XSpiPs *spi_ps, uint8_t data)
{
    uint32_t reg = 0;

    XSpiPs_WriteReg(spi_ps->Config.BaseAddress, XSPIPS_TXD_OFFSET, data);

    /*等待tx fifo空*/
    reg = XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET);
    while((reg & XSPIPS_IXR_TXOW_MASK) == 0)
    {
        reg = XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET);
    }


    reg = XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET);
    while((reg & XSPIPS_IXR_RXNEMPTY_MASK) == 0)
    {
        reg = XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET);
    }

    data = (uint8_t)XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_RXD_OFFSET);

    /*清空标志位*/
    reg = XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET);
    XSpiPs_WriteReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET, reg);
    reg = XSpiPs_ReadReg(spi_ps->Config.BaseAddress, XSPIPS_SR_OFFSET);

    return data;
}

static inline spi_set_prescaler(struct spi_describe *desc, uint32_t frequency)
{
    uint8_t baud_divisor = SPI_REFERENCE_CLOCK / frequency;
    int32_t ret = -1;

    baud_divisor /= 4; //convert to XSPIPS_CLK_PRESCALE_* in xspips.h
    //limit baud divisor with max/min value
    if(baud_divisor < XSPIPS_CLK_PRESCALE_4)
        baud_divisor = XSPIPS_CLK_PRESCALE_4;
    if(baud_divisor > XSPIPS_CLK_PRESCALE_256)
        baud_divisor = XSPIPS_CLK_PRESCALE_256;

    ret = XSpiPs_SetClkPrescaler(desc->spi_ps, baud_divisor);
    if(ret == XST_SUCCESS)
    {
        desc->curr_freq = frequency;
    }

    return ret;
}

/*
 * Exchange a block of data on SPI without using DMA
 */
int32_t spi_transfer(struct spi_node *node, uint8_t *send_buf, uint8_t *recv_buf, int16_t len)
{
	struct spi_describe *desc = NULL;
    uint8_t *src = send_buf;
    uint8_t *dest = recv_buf;
    int32_t ret = -1;

    if(send_buf == NULL && recv_buf == NULL)
        return -1;

    if(node == NULL || node->bus_id >= ZYNQ_MAX_SPI_BUS_NUM)
    {
        pilot_err("no such spi bus or node\n");
        return -1;
    }

    desc = spi_descs[node->bus_id];

    xSemaphoreTake(desc->bus_mutex, portMAX_DELAY);
    if(desc->curr_freq != node->frequency)
    {
        ret = spi_set_prescaler(desc, node->frequency);
        if (ret != XST_SUCCESS) 
        {
            pilot_err("SPI set frequency %d Failed %d\r\n", node->frequency, ret);
            xSemaphoreGive(desc->bus_mutex);
            return -1;
        }
    }

    spi_select(desc, node, true);

	ret = XSpiPs_PolledTransfer(desc->spi_ps, src, dest, len);
    if(ret != XST_SUCCESS)
        ret = -1;

    spi_select(desc, node, false);

    xSemaphoreGive(desc->bus_mutex);

    return ret;
}

int32_t spi_set_frequency(struct spi_node *node, uint32_t frequency)
{
	struct spi_describe *desc = NULL;
    
    if(node == NULL || node->bus_id >= ZYNQ_MAX_SPI_BUS_NUM)
    {
        pilot_err("no such spi bus:%d\n", node->bus_id);
        return -1;
    }

    desc = spi_descs[node->bus_id];

    xSemaphoreTake(desc->bus_mutex, portMAX_DELAY);

    node->frequency = frequency;

    xSemaphoreGive(desc->bus_mutex);

    return 0;
}

