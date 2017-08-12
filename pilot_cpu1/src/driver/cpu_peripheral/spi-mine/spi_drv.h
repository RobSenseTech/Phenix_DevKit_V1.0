/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    spi_drv.h
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-08-12 14:04:02
 *Last Modify: 2017-08-12 14:04:02
 *Description: 
**********************************************************************************/

#pragma once
#include <stdlib.h>
#include <string.h>

#include "xil_exception.h"
#include "pilot_print.h"
#include "xstatus.h"
#include "xparameters_ps.h"
#include "xparameters.h"
#include "xiicps.h"
#include "xiicps_hw.h"
#include "board_config.h"
#include "FreeRTOS.h"
#include "semphr.h"



/*
 *          spi_drv_init
 *               |
 *          spi_cs_init
 *               |
 *       spi_register_node
 *               |
 *          spi_transfer
 */

struct spi_node
{
    /*internal variable, don't use'*/
    struct list_head spi_list;

    uint32_t port_id; 
    uint32_t cs_pin;
};

/*
 * this api must be called first
 */
void spi_drv_init(uint32_t port_id);

/*
 * register a spi slave to spi bus
 */
void spi_register_node(struct spi_node *new_node);

/*
 * delete a registered spi node
 */
void spi_deregister_node(struct spi_node *node);

/*
 * init chip select gpio
 * this api must called after spi_drv_init & spi_register_node
 */
void spi_cs_init(struct spi_node *node);

/*
 * Exchange a block of data on SPI without using DMA
 */
int32_t spi_transfer(struct spi_node *node, uint8_t *send_buf, uint8_t *recv_buf, int16_t len);

