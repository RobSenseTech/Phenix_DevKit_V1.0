/*********************************************************************************
 *Copyright(C),2015-2017, Robsense Tech. All rights reserved.
 *FileName:    spi_drv.c
 *Author:      HeBin
 *Version:     0.1
 *Date:        2017-08-12 14:03:49
 *Last Modify: 2017-08-12 14:03:49
 *Description: 
**********************************************************************************/

#include "spi_drv.h"

struct spi_describe {
    uint8_t bus_id;
    XSpiPs *spi_ps;
    xSemaphoreHandle iic_mutex;
    struct list_head node_list_head; /*head of spi node list*/
};


void spi_drv_init(uint32_t port_id)
{

}

