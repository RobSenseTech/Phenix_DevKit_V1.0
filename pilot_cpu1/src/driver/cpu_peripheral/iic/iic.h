#ifndef __IIC_H
#define __IIC_H

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

#ifdef __cplusplus
extern "C" {
#endif
/************************** Function Prototypes ******************************/
int32_t iic_init(uint8_t bus_id, uint32_t clk);
void *iic_register(uint8_t bus_id, uint8_t dev_addr, uint32_t clk);
void iic_deregister(void *iic_priv);
int32_t iic_transfer(void *iic_priv, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len);

#ifdef __cplusplus
}
#endif

#endif
