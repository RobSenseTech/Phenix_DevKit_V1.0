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
#include "board_config.h"
#include "FreeRTOS.h"
#include "semphr.h"

/* I2C Device Private Data */

typedef struct{
    u8  bus_id;
	XIicPs* iicbus; 
    xSemaphoreHandle iic_mutex;
	u32 frequency;
	u8  devaddr;
	u8  regaddr;
}iic_priv_s;


#ifdef __cplusplus
extern "C" {
#endif
/************************** Function Prototypes ******************************/
int iic_init(u8 iIicId, u32 FsclHz);
iic_priv_s *iic_get_priv(u8 iIicId, u8 DevAddr, u32 FsclHz);
int iic_transfer(iic_priv_s *iic_priv, const uint8_t *send, unsigned send_len, uint8_t *recv, unsigned recv_len, unsigned regaddrflag);
void Iic_set_address(iic_priv_s *iic_priv, u8 reg_addr);

#ifdef __cplusplus
}
#endif

#endif
