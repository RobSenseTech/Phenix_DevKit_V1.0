#include "xparameters.h"
#include <stdio.h>
#include "xil_cache.h"
#include "driver.h"
#include "board_config.h"

#if 0
int OcmAdd()
{
//	COMM_TX_FLAG ++;
    memset(OCM_TX_MSG_START_ADDR, 0x11111111, sizeof(int));
	Xil_DCacheFlushRange(OCM_TX_MSG_START_ADDR, 4);//必须flush，不然真正的ocm中的值不会变
	return 0;
}
#endif
