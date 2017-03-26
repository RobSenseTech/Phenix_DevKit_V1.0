#ifndef _ZYNQ_GIC_
#define _ZYNQ_GIC_

#include "xscugic.h"

//指定CPU为中断target时对应的值
enum{
	CPU_NO_TARGETED = 0,
	CPU_0_TARGETED,
	CPU_1_TARGETED,
	CPU_BOTH_TARGETED,
};

enum{
	INTR_TRIGGER_HIGH_LEVEL = 1,
	INTR_TRIGGER_RAISING_EDGE = 3,
};

extern XScuGic xInterruptController;//整个系统公用一个InterruptController，不然会重复初始化导致中断错误

void GicBindInterruptToCpu(int32_t iIntrId, int32_t iCpuId);
int32_t GicIsrHandlerRegister(int32_t iIntrId, Xil_InterruptHandler Handler, void *CallBackRef);
void GicInterruptEnable(int32_t iIntrId);
void GicSetTriggerType(uint32_t iIntrId, uint8_t Trigger);
void GicInit();
#endif 

