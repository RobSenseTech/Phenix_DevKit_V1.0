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

void gic_bind_interrupt_to_cpu(int32_t intr_id, int32_t cpu_id);
int32_t gic_isr_register(int32_t intr_id, Xil_InterruptHandler Handler, void *CallBackRef);
void gic_interrupt_enable(int32_t intr_id);
void gic_set_trigger_type(uint32_t intr_id, uint8_t Trigger);
void gic_init();
#endif 

