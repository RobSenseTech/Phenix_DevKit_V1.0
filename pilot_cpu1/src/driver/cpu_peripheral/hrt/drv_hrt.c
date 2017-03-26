/*
 * 高精度定时器，移植自px4，基于zynq ttc0的counter0
 * 用于高速外设驱动的线程调度和高精度时间维护
 */
#include "xparameters.h"
#include "xstatus.h"
#include "xttcps.h"
#include "FreeRTOS_Print.h"
#include "driver.h"

/**
 * Minimum/maximum deadlines.
 *
 * These are suitable for use with a 16-bit timer/counter clocked
 * at 1MHz.  The high-resolution timer need only guarantee that it
 * not wrap more than once in the 50ms period for absolute time to
 * be consistently maintained.
 *
 * The minimum deadline must be such that the time taken between
 * reading a time and writing a deadline to the timer cannot
 * result in missing the deadline.
 */
#define HRT_INTERVAL_MIN	50
#define HRT_INTERVAL_MAX	50000

/*
 * Period of the free-running counter, in microseconds.
 */
#define HRT_COUNTER_PERIOD 65536

/* 将计数器值转换成微妙 目前是乘以7, 由于要在中断调用，用移位运算，速度快*/
#define HRT_COUNTER_SCALE(_c)	(_c)	//(((_c) << 3) + _c)

static XTtcPs xTtcPsInst;	//One timer counter
static struct list_head callout_queue;
static uint16_t			latency_baseline;

/* timer count at interrupt (for latency purposes) */
static uint16_t			latency_actual;

/* latency histogram */
#define LATENCY_BUCKET_COUNT 8
__EXPORT const uint16_t latency_bucket_count = LATENCY_BUCKET_COUNT;
__EXPORT const uint16_t	latency_buckets[LATENCY_BUCKET_COUNT] = { 1, 2, 5, 10, 20, 50, 100, 1000 };
__EXPORT uint32_t		latency_counters[LATENCY_BUCKET_COUNT + 1];


/* timer-specific functions */
static void		hrt_tim_isr(void *CallBackRef);
static void		hrt_latency_update(void);

/* callout list manipulation */
static void		hrt_call_internal(struct hrt_call *entry,
		hrt_abstime deadline,
		hrt_abstime interval,
		hrt_callout callout,
		void *arg);
static void		hrt_call_enter(struct hrt_call *entry);
static void		hrt_call_reschedule(void);
static void		hrt_call_invoke(void);

static void hrt_tim_isr(void *CallBackRef)
{
	int32_t iStatusEvent;
	XTtcPs *xTimer = &xTtcPsInst;
#ifdef HRT_DEBUG
	unsigned long hb=0;
	uint32_t CounterValue = 0;
	static uint32_t lastCount = 0;
	static x = 0;
#endif

	iStatusEvent = XTtcPs_GetInterruptStatus(xTimer);
	XTtcPs_ClearInterruptStatus(xTimer, iStatusEvent);

	if((XTTCPS_IXR_MATCH_0_MASK & iStatusEvent) != 0)
	{
	#ifdef HRT_DEBUG
		x++;
		CounterValue = XTtcPs_GetCounterValue(xTimer);
		if(lastCount - CounterValue >= 1000)
		{
			hb = (CounterValue+1000) & 0xffff;	
			if(x >= 1000)
			{
				Print_Info("CounterValue=%d lastCount=%d next match:%d x=%d\n", CounterValue, lastCount, hb, x);
				x = 0;
			}
			XTtcPs_SetMatchValue(xTimer, 0, hb);//每1us,定时器counter加7,默认每1ms中断一次
			lastCount = CounterValue;
		}
	#endif
		/* do latency calculations */
		hrt_latency_update();

		/* run any callouts that have met their deadline */
		hrt_call_invoke();

		/* and schedule the next interrupt */
		hrt_call_reschedule();
	}

//	Print_Info("ttc test iStatusEvent:%x \n",iStatusEvent);
}

int hrt_init()
{
	int iStatus = 0;
	XTtcPs_Config *pxConfig;
	XTtcPs *xTimer = &xTtcPsInst;

	//初始化驱动队列
	INIT_LIST_HEAD(&callout_queue);	

	pxConfig = XTtcPs_LookupConfig(XPAR_XTTCPS_0_DEVICE_ID);
	if(pxConfig == NULL)
	{
		Print_Err("There is no ttc config\n");
		return XST_FAILURE;
	}

	iStatus = XTtcPs_CfgInitialize(xTimer, pxConfig, pxConfig->BaseAddress);
	if(iStatus != XST_SUCCESS)
	{
		Print_Err("Init ttcps cfg failed:%d\n", iStatus);
		return XST_FAILURE;
	}

	/*设置定时器工作模式*/
	XTtcPs_SetOptions(xTimer, XTTCPS_OPTION_EXTERNAL_CLK | XTTCPS_OPTION_MATCH_MODE | XTTCPS_OPTION_WAVE_DISABLE);

	//定时器输入时钟由PL提供8Mhz，分频8倍转成1Mhz
	XTtcPs_SetPrescaler(xTimer, 2);

	/*设置match寄存器的值*/
	XTtcPs_SetMatchValue(xTimer, 0, 1000);//每1us,定时器counter加1,初始化成每1ms中断一次

	/*设置定时器中断目标为cpu1*/
	GicBindInterruptToCpu(XPAR_XTTCPS_0_INTR, CPU_1_TARGETED);

	/*注册中断处理函数*/
	iStatus = GicIsrHandlerRegister(XPAR_XTTCPS_0_INTR, (Xil_ExceptionHandler)hrt_tim_isr, (void *)xTimer);
	if(iStatus != XST_SUCCESS)
	{
		Print_Err("Set ttc interrupt handler failed:%d\n", iStatus);
		return XST_FAILURE;
	}
	
	GicInterruptEnable(XPAR_XTTCPS_0_INTR);

	/*使能match模式中断*/
	XTtcPs_EnableInterrupts(xTimer, XTTCPS_IXR_MATCH_0_MASK);

	/*启动定时器*/
	XTtcPs_Start(xTimer);

	return XST_SUCCESS;
}

/*
 * 以微妙为单位，对时间的累加，实际加的就是计数器的值，由于计数器是1/6.9M的时间计数一次，所以相当计数器于每1us加7
 */
hrt_abstime hrt_absolute_time(void)
{
	hrt_abstime abstime;
	uint32_t count;
	irqstate_t flags;
	
	static volatile hrt_abstime base_time;
	static volatile uint32_t last_count;

	/* 保存并关中断，防止重入*/
	flags = irqsave();

	count = XTtcPs_GetCounterValue(&xTtcPsInst);

	/*
	 * 判断计数器是否已经溢出过
	 */
	if(count < last_count)
	{
		base_time += HRT_COUNTER_PERIOD;
	}

	last_count = count;

	abstime = HRT_COUNTER_SCALE(base_time + count);

	irqrestore(flags);

	return abstime;
}

/*
 * Compare i time value with the current time.
 */
hrt_abstime hrt_elapsed_time(const volatile hrt_abstime *then)
{
	irqstate_t flags = irqsave();

	hrt_abstime delta = hrt_absolute_time() - *then;

	irqrestore(flags);

	return delta;
}

/*
 * Store the absolute time in an interrupt-safe fashion
 */
hrt_abstime hrt_store_absolute_time(volatile hrt_abstime *now)
{
	irqstate_t flags = irqsave();

	hrt_abstime ts = hrt_absolute_time();

	irqrestore(flags);

	return ts;
}

/*
 *  Call callout(arg) after interval has elapsed.
 */
void hrt_call_after(struct hrt_call *entry, hrt_abstime delay, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, hrt_absolute_time()+delay, 0, callout, arg);
}

/*
 * Call callout(arg) at calltime.
 */
void hrt_call_at(struct hrt_call *entry, hrt_abstime calltime, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry, calltime, 0, callout, arg);
}

/**
 * Call callout(arg) every period.
 */
void hrt_call_every(struct hrt_call *entry, hrt_abstime delay, hrt_abstime interval, hrt_callout callout, void *arg)
{
	hrt_call_internal(entry,
			  hrt_absolute_time() + delay,
			  interval,
			  callout,
			  arg);
}

static void hrt_call_internal(struct hrt_call *entry, hrt_abstime deadline, hrt_abstime interval, hrt_callout callout, void *arg)
{
	irqstate_t flags = irqsave();

	/* if the entry is currently queued, remove it */
	/* note that we are using a potentially uninitialised
	   entry->link here, but it is safe as sq_rem() doesn't
	   dereference the passed node unless it is found in the
	   list. So we potentially waste a bit of time searching the
	   queue for the uninitialised entry->link but we don't do
	   anything actually unsafe.
	*/
	if (entry->deadline != 0) {
		list_del(&entry->link);
	}

	entry->deadline = deadline;
	entry->period = interval;
	entry->callout = callout;
	entry->arg = arg;

	hrt_call_enter(entry);

	irqrestore(flags);
}

/**
 * If this returns true, the call has been invoked and removed from the callout list.
 *
 * Always returns false for repeating callouts.
 */
bool
hrt_called(struct hrt_call *entry)
{
	return (entry->deadline == 0);
}

/**
 * Remove the entry from the callout list.
 */
void
hrt_cancel(struct hrt_call *entry)
{
	irqstate_t flags = irqsave();

	list_del(&entry->link);
	entry->deadline = 0;

	/* if this is a periodic call being removed by the callout, prevent it from
	 * being re-entered when the callout returns.
	 */
	entry->period = 0;

	irqrestore(flags);
}


static void hrt_call_enter(struct hrt_call *entry)
{
	struct hrt_call	*call;

	call = list_first_entry_or_null(&callout_queue, struct hrt_call,link);

	/*按deadline大小，在callout队列中插入当前注册的节点（包含回调函数），若是空队列，或者deadline比头节点的deadline跟小，则，直接修改
	  下一个定时器中断到来的时间*/
	if ((call == NULL) || (entry->deadline < call->deadline)) {
		list_add(&entry->link, &callout_queue);
		//lldbg("call enter at head, reschedule\n");
		/* we changed the next deadline, reschedule the timer event */
		hrt_call_reschedule();

	} 
	else 
	{
		/*轮训队列，直到找到比要插入节点的deadline更大的节点，然后插入到它前面去*/
		list_for_each_entry(call, &callout_queue, link)
		{
			if(entry->deadline < call->deadline)
			{
				list_add_after(&entry->link, &call->link, &callout_queue);
				break;
			}	
		}
		//轮训到最后也没找到就加到队尾
		if(&call->link == &callout_queue)
		{
			list_add_tail(&entry->link, &callout_queue);
		}
	}

	//lldbg("scheduled\n");
}

//调用注册的回调函数
static void hrt_call_invoke(void)
{
	struct hrt_call	*call;
	hrt_abstime deadline;

	while (true) {
		/* get the current time */
		hrt_abstime now = hrt_absolute_time();

		call = list_first_entry_or_null(&callout_queue, typeof(*call),link);

		if (call == NULL) {
			break;
		}

		if (call->deadline > now) {
			break;
		}

		list_del(&call->link);
		//lldbg("call pop\n");

		/* save the intended deadline for periodic calls */
		deadline = call->deadline;

		/* zero the deadline, as the call has occurred */
		call->deadline = 0;

		/* invoke the callout (if there is one) */
		if (call->callout) {
			//lldbg("call %p: %p(%p)\n", call, call->callout, call->arg);
			call->callout(call->arg);
		}

		/* if the callout has a non-zero period, it has to be re-entered */
		if (call->period != 0) {
			// re-check call->deadline to allow for
			// callouts to re-schedule themselves
			// using hrt_call_delay()
			if (call->deadline <= now) {
				call->deadline = deadline + call->period;
			}

			hrt_call_enter(call);
		}
	}
}

/**
 * Reschedule the next timer interrupt.
 *
 * This routine must be called with interrupts disabled.
 */
static void hrt_call_reschedule()
{
	hrt_abstime now = hrt_absolute_time();
	struct hrt_call *head = list_first_entry_or_null(&callout_queue, typeof(*head),link);
	hrt_abstime deadline = now + HRT_INTERVAL_MAX;

	/*
	 * Determine what the next deadline will be.
	 *
	 * Note that we ensure that this will be within the counter
	 * period, so that when we truncate all but the low 16 bits
	 * the next time the compare matches it will be the deadline
	 * we want.
	 *
	 * It is important for accurate timekeeping that the compare
	 * interrupt fires sufficiently often that the base_time update in
	 * hrt_absolute_time runs at least once per timer period.
	 */
	if (head != NULL) {
		//lldbg("entry in queue\n");
		if (head->deadline <= (now + HRT_INTERVAL_MIN)) {
			//lldbg("pre-expired\n");
			/* set a minimal deadline so that we call ASAP */
			deadline = now + HRT_INTERVAL_MIN;

		} else if (head->deadline < deadline) {
			//lldbg("due soon\n");
			deadline = head->deadline;
		}
	}

	/* set the new compare value and remember it for latency tracking */
	latency_baseline = HRT_COUNTER_SCALE(deadline) & 0xffff;
	XTtcPs_SetMatchValue(&xTtcPsInst, 0, HRT_COUNTER_SCALE(deadline) & 0xffff);
}

static void hrt_latency_update(void)
{
	uint16_t latency = latency_actual - latency_baseline;
	unsigned	index;

	/* bounded buckets */
	for (index = 0; index < LATENCY_BUCKET_COUNT; index++) {
		if (latency <= latency_buckets[index]) {
			latency_counters[index]++;
			return;
		}
	}

	/* catch-all at the end */
	latency_counters[index]++;
}

void
hrt_call_init(struct hrt_call *entry)
{
	memset(entry, 0, sizeof(*entry));
}

void
hrt_call_delay(struct hrt_call *entry, hrt_abstime delay)
{
	entry->deadline = hrt_absolute_time() + delay;
}





