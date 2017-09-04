UORB_PATH += $(MODULES_ROOT_PATH)/uORB/
SRCS_UORB += $(shell find $(UORB_PATH) -name "*.cpp")

OCM_PATH += $(MODULES_ROOT_PATH)/ocm/
SRCS_OCM += $(OCM_PATH)/ocm.c

PERF_PATH += $(MODULES_ROOT_PATH)/perf/
SRCS_PERF += $(PERF_PATH)/perf_counter.c

OBJS_MODULES += $(SRCS_UORB:%.cpp=%.o)
OBJS_MODULES += $(SRCS_OCM:%.c=%.o)
OBJS_MODULES += $(SRCS_PERF:%.c=%.o)
			


