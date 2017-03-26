
-include environ.mk
-include ../src/FreeRTOS/Makefile
-include $(DRV_ROOT_PATH)/Makefile
-include $(TEST_ROOT_PATH)/Makefile
-include $(MODULES_ROOT_PATH)/Makefile
-include $(LIBRARIES_ROOT_PATH)/Makefile
-include $(COPTER_ROOT_PATH)/Makefile

SRCS += ../src/main.c
SRCS += ../src/sensor.c

OBJS += $(SRCS:%.c=%.o)
OBJS_ALL += $(OBJS) $(OBJS_S) $(OBJS_DRV) $(OBJS_RTOS) $(OBJS_TEST) $(OBJS_MODULES) $(OBJS_LIBRARIES) $(OBJS_COPTER) 
	


#all target
all: $(Output)$(PRJ_NAME)

#tool invocations
$(Output)$(PRJ_NAME):$(OBJS_ALL)
	@echo 'Building target: $@'
	@echo 'Invoking:$(CXX)'
	$(CXX) -Wl,-T -Wl,$(LSCRIPT) $(LIBPATH) -o $@ $^ $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

%.o:%.S
	@echo 'SRCS_S=$(SRCS_S)'
	@echo 'Building .S file: $<'
	@echo 'Invoking:$(CC)'
	$(CC) $(CFLAGS) $(INCLUDEPATH) -c -MMD -o $@ $< 
	@echo 'Finished building: $<'
	@echo ' '

%.o:%.c
	@echo 'Building C file: $<'
	@echo 'Invoking:$(CC)'
	$(CC) $(CFLAGS) $(INCLUDEPATH) -c -MMD -o $@ $< 
	@echo 'Finished building: $<'
	@echo ' '

%.o:%.cpp
	@echo 'Building C++ file: $<'
	@echo 'Invoking:$(CXX)'
	$(CXX) -std=gnu++11 $(CFLAGS) $(INCLUDEPATH) -c -MMD -o $@ $< 
	@echo 'Finished building: $<'
	@echo ' '

-include $(OBJS_ALL:.o=.d)

#other targets
clean:
	find $(SRC_PATH) -name "*.d" -exec rm -rf {} \;
	find $(SRC_PATH) -name "*.o" -exec rm -rf {} \;
	-$(RM) $(Output)*

.PHONY:all clean
.SECONDARY:

