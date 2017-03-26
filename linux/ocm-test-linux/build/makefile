#set your project name
PRJ_NAME = ocm_test-linux.elf

#set your project type : choose one below
CC = arm-xilinx-linux-gnueabi-gcc
#CC = 
#CC = ar -r 

#set Debug or Release
Compile_Flag = Debug
#Compile_Flag = Release

#set your output path
Output:= ../bin/

#set your source folder
SRC_PATH:= ../src

#add the lib you used here
#LIBS := -lLib1 -lLib2 -lLib3
LIBS := 
#LIBPATH := -Lpath1 -Lpath2 -Lpath3
LIBPATH := 
INCLUDEPATH :=
# INCLUDEPATH := -I/usr/lib/XXX/include

LSCRIPT :=
###################################
#DON"T MODIFY THE BELOWS

#list all dirs
SUBDIRS := $(shell find $(SRC_PATH) -type d)
#flags in makefile
DEBUG_FLAG = -O0 -g3 -Wall -c -fmessage-length=0
RELEASE_FLAG = -O3 -Wall -c -fmessage-length=0
RM := rm -rf

#set compile flag
ifeq ($(Compile_Flag),Debug)
CFLAGS := $(DEBUG_FLAG)
else
CFLAGS := $(RELEASE_FLAG)
endif

#prepare files
SRCS:=$(shell find $(SRC_PATH) -name *.c)
SRCS_S:=$(shell find $(SRC_PATH) -name *.S)
OBJS:=$(SRCS:%.c=%.o)
OBJS_S:=$(SRCS_S:%.S=%.o)

#all target
all: $(Output)$(PRJ_NAME)

#tool invocations
$(Output)$(PRJ_NAME):$(OBJS) $(OBJS_S) $(LSCRIPT)
	@echo 'Building target: $@'
	@echo 'Invoking:$(CC)'
	$(CC) -o "$@" $(OBJS) $(OBJS_S) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

%o:%S
	@echo 'Building file: $<'
	@echo 'Invoking:$(CC)'
	$(CC) $(CFLAGS) $(INCLUDEPATH) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

%o:%c
	@echo 'Building file: $<'
	@echo 'Invoking:$(CC)'
	$(CC) $(CFLAGS) $(INCLUDEPATH) -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o"$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

#other targets
clean:
	find $(SRC_PATH) -name "*.d" -exec rm -rf {} \;
	find $(SRC_PATH) -name "*.o" -exec rm -rf {} \;
	-$(RM) $(Output)/*

.PHONY:all clean
.SECONDARY:

