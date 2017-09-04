#set your project name
PRJ_NAME = ocmfs_master

#set your project type : choose one below
CC = arm-linux-gnueabihf-gcc  	#arm-xilinx-linux-gnueabi-gcc
#CC = 
#CC = ar -r 

#set Debug or Release
Compile_Flag = Debug
#Compile_Flag = Release

#set your output path
Output:= ./

#set your source folder
SRC_PATH:= ./
MODULES_PATH:= ./../../pilot_cpu1/src/modules/
OCM_MOD:= $(MODULES_PATH)ocm/

#add the lib you used here
#LIBS := -lLib1 -lLib2 -lLib3
LIBS := #-lpthread 
#LIBPATH := -Lpath1 -Lpath2 -Lpath3
LIBPATH := 
INCLUDEPATH := -I$(MODULES_PATH)\
			   -I./../../pilot_cpu1/include/print/\
			   -I./../../pilot_cpu1/include/common/\
			   -I./../../pilot_cpu1/src/FreeRTOS/robsense_custom/fs/ocmfs/
# INCLUDEPATH := -I/usr/lib/XXX/include

###################################
#DON"T MODIFY THE BELOWS

#list all dirs
SUBDIRS := $(shell find $(SRC_PATH) -type d)
#flags in makefile
DEBUG_FLAG = -g3 -O0 -Wall -c -fmessage-length=0 -D_GNU_SOURCE #wipe off warning of asprintf
RELEASE_FLAG =   -O0 -Wall -c -fmessage-length=0 -D_GNU_SOURCE
RM := rm -rf

#set compile flag
ifeq ($(Compile_Flag),Debug)
CFLAGS := $(DEBUG_FLAG) $(INCLUDEPATH) $(LIBS)
else
CFLAGS := $(RELEASE_FLAG)
endif

#prepare files
#SRCS := $(shell find $(SRC_PATH) -name *.c)
vpath %.c $(OCM_MOD)
SRCS = $(wildcard *.c) $(notdir $(wildcard $(OCM_MOD)*.c))  
OBJS:=$(SRCS:%.c=./%.o)

#all target
all: $(Output)$(PRJ_NAME)

#tool invocations
$(Output)$(PRJ_NAME):$(OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking:$(CC)'
	@echo 'src:$(SRCS)'
	$(CC) -static -o "$@" $(OBJS) $(LIBS)
	cp $(Output)$(PRJ_NAME) ../filesystem/rootfs/usr/
	../filesystem/mkramfs.sh ../filesystem/rootfs/
	cp  uramdisk.image.gz ../../amp_system/linux_image/
	mv  uramdisk.image.gz ../filesystem
	@echo 'Finished building target: $@'
	@echo ' '

#other targets
clean:
	find $(SRC_PATH) -name "*.d" -exec rm -rf {} \;
	find $(SRC_PATH) -name "*.o" -exec rm -rf {} \;
	rm $(Output)$(PRJ_NAME) 

.PHONY:all clean
.SECONDARY:


