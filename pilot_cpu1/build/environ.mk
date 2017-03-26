#set your project name
PRJ_NAME = freeRTOS.elf

#set your project type : choose one below
CC = arm-xilinx-eabi-gcc
CXX = arm-xilinx-eabi-g++
#CC = 
#CC = ar -r 

#set your output path
Output:= ../bin/

#set your source folder
SRC_PATH:= ../src
DRV_ROOT_PATH:= $(SRC_PATH)/driver
TEST_ROOT_PATH:= $(SRC_PATH)/test
MODULES_ROOT_PATH:=$(SRC_PATH)/modules
LIBRARIES_ROOT_PATH:=$(SRC_PATH)/libraries
COPTER_ROOT_PATH:=$(SRC_PATH)/copter

#set Debug or Release
Compile_Flag = Debug
#Compile_Flag = Release

#add the lib you used here
#LIBS := -lLib1 -lLib2 -lLib3
LIBS := -Wl,--start-group,-lxil,-lgcc,-lm,-lc,-lstdc++,--end-group
#LIBPATH := -Lpath1 -Lpath2 -Lpath3
LIBPATH := -L../libs
INCLUDEPATH := -I../src/FreeRTOS \
			   -I../src/FreeRTOS/FreeRTOS_Source/include/ \
			   -I../src/FreeRTOS/FreeRTOS_Source/portable/GCC/ARM_CA9 \
			   -I../include/standalone \
			   -I../include/print \
			   -I../include/drivers\
			   -I../include/common \
			   -I../include \
			   -I../src \
			   -I../src/driver \
			   -I../src/driver/cpu_peripheral \
			   -I../src/modules \
			   -I../include/config \
			   -I../src/driver/common \
			   -I../src/libraries \
			   -I../src/libraries/AP_HAL \
			   -I../src/libraries/AP_HAL_ZYNQ \
			   -I../include/GCS_MAVLink
# INCLUDEPATH := -I/usr/lib/XXX/include

LSCRIPT := ../src/lscript.ld

#flags in makefile
DEBUG_FLAG = -O2 -g3 -Wall -c -fmessage-length=0 
RELEASE_FLAG = -O2 -Wall -c -fmessage-length=0
RM := rm -rf

#list all dirs
SUBDIRS := $(shell find $(SRC_PATH) -type d)

#set compile flag
ifeq ($(Compile_Flag),Debug)
CFLAGS := $(DEBUG_FLAG)
else
CFLAGS := $(RELEASE_FLAG)
endif

CFLAGS += -D__PX4_FREERTOS
CXXFLAGS += 
