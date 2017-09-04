
#新驱动直接把路径加在下面即可
#DRIVER_PATH += $(DRV_ROOT_PATH)/mpu6000
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/uart
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/uartns550
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/hrt
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/gic
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/sd
DRIVER_PATH += $(DRV_ROOT_PATH)/device
DRIVER_PATH += $(DRV_ROOT_PATH)/rc
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/spi
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/gpio
DRIVER_PATH += $(DRV_ROOT_PATH)/cpu_peripheral/iic
DRIVER_PATH += $(DRV_ROOT_PATH)/common/conversion
DRIVER_PATH += $(DRV_ROOT_PATH)/i3g4250d
DRIVER_PATH += $(DRV_ROOT_PATH)/iis328dq
DRIVER_PATH += $(DRV_ROOT_PATH)/lis3mdl
DRIVER_PATH += $(DRV_ROOT_PATH)/hmc5883
DRIVER_PATH += $(DRV_ROOT_PATH)/rgbled
DRIVER_PATH += $(DRV_ROOT_PATH)/ms5611
DRIVER_PATH += $(DRV_ROOT_PATH)/mpu6500
DRIVER_PATH += $(DRV_ROOT_PATH)/l3gd20
DRIVER_PATH += $(DRV_ROOT_PATH)/fmu
DRIVER_PATH += $(DRV_ROOT_PATH)/pwm
DRIVER_PATH += $(DRV_ROOT_PATH)/px4flow
DRIVER_PATH += $(DRV_ROOT_PATH)/perf_cmd
DRIVER_PATH += $(DRV_ROOT_PATH)/common/drv_unistd
DRIVER_PATH += $(DRV_ROOT_PATH)/common/systemlib/mixer
DRIVER_PATH += $(DRV_ROOT_PATH)/common/systemlib/pwm_limit
DRIVER_PATH += $(DRV_ROOT_PATH)/common/systemlib/airspeed
DRIVER_PATH += $(DRV_ROOT_PATH)/common/systemlib/conversions
DRIVER_PATH += $(DRV_ROOT_PATH)/common/systemlib/geo


SRCS_DRV += $(shell find $(DRIVER_PATH) -name "*.c")
SRCS_DRV += $(shell find $(DRIVER_PATH) -name "*.cpp")

OBJS_TMP_DRV += $(patsubst %.c, %.o, $(SRCS_DRV)) 
OBJS_DRV += $(patsubst %.cpp, %.o, $(OBJS_TMP_DRV)) 



