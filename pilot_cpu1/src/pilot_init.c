/* Standard includes. */
#include <stdio.h>
#include <limits.h>
#include <string.h>

/* Scheduler include files. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

/*Custom includes*/
#include "FreeRTOS_Print.h"
#include "drv_accel.h"
#include "drv_gyro.h"
#include "driver.h"
#include "pilot_init.h"
#include "ocm_config.h"
#include  <fs/fs.h>
#include <sys/stat.h>


extern int mpu6000_main(int argc, char *argv[]);
extern int i3g4250d_main(int argc, char *argv[]);
extern int iis328dq_main(int argc, char *argv[]);
extern int fmu_main(int argc, char *argv[]);
extern int lis3mdl_main(int argc, char *argv[]);
extern int hmc5883_main(int argc, char *argv[]);
extern int ms5611_main(int argc, char *argv[]);
extern int ArduPilot_main(int argc, char* argv[]);
extern int rgbled_main(int argc, char *argv[]);

static main_t main_list[]=
{	
    {rgbled_main, 2, {"rgbled_main", "start"}},
    {rgbled_main, 5, {"rgbled_main", "rgb", "16", "16", "16"}},
//    {rgbled_main, 2, {"rgbled_main", "test"}},
    {fmu_main, 2, {"fmu_main", "mode_pwm4"}},
    {i3g4250d_main, 2, {"i3g4250d_main","start"}},
//    {i3g4250d_main, 4, {"i3g4250d_main", "-R", "10","start"}},
//    {iis328dq_main, 4, {"iis328dq_main", "-R", "6","start"}},
    {iis328dq_main, 4, {"iis328dq_main", "-R", "10","start"}},
//	{lis3mdl_main, 4, {"lis3mdl_main", "-R", "6","start"}},
	{lis3mdl_main, 4, {"lis3mdl_main", "-R", "8","start"}},
	{ms5611_main, 2, {"ms5611_main","start"}},
//	{hmc5883_main, 2, {"hmc5883_main","start"}}
    {ArduPilot_main, 2, {"ArduPilot_main", "start"}},
};

static void cmd_task(void *pvParameters)
{
    char cmd[MAX_CMD_LEN] ={0};
    int argc = 0;
    char *argv[40] = {NULL};
    char func_name[MAX_CMD_LEN] = {0};
	int num = sizeof(main_list)/sizeof(main_t);

    while(1)
    {
        if(*CMD_LINE_POINTER != 0)
        {
            memcpy(cmd, (void *)CMD_LINE_POINTER, sizeof(cmd)); 
            argv[argc] = strtok(cmd, " \n");//fgets会把最后的回车符也读进来
            argc++;
            if(argc < 40)
            {
                while(1)
                {
                    argv[argc] = strtok(NULL, " \n");
                    if(argv[argc] == NULL)
                        break;
                    argc++;
                }
            }
            else
                Print_Err("invalid argc:%d\n", argc);
            

            int i;
         /*   for(i = 0; i < argc; i++)
                Print_Info("%s\n", argv[i]);
*/
            snprintf(func_name, sizeof(func_name), "%s%s", argv[0], "_main");
            argv[0] = func_name;
            for(i = 0; i < num; i++)
            {
                if(strcmp(func_name, main_list[i].argv[0]) == 0)
                {
                    if(main_list[i].main_func!= NULL)
                    {
                        main_list[i].main_func(argc, argv);
                        break; 
                    }
                    else
                    {
                        Print_Err("main_func == null func_name=%s\n", func_name);
                    }
                }
            }
            argc = 0;
            memset(argv, 0, sizeof(argv));
            memset((void *)CMD_LINE_POINTER, 0, sizeof(cmd));
            Xil_DCacheFlushRange(CMD_ADDR, sizeof(cmd));//must flush the cache, or value in ocm will not change
        }
		vTaskDelay(300 / portTICK_RATE_MS);
    }
}

static void start_main_list()
{
	int num = sizeof(main_list)/sizeof(main_t);
	int i;

    Print_Info("start everything\n");

	for(i = 0; i < num; i++)
	{
		if(main_list[i].main_func(main_list[i].argc, main_list[i].argv) != 0)
		{
			Print_Err("Sensor:%d detect error!!\n", i);
		}
	}

    Print_Info("start drivers over\n");
    Print_Info("create cmd receive task:%d\n", (int)xTaskCreate(cmd_task, "command line test", 4000, NULL, 1, NULL));
}

static int cpu_peripheral_init()
{
    int i;
	int iRet = -1;

	//init usrt in arm core
	iRet = UartPsInit(0);
	if(iRet != 0)
	{
		Print_Err("Uart init failed:%d !!\n", iRet);
	}

	//init iic in arm core
    iRet = Iic_Init(0, 400000);
	if(iRet != 0)
	{
		Print_Err("IIC0 init failed:%d !!\n", iRet);
	}

    iRet = Iic_Init(1, 400000);
	if(iRet != 0)
	{
		Print_Err("IIC1 init failed:%d !!\n", iRet);
	}

	//init gpio in arm core
	iRet = GpioPsInit();
	if(iRet != 0)
	{
		Print_Err("Gpio init failed:%d !!\n", iRet);
	}

	//init spi in arm core
	 for(i=0; i<XPAR_XSPIPS_NUM_INSTANCES; i++)
	 {
	   iRet = SpiPsInit(i, 0x0);
	   if(iRet != 0)
	   {
	      Print_Err("Spi_bus %d init failed:%d !!\n", i,iRet);
	   }
	 }

#if 1
	//init fpga uart16550 ip core
    for(i = 0; i < XPAR_XUARTNS550_NUM_INSTANCES; i++)
    {
    	iRet = uartns_init(i);
        if(iRet != 0)
        {
            Print_Err("UartLite init failed:%d !!\n", iRet);
        }
    }
#endif

    //init sd card
    mmcsd_initialize(0);

	//init high-resolution timer 
	hrt_init();
	return 0;
}

static int check_cpu()
{
   union w
   { 
       int a;
       char b;
   } c;
   c.a = 1;
   return(c.b ==1);
}

static void pilot_first_task(void *param)
{
    int32_t ret = 0;

	ret = mount("/dev/mmcsd0", "/fs/microsd", "vfat", 0, NULL);
    if(ret != 0)
        Print_Err("Failed to mount sd card:%d!!\n", (int)ret);
    else
        Print_Info("mount sd card success!!\n");

    //Create APM directory
    ret = mkdir("/fs/microsd/APM", 0777);

    uorb_main();

    start_main_list();
//    ImuTest();
//    SpiTest();
//    UartTest();
    //SbusTest();
    

//    sd_test();

    while(1)
        vTaskDelay(portMAX_DELAY);
}

int pilot_bringup()
{
    /*init generic interrupt controller*/
	GicInit();

	Print_Info("freeRTOS start: is little endian:%d\n", check_cpu());

    fs_initialize();

	cpu_peripheral_init();

    //SD_Init(0);

    Print_Info("bringup pilot\n");
    xTaskCreate(pilot_first_task, "first task", 4000, NULL, 0, NULL);
}
