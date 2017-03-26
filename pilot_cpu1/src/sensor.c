#include "sensor.h"
#include "FreeRTOS_Print.h"
#include <string.h>
#include "ocm_config.h"
#include "driver.h"
#include "task.h"

extern int mpu6000_main(int argc, char *argv[]);
extern int i3g4250d_main(int argc, char *argv[]);
extern int iis328dq_main(int argc, char *argv[]);
extern int fmu_main(int argc, char *argv[]);
extern int lis3mdl_main(int argc, char *argv[]);
extern int hmc5883_main(int argc, char *argv[]);
extern int ms5611_main(int argc, char *argv[]);
extern int ArduPilot_main(int argc, char* const argv[]);
extern int rgbled_main(int argc, char *argv[]);

//后面如果飞控型号多起来可以用宏控制不同传感器列表
static SensorList_t xSensorList[]=
{	
    {rgbled_main, 2, {"rgbled_main", "start"}},
    {rgbled_main, 5, {"rgbled_main", "rgb", "16", "16", "16"}},
//    {rgbled_main, 2, {"rgbled_main", "test"}},
    {fmu_main, 2, {"fmu_main", "mode_pwm4"}},
    //{mpu6000_main, 3, {"mpu6000_main","start","-X"}},
    {i3g4250d_main, 2, {"i3g4250d_main","start"}},
//    {i3g4250d_main, 4, {"i3g4250d_main", "-R", "10","start"}},
//    {iis328dq_main, 4, {"iis328dq_main", "-R", "6","start"}},
    {iis328dq_main, 4, {"iis328dq_main", "-R", "10","start"}},
//	{lis3mdl_main, 4, {"lis3mdl_main", "-R", "6","start"}},
	{lis3mdl_main, 4, {"lis3mdl_main", "-R", "8","start"}},
	{ms5611_main, 2, {"ms5611_main","start"}},
    {ArduPilot_main, 2, {"ArduPilot_main", "start"}},
//	{hmc5883_main, 2, {"hmc5883_main","start"}}
};

static void prvCmdTask(void *pvParameters)
{
    char cmd[MAX_CMD_LEN] ={0};
    int argc = 0;
    char *argv[40] = {NULL};
    char func_name[MAX_CMD_LEN] = {0};
	int iSensorNum = sizeof(xSensorList)/sizeof(SensorList_t);

    while(1)
    {
        if(*CMD_LINE_POINTER != 0)
        {
            memcpy(cmd, CMD_LINE_POINTER, sizeof(cmd)); 
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
            for(i = 0; i < iSensorNum; i++)
            {
                if(strcmp(func_name, xSensorList[i].argv[0]) == 0)
                {
                    if(xSensorList[i].sensor_main != NULL)
                    {
                        xSensorList[i].sensor_main(argc, argv);
                        break; 
                    }
                    else
                    {
                        Print_Err("sensor_main == null func_name=%s\n", func_name);
                    }
                }
            }
            argc = 0;
            memset(argv, 0, sizeof(argv));
            memset(CMD_LINE_POINTER, 0, sizeof(cmd));
            Xil_DCacheFlushRange(CMD_ADDR, sizeof(cmd));//必须flush，不然真正的ocm中的值不会变
        }
		vTaskDelay(300 / portTICK_RATE_MS);
    }
}

void vStartSenSors()
{
	int iSensorNum = sizeof(xSensorList)/sizeof(SensorList_t);
	int i;

    Print_Info("start drivers\n");
	for(i = 0; i < iSensorNum; i++)
	{
		if(xSensorList[i].sensor_main(xSensorList[i].argc, xSensorList[i].argv) != 0)
		{
			Print_Err("Sensor:%d detect error!!\n", i);
		}
	}

    Print_Info("start drivers over\n");
    Print_Info("create cmd receive task:%d\n", xTaskCreate(prvCmdTask, "command line test", 4000, NULL, 1, NULL));
}

