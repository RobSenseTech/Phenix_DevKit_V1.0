#ifndef SENEOR_H
#define SENSOR_H

typedef struct
{
	int (*sensor_main)(int argc, char *argv[]);
    int argc;
    char *argv[40];
}SensorList_t;


extern void vStartSenSors();

#endif

