#ifndef SENEOR_H
#define SENSOR_H

typedef struct
{
	int (*main_func)(int argc, char *argv[]);
    int argc;
    char *argv[40];
}main_t;


int pilot_bringup();

#endif

