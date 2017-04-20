#ifndef FREERTOS_PRINT_H
#define FREERTOS_PRINT_H

#ifdef __cplusplus
extern "C"{
#include <stdio.h>

//Green
#define pilot_info(format, ...) ::printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\033[0m\r", __func__, __LINE__, ##__VA_ARGS__)
//Yellow
#define pilot_warn(format, ...) ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\033[0m\r", __func__, __LINE__, ##__VA_ARGS__)
//Red
#define pilot_err(format, ...)  ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\033[0m\r", __func__, __LINE__, ##__VA_ARGS__)

}
#else
#include <stdio.h>

//Green
#define pilot_info(format, ...) printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\033[0m\r", __func__, __LINE__, ##__VA_ARGS__)
//Yellow
#define pilot_warn(format, ...) printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\033[0m\r", __func__, __LINE__, ##__VA_ARGS__)
//Red
#define pilot_err(format, ...)  printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\033[0m\r", __func__, __LINE__, ##__VA_ARGS__)

#endif

#define err(eval, format, ...)            pilot_err(format"\n", ##__VA_ARGS__)   
#define errc(eval, errcode, format, ...)  pilot_err(format"\n", ##__VA_ARGS__)
#define verrc(eval, errcode, format, ...) pilot_err(format"\n", ##__VA_ARGS__)
#define errx(eval, format, ...)           pilot_err(format"\n", ##__VA_ARGS__)
#define verrx(eval, format, ...)          pilot_err(format"\n", ##__VA_ARGS__)


//  support old warn print function.
#define warn(format, ...)                 pilot_warn(format"\n", ##__VA_ARGS__)   
#define warnc(code, format, ...)          pilot_warn(format"\n", ##__VA_ARGS__)
#define vwarnc(code, format, ...)         pilot_warn(format"\n", ##__VA_ARGS__)
#define warnx(format, ...)                pilot_warn(format"\n", ##__VA_ARGS__)
#define vwarnx(format, ...)               pilot_warn(format"\n", ##__VA_ARGS__)

//  support old px4 log print function.
#define DEVICE_DEBUG(format, ...)         pilot_info(format"\n", ##__VA_ARGS__)
#define DEVICE_LOG(format, ...)           pilot_info(format"\n", ##__VA_ARGS__)

#define fdbg(format, ...)    
#endif
