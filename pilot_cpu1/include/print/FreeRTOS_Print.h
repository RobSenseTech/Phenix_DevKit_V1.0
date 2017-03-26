#ifndef FREERTOS_PRINT_H
#define FREERTOS_PRINT_H

#ifdef __cplusplus
extern "C"{
#include <stdio.h>

//Green
#define Print_Info(format, ...) ::printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__)
//Yellow
#define Print_Warn(format, ...) ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__)
//Red
#define Print_Err(format, ...)  ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__)

#define err(eval, format, ...)            ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define verr(eval, format, ...)  
#define errc(eval, errcode, format, ...)  ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define verrc(eval, errcode, format, ...) ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define errx(eval, format, ...)           ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define verrx(eval, format, ...)          ::printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 


//  support old warn print function.
#define warn(format, ...)         ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define vwarn(format, ...)        ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define warnc(code, format, ...)  ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define vwarnc(code, format, ...) ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define warnx(format, ...)        ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\n\r", __func__, __LINE__, ##__VA_ARGS__) 
#define vwarnx(format, ...)       ::printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\n\r", __func__, __LINE__, ##__VA_ARGS__) 

//  support old px4 log print function.
#define DEVICE_DEBUG(format, ...) ::printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\n\r", __func__, __LINE__, ##__VA_ARGS__) 
#define DEVICE_LOG(format, ...)   ::printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\n\r", __func__, __LINE__, ##__VA_ARGS__) 

}
#else
#include <stdio.h>

//Green
#define Print_Info(format, ...) printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__)
//Yellow
#define Print_Warn(format, ...) printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__)
//Red
#define Print_Err(format, ...)  printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__)

#define err(eval, format, ...)              printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define verr(eval, format, ...)  
#define errc(eval, errcode, format, ...)    printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define verrc(eval, errcode, format, ...)   printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define errx(eval, format, ...)             printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define verrx(eval, format, ...)            printf("\033[1;31;40m [FreeRTOS ERR]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 


//  support old warn print function.
#define warn(format, ...)           printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define vwarn(format, ...)          printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define warnc(code, format, ...)    printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define vwarnc(code, format, ...)   printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define warnx(format, ...)          printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define vwarnx(format, ...)         printf("\033[1;33;40m [FreeRTOS WARN]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 

//  support old px4 log print function.
#define DEVICE_DEBUG(format, ...)   printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#define DEVICE_LOG(format, ...)     printf("\033[1;32;40m [FreeRTOS INFO]%s line:%d: " format "\r", __func__, __LINE__, ##__VA_ARGS__) 
#endif


#endif
