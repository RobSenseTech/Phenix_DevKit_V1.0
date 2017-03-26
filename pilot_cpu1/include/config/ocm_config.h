#ifndef _OCM_CONFIG_H_
#define _OCM_CONFIG_H_

/**********************************OCM***************************************/
/*
 *zynq-ocm f800c000.ocmc: OCM block 0, start fffc0000, end fffcffff
 *zynq-ocm f800c000.ocmc: OCM block 1, start fffd0000, end fffdffff
 *zynq-ocm f800c000.ocmc: OCM block 2, start fffe0000, end fffeffff
 *zynq-ocm f800c000.ocmc: OCM block 3, start ffff0000, end ffffffff
 *zynq-ocm f800c000.ocmc: OCM resources 0, start fffc0000, end ffffffff
*/
#define CMD_ADDR                (0xFFFD0000)
#define CMD_LINE_POINTER        (volatile unsigned long *)CMD_ADDR//由cpu0初始化
#define MAX_CMD_LEN     50
/*
  This stores eeprom data in the PX4 MTD interface with a 4k size, and
  a in-memory buffer. This keeps the latency and devices IOs down.
 */

// name the storage file after the sketch so you can use the same sd
// card for ArduCopter and ArduPlane
#define SKETCHNAME "zynq_pilot" 
#define STORAGE_DIR "/mnt/APM"
#define MTD_PARAMS_FILE "/mnt/pilot.param" //通过OCM把数据发给cpu0，由linux写入数据
#define OLD_STORAGE_FILE STORAGE_DIR "/" SKETCHNAME ".stg"
#define OLD_STORAGE_FILE_BAK STORAGE_DIR "/" SKETCHNAME ".bak"

#define COMM_TX_FLAG_ADDR     (0xFFFE0000)
#define COMM_TX_FLAG    *(volatile unsigned long *)COMM_TX_FLAG_ADDR    //由cpu0初始化

#define OCM_STORAGE_START_ADDR      (0xFFFE0004)
#define OCM_STORAGE_START           (volatile unsigned long *)OCM_STORAGE_START_ADDR 
#define HAL_STORAGE_SIZE            16384





#define COMM_LOG_WRITE_ADDR     (0xFFFF0000)
#define COMM_LOG_WRITE    *(volatile unsigned long *)COMM_LOG_WRITE_ADDR    //由cpu0初始化
#define COMM_LOG_READ_ADDR      (0xFFFF0004)
#define COMM_LOG_READ     *(volatile unsigned long *)COMM_LOG_READ_ADDR    //由cpu0初始化

#define OCM_LOG_START_ADDR      (0xFFFF0008)
#define OCM_LOG_START           (volatile unsigned long *)OCM_LOG_START_ADDR 
#define HAL_LOG_SIZE            16384
#define LOG_DEVICE "/dev/log" //通过OCM把数据发给cpu0，由linux写入数据

#endif
