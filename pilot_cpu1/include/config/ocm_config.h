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

#endif
