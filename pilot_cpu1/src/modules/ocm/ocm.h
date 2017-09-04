#ifndef _ZYNQ_OCM_H_
#define _ZYNQ_OCM_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>

#define OCM_CHN_FS  0 //ocmfs use channel 0

#define OCM_ALIGN_BYTES  (4)

int32_t ocm_msg_init();
int32_t ocm_msg_chn_init(int32_t chn, char *owner);
int32_t ocm_msg_recv(int32_t chn, uint8_t *data, int32_t len);
int32_t ocm_msg_send(int32_t chn, uint8_t *data, int32_t len);
bool    ocm_msg_busy(int32_t chn, int32_t len);
#endif

