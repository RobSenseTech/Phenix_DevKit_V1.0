#ifndef _SBUS_H_
#define _SBUS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C"{
#endif

__EXPORT int	sbus_init(const char *device, bool singlewire);
__EXPORT bool	sbus_input(int sbus_fd, uint16_t *values, uint16_t *num_values, bool *sbus_failsafe,
			   bool *sbus_frame_drop,
			   uint16_t max_channels);

#ifdef __cplusplus
}
#endif
#endif
