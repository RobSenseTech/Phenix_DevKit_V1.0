#pragma once

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <AP_HAL/AP_HAL.h>

#include "driver.h"
#include "AP_HAL_PX4.h"
#include "AP_HAL_PX4_Namespace.h"
#include "timers.h"

#ifndef OK
#define OK 0
#endif

class HAL_PX4 : public AP_HAL::HAL {
public:
    HAL_PX4();
    void run(int argc, char* const argv[], Callbacks* callbacks) const override;
};

void hal_px4_set_priority(uint8_t priority);

