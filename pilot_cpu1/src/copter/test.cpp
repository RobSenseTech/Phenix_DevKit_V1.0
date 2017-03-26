// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"

#if CLI_ENABLED == ENABLED

void Copter::print_hit_enter()
{
    cliSerial->printf("Hit Enter to exit.\n\n");
}

#endif // CLI_ENABLED
