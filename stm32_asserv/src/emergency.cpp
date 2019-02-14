#include "emergency.h"
#include "pins.h"
#include "compat.h"
#include "parameters.h"

emergency_status_t emergency_status[2] = {
	{.phase = NO_EMERGENCY, .in_use = USE_SHARP, .total_time = 0},
	{.phase = NO_EMERGENCY, .in_use = USE_SHARP, .total_time = 0},
};

void EmergencySetStatus(int enable) {
	emergency_status[EM_FORWARD].in_use = enable;
	emergency_status[EM_BACKWARD].in_use = enable;
	if (!enable) {
		emergency_status[EM_FORWARD].phase = NO_EMERGENCY;
		emergency_status[EM_BACKWARD].phase = NO_EMERGENCY;
	}
	else
	{
		emergency_status[EM_FORWARD].phase = FIRST_STOP;
		emergency_status[EM_BACKWARD].phase = FIRST_STOP;
	}
}

