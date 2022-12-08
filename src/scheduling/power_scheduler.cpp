/*
 * power_scheduler.cpp
 *
 *  Created on: 10 févr. 2020
 *      Author: Vincent
 */

#include <string.h>
#include "millis.h"
#include "boards.h"
#include "Model.h"
#include "power_scheduler.h"
#include "segger_wrapper.h"


#define POWER_SCHEDULER_MAX_IDLE_MIN               10
#define MILLIS_IN_MINUTE                           60000u

static uint32_t m_last_ping = 0;


void power_scheduler__run(void) {


	if (millis() - m_last_ping > (MILLIS_IN_MINUTE * POWER_SCHEDULER_MAX_IDLE_MIN)) {

		power_scheduler__shutdown();
	}

}


void power_scheduler__shutdown(void) {

	NRF_LOG_ERROR("power_scheduler__shutdown");

	w_task_delay(100);

	// shutdown system
	app_shutdown();

}


void power_scheduler__ping(ePowerSchedulerPing ping_type) {


	switch (ping_type) {

	case ePowerSchedulerPingULN:
	{
		m_last_ping = millis();
	} break;

	case ePowerSchedulerPingBLE:
	{
		m_last_ping = millis();
	} break;


	default:
		break;
	}

}
