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


static uint32_t m_last_ping = 0;


void power_scheduler__run(void) {


	if (millis() - m_last_ping > (600000 * POWER_SCHEDULER_MAX_IDLE_MIN)) {

		//nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_STAY_IN_SYSOFF);
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

	case ePowerSchedulerPingANT:
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
