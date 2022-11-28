/*
 * Model.cpp
 *
 *  Created on: 17 oct. 2017
 *      Author: Vincent
 */

#include "Model.h"
#include "sdk_config.h"
#include "helper.h"
#include "app_scheduler.h"
#include "power_scheduler.h"
#include "segger_wrapper.h"

#if defined (BLE_STACK_SUPPORT_REQD)
#include "app_ble_peripheral.h"
#include "uln2003.h"

#endif

#ifdef ANT_STACK_SUPPORT_REQD
#include "app_ant.h"

#endif


/**
 *
 */
void peripherals_task(void * p_context) {

    w_task_delay(10);

    wdt_reload();

    for(;;)
    {
    	w_task_delay(500);

    	wdt_reload();

    	power_scheduler__run();
    }

}

/**
 *
 */
void sensors_task(void * p_context) {

    w_task_delay(4000);

    for(;;)
    {
        w_task_delay(5000);
    }

}

/**
 *
 * @param p_context
 */
void idle_task(void * p_context)
{

    uln2003__init();

    for(;;)
    {
    	// BSP tasks
    	// bsp_tasks();

    	NRF_LOG_FLUSH();

    	//No more logs to process, go to sleep
//    	W_SYSVIEW_OnIdle();
//    	pwr_mgmt_run();

#if APP_SCHEDULER_ENABLED
    	app_sched_execute();
#endif

        uln2003__service();

    	task_yield();
    }
}
