/*
 * Global.h
 *
 *  Created on: 17 oct. 2017
 *      Author: Vincent
 */

#ifndef SOURCE_MODEL_H_
#define SOURCE_MODEL_H_

#include <stdbool.h>
#include "parameters.h"
#include "segger_wrapper.h"

#if defined(__cplusplus)
extern "C" {
#endif // defined C++

void perform_system_tasks_light(void);

void wdt_reload(void);

void app_shutdown(void);

void sensors_task(void * p_context);

void idle_task(void * p_context);

void peripherals_task(void * p_context);

#if defined(__cplusplus)
}
#endif // defined C++


#endif /* SOURCE_MODEL_H_ */
