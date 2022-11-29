/*
 * App.cpp
 *
 *  Created on: 8 oct. 2017
 *      Author: Vincent
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <nrf_drv_power.h>

#include "hardfault.h"
#include "helper.h"
#include "bsp.h"
#include "millis.h"
#include "boards.h"
#include "app_ble_peripheral.h"
#include "bsp_btn_ble.h"
#include "app_scheduler.h"
#include "app_timer.h"
#include "nrf_sdm.h"
#include "nrf_strerror.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_clock.h"
#include "task_manager.h"
#include "nrf_bootloader_info.h"
#include "nrfx_wdt.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"
#include "Model.h"
#include "segger_wrapper.h"

#ifdef SOFTDEVICE_PRESENT

#include "nrf_sdh.h"

#endif

#ifdef __cplusplus
{
#endif

#include "nrf_pwr_mgmt.h"
#include "app_ble_central.h"

#ifdef __cplusplus
}
#endif


#define SCHED_MAX_EVENT_DATA_SIZE      32              /**< Maximum size of scheduler events. */
#ifdef SVCALL_AS_NORMAL_FUNCTION
#define SCHED_QUEUE_SIZE               40                                           /**< Maximum number of events in the scheduler queue. More is needed in case of Serialization. */
#else
#define SCHED_QUEUE_SIZE               40              /**< Maximum number of events in the scheduler queue. */
#endif

nrfx_wdt_channel_id m_channel_id;

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name) {
    assert_info_t assert_info =
            {
                    .line_num    = line_num,
                    .p_file_name = file_name,
            };
    app_error_fault_handler(NRF_FAULT_ID_SDK_ASSERT, 0, (uint32_t) (&assert_info));

#ifndef DEBUG_NRF_USER
    LOG_WARNING("System reset");
    LOG_FLUSH();
    NVIC_SystemReset();
#else
    NRF_BREAKPOINT_COND;

    bool loop = true;
    while (loop);
#endif // DEBUG

    UNUSED_VARIABLE(assert_info);
}


/**@brief Function for error handling, which is called when an error has occurred.
 *
 * @param[in] error_code  Error code supplied to the handler.
 */
void app_error_handler_bare(ret_code_t error_code) {

    NRF_LOG_ERROR("Error %u (0x%X)", error_code, error_code);
    NRF_LOG_FLUSH();

}

/**
 *
 * @param id
 * @param pc
 * @param info
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info) {
    NRF_LOG_FLUSH();

    switch (id) {
#if defined(SOFTDEVICE_PRESENT)
        case NRF_FAULT_ID_SD_ASSERT:
            NRF_LOG_ERROR("SOFTDEVICE: ASSERTION FAILED");
            break;
        case NRF_FAULT_ID_APP_MEMACC:
            NRF_LOG_ERROR("SOFTDEVICE: INVALID MEMORY ACCESS");
            break;
#endif
        case NRF_FAULT_ID_SDK_ASSERT: {
            assert_info_t *p_info = (assert_info_t *) info;
#if USE_SVIEW
            SEGGER_SYSVIEW_ErrorfHost(
                    "ASSERTION FAILED at %s:%u",
                    p_info->p_file_name,
                    p_info->line_num);
#else
            NRF_LOG_ERROR(
                    "ASSERTION FAILED at %s:%u",
                    p_info->p_file_name,
                    p_info->line_num);
#endif
            break;
        }
        case NRF_FAULT_ID_SDK_ERROR: {
            error_info_t *p_info = (error_info_t *) info;
#if USE_SVIEW
            SEGGER_SYSVIEW_ErrorfHost(
                    "ERROR 0x%X [%s] at %s:%u",
                    (unsigned int)p_info->err_code,
                    nrf_strerror_get(p_info->err_code),
                    p_info->p_file_name,
                    (uint16_t)p_info->line_num);
#else
            NRF_LOG_ERROR(
                    "ERROR 0x%X [%s] at %s:%u",
                    (unsigned int) p_info->err_code,
                    nrf_strerror_get(p_info->err_code),
                    p_info->p_file_name,
                    (uint16_t) p_info->line_num);
#endif
            break;
        }
        default: {
            NRF_LOG_ERROR("UNKNOWN FAULT at 0x%08X", pc);
            break;
        }
    }

    NRF_LOG_FLUSH();

#ifdef DEBUG_NRF
    if (id != NRF_FAULT_ID_SDK_ERROR) {
        NRF_BREAKPOINT_COND;
    }
    // On assert, the system can only recover with a reset.
#endif

}

void HardFault_process(HardFault_stack_t *p_stack) {
#ifdef DEBUG_NRF
    NRF_BREAKPOINT_COND;
    // On hardfault, the system can only recover with a reset.

    while (true);
#endif
    // Restart the system by default
    NVIC_SystemReset();
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void) {
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Handler for shutdown preparation.
 *
 * @details During shutdown procedures, this function will be called at a 1 second interval
 *          untill the function returns true. When the function returns true, it means that the
 *          app is ready to reset to DFU mode.
 *
 * @param[in]   event   Power manager event.
 *
 * @retval  True if shutdown is allowed by this power manager handler, otherwise false.
 */
static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event) {
    ret_code_t err_code;

    switch (event) {
        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP: {
            LOG_WARNING("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP");

            // prepare wakeup source
            err_code = bsp_btn_ble_sleep_mode_prepare();
            APP_ERROR_CHECK(err_code);
        }
            // no break
        case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF: {
            LOG_WARNING("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF");

            // Stop all timers
            err_code = app_timer_stop_all();
            APP_ERROR_CHECK(err_code);

            NRF_LOG_FLUSH();

#ifdef SOFTDEVICE_PRESENT
            nrf_sdh_disable_request();
#endif

        }
            break;
        case NRF_PWR_MGMT_EVT_PREPARE_DFU: {
            LOG_WARNING("NRF_PWR_MGMT_EVT_PREPARE_DFU");

            // we are always in BLE mode here

            err_code = app_timer_stop_all();
            APP_ERROR_CHECK(err_code);

        }
            break;
        default:
            break;
    }

    LOG_FLUSH();

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void) {
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void _bsp_event_callback(bsp_event_t evt) {

    NRF_LOG_WARNING("_bsp_event_callback %d", evt);

    switch (evt) {

        case BSP_EVENT_KEY_0:
            break;

        default:
            break;
    }

}

/**@brief Function for initializing buttons and LEDs. */
static void leds_init(void) {
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, _bsp_event_callback);
    APP_ERROR_CHECK(err_code);
}

static void pins_init(void) {
    ret_code_t err_code;

    err_code = nrfx_gpiote_init();
    APP_ERROR_CHECK(err_code);

}

void wdt_reload(void) {
#if NRFX_WDT_ENABLED
    nrfx_wdt_channel_feed(m_channel_id);
#endif
}

void app_shutdown(void) {
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}


#ifdef SOFTDEVICE_PRESENT

/**@brief Function for initializing the softdevice
 *
 * @details Initializes the SoftDevice
 */
static void sdh_init(void) {
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    ASSERT(nrf_sdh_is_enabled());

    //sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

}

#endif


/**
 *
 * @return 0
 */
int main(void) {
    ret_code_t err_code;

    // Initialize.
    //Configure WDT.
#if NRFX_WDT_ENABLED
    nrfx_wdt_config_t wdt_config = NRFX_WDT_DEAFULT_CONFIG;
    err_code = nrfx_wdt_init(&wdt_config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
#endif

    millis_init();

    segger_init();

    log_init();

#ifdef FPU_INTERRUPT_MODE
    // Enable FPU interrupt
    NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
    NVIC_ClearPendingIRQ(FPU_IRQn);
    NVIC_EnableIRQ(FPU_IRQn);
#endif

    pins_init();

    LOG_INFO("Init start");

    {
        nrf_drv_power_config_t pPower = {
                .dcdcen = false,
                .dcdcenhv = false,
        };
        err_code = nrf_drv_power_init(&pPower);
        APP_ERROR_CHECK(err_code);
    }

    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);

#if APP_SCHEDULER_ENABLED
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
#endif

    leds_init();

    // init BLE + ANT
#ifdef SOFTDEVICE_PRESENT
    sdh_init();
#endif
#if defined (ANT_STACK_SUPPORT_REQD)
    ant_stack_init();
    ant_setup_init();
    ant_setup_start();
#endif
#if defined (BLE_STACK_SUPPORT_REQD)
    ble_init();
    app_ble_central__init();
#endif

    LOG_INFO("App init done");

    NRF_LOG_FLUSH();

    task_manager_start_timer();

	//(void)task_create(sensors_task		, "sensors_task"		, NULL, 2048);
	(void)task_create(peripherals_task	, "peripherals_task"	, NULL, 2048);

	// does not return
	task_manager_start(idle_task, NULL);

}

