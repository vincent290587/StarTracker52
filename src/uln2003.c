//
// Created by vince on 28/11/2022.
//

#include "uln2003.h"

#include <app_scheduler.h>
#include <app_timer.h>
#include <nrf_drv_timer.h>
#include "helper.h"
#include "segger_wrapper.h"
#include "task_manager_wrapper.h"
#include "custom_board.h"
#include "millis.h"

#define USE_TIMER 1

/*********************************************************************
*
*       Defines, fixed
*
**********************************************************************
*/
#define DEMCR                     (*(volatile unsigned long*) (0xE000EDFCuL))   // Debug Exception and Monitor Control Register
#define TRACEENA_BIT              (1uL << 24)                                   // Trace enable bit
#define DWT_CTRL                  (*(volatile unsigned long*) (0xE0001000uL))   // DWT Control Register
#define NOCYCCNT_BIT              (1uL << 25)                                   // Cycle counter support bit
#define CYCCNTENA_BIT             (1uL << 0)                                    // Cycle counter enable bit

#define GET_TIMESTAMP()      (*(U32 *)(0xE0001004))            // Retrieve a system timestamp. Cortex-M cycle counter.
#define TIMESTAMP_BITS       32                                // Define number of valid bits low-order delivered by clock source

extern U32 SystemCoreClock;

//////////////////////////////////////////////////////////////////////////////////////

#define M_PI 3.1415926f
#define NUM_STEPS 8
#define RADS_PER_SEC 7.292115e-05f

#define LENGTH_M 0.28884f // fill in with precise measured value
#define ELE_PER_TURN (0.0005f) // fill in with precise measured value: elevation per full thread rotation

// 4096 steps per rotation
// 0.5mm per rotation,
// radians per motor rotation = 0.5mm / (2*Pi*r) = 2.6523e-4
#define RADIANS_PER_ROT (ELE_PER_TURN / (2*M_PI*LENGTH_M))

#define PULSE_PER_SEC   (4096u * RADS_PER_SEC / RADIANS_PER_ROT) /* About 1125 pulse per second */
#define USEC_PER_PULSE  (1000000uL * RADIANS_PER_ROT / (RADS_PER_SEC * 4096u)) /* pulse period in usec */
#define CNT_PER_PULSE   (64000000uL * RADIANS_PER_ROT / (RADS_PER_SEC * 4096u)) /* pulse period in counts */

// from manufacturers datasheet
static const U8 m_stepper_sequence[NUM_STEPS] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};

static volatile U32 m_total_steps = 0;
static volatile U8 m_stepper_index = 0;

static inline void _increment(void) {

    nrf_gpio_pin_write(ULN_PINA, m_stepper_sequence[m_stepper_index] & 0b0001);
    nrf_gpio_pin_write(ULN_PINB, m_stepper_sequence[m_stepper_index] & 0b0010);
    nrf_gpio_pin_write(ULN_PINC, m_stepper_sequence[m_stepper_index] & 0b0100);
    nrf_gpio_pin_write(ULN_PIND, m_stepper_sequence[m_stepper_index] & 0b1000);

    m_stepper_index = (++m_stepper_index) & 0b0111;
    m_total_steps++;

    //NRF_LOG_INFO("Increment %u", m_stepper_index);
}

/**
 * @brief Handler for timer events.
 */
static void timer_event_handler(nrf_timer_event_t event_type,
                                void            * p_context)
{
    W_SYSVIEW_RecordEnterISR();

    if (NRF_TIMER_EVENT_COMPARE0 == event_type) {
        _increment();
    }

    W_SYSVIEW_RecordExitISR();
}

//////////////////////////////////////////////////////////////////////////////////////

void uln2003__init(void) {

    nrf_gpio_cfg_output(ULN_PINA);
    nrf_gpio_cfg_output(ULN_PINB);
    nrf_gpio_cfg_output(ULN_PINC);
    nrf_gpio_cfg_output(ULN_PIND);

#if USE_TIMER
//    APP_TIMER_DEF(m_job_timer);
//
//    ret_code_t err_code;
//    err_code = app_timer_create(&m_job_timer, APP_TIMER_MODE_REPEATED, timer_event_handler);
//    APP_ERROR_CHECK(err_code);
//
//    err_code = app_timer_start(m_job_timer, APP_TIMER_TICKS(1000), NULL);
//    APP_ERROR_CHECK(err_code);

    static const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(1);

    uint32_t time_ticks;
    //Configure TIMER_LED for generating simple light effect - leds on board will invert his state one after the other.
    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    uint32_t err_code = nrf_drv_timer_init(&TIMER_LED, &timer_cfg, timer_event_handler);
    APP_ERROR_CHECK(err_code);

    time_ticks = nrf_drv_timer_us_to_ticks(&TIMER_LED, USEC_PER_PULSE);

    nrf_drv_timer_extended_compare(
            &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

    nrf_drv_timer_enable(&TIMER_LED);
#else
    //
    //  The cycle counter must be activated in order
    //  to use time related functions.
    //
    if ((DWT_CTRL & NOCYCCNT_BIT) == 0) {       // Cycle counter supported?
        if ((DWT_CTRL & CYCCNTENA_BIT) == 0) {    // Cycle counter not enabled?
            DWT_CTRL |= CYCCNTENA_BIT;              // Enable Cycle counter
        }
    }
#endif
}

void uln2003__service(void) {

#if USE_TIMER==0

    if (millis() < 3000) {
        return;
    }

    static U32 m_commanded_steps = 0;
    static U32 m_start_cnt = 0;

    if (!m_start_cnt) {
        m_start_cnt = GET_TIMESTAMP();
    }

    U32 diff_cnt = (GET_TIMESTAMP() - m_start_cnt);
    U32 th_steps = diff_cnt / CNT_PER_PULSE;

    if (th_steps > m_total_steps) {
        m_commanded_steps = th_steps - m_total_steps;
    } else {
        m_commanded_steps = 0;
    }

    if (m_commanded_steps) {
        _increment();
    }
#endif

}
