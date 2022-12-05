//
// Created by vgol on 29/11/2022.
//

#include <stdbool.h>
#include <stdint.h>
#include <nrf_error.h>
#include <app_error.h>

#include "a6x_handler.h"
#include "app_ble_central.h"

#include "segger_wrapper.h"
#include "task_manager_wrapper.h"
#include "a6x_commands.h"
#include "uln2003.h"


static volatile bool m_focus_acquired = false;
static volatile bool m_shutter_triggered = false;

static eA6X_sm_state _state = eA6X_sm_state_idle;

static uint8_t  m_picture_nb;
static uint16_t m_exposure;

void a6x_handler__on_data(uint8_t second, uint8_t third) {


    switch (second) {
        case 0x3F: {
            uint8_t _focusStatus = third;

            if (_focusStatus == 0x20) {
                m_focus_acquired = true;
//                            rs->set(Status::FOCUS_ACQUIRED);
            } else {
                m_focus_acquired = false;
//                            rs->set(Status::READY);
            }

        } break;

        case 0xA0: {
            uint8_t _shutterStatus = third;

            if (_shutterStatus == 0x20) {
//                            rs->set(Status::SHUTTER);
                m_shutter_triggered = true;
            } else {
//                            rs->set(Status::READY);
                m_shutter_triggered = false;
            }

        } break;

        case 0xD5:
        {
            uint8_t _recordingStatus = third;
        } break;

        default:
            // No implementation needed.
            break;
    }
}

void a6x_handler__program(uint8_t picture_nb, uint16_t exposure) {

    m_picture_nb = picture_nb;
    m_exposure = exposure;
}

void a6x_handler__set_state(eA6X_sm_state state) {

    if (_state == eA6X_sm_state_idle && state == eA6X_sm_state_start) {
        _state = eA6X_sm_state_start;

        // start rotation
        uln2003__pause(1);
    }

    if (state == eA6X_sm_state_idle) {
        _state = eA6X_sm_state_idle;

        // start rotation
        uln2003__pause(0);
    }
}

/*
 * All commands send one code for pressing the button down, and another when you release the button with the exception
 * of the video record button which only sends when you press it down, and operates like a toggle.
 * A light on the remote stays on when the camera is recording, but I'm not absolutely sure that it actually
 * is reading that status back out of the camera vs just tracking it's own state.
 */
void a6x_handler__run_sm(void) {

    // on / off

    // focus
    if (m_focus_acquired && _state == eA6X_sm_state_start) {
        _state = eA6X_sm_state_trigger;
    }
    // shutter
    if (m_shutter_triggered && _state == eA6X_sm_state_trigger) {
        _state = eA6X_sm_state_done;
    }

    switch (_state) {
        case eA6X_sm_state_idle: // nothing
            break;

        case eA6X_sm_state_start:
        {
            m_focus_acquired = false;
            m_shutter_triggered = false;

            app_ble_central__send_a6x_command(ble_a6x_app_update_focus_up);

            w_task_delay(10);

            app_ble_central__send_a6x_command(ble_a6x_app_update_focus_down);
        } break;

        case eA6X_sm_state_focus:
        {
            // TODO remove dummy state
            app_ble_central__send_a6x_command(ble_a6x_app_update_shutter_up);

            _state = 3u;
        } break;

        case eA6X_sm_state_trigger:
        {
            if (!m_focus_acquired) {
                _state = 1u;
                return;
            }

            app_ble_central__send_a6x_command(ble_a6x_app_update_shutter_up);

            w_task_delay(10);

            app_ble_central__send_a6x_command(ble_a6x_app_update_shutter_down);

            w_task_delay(m_exposure);

            app_ble_central__send_a6x_command(ble_a6x_app_update_shutter_up);

            w_task_delay(10);

            app_ble_central__send_a6x_command(ble_a6x_app_update_shutter_down); // releases shutter : end BULB with that one

            w_task_delay(100);

            app_ble_central__send_a6x_command(ble_a6x_app_update_focus_up);
        } break;

        case eA6X_sm_state_done:
        {
            NRF_LOG_INFO("Picture taken !");

            m_focus_acquired = false;
            m_shutter_triggered = false;

            if (m_picture_nb > 1) {
                m_picture_nb--;
                _state = eA6X_sm_state_start;
            } else {
                m_picture_nb = 0;
                _state = eA6X_sm_state_idle;
                // end rotation
                uln2003__pause(0);
            };
        } break;

        default:
            break;
    }

}