//
// Created by vgol on 29/11/2022.
//

#ifndef STARTRACKER_A6X_HANDLER_H
#define STARTRACKER_A6X_HANDLER_H

#include <stdint.h>

typedef enum {
    eA6X_sm_state_idle     = 0u,
    eA6X_sm_state_start    = 1u,
    eA6X_sm_state_focus    = 2u,
    eA6X_sm_state_trigger  = 3u,
    eA6X_sm_state_done     = 4u,

    eA6X_sm_state_stop,
} eA6X_sm_state;

#ifdef __cplusplus
extern "C" {
#endif

void a6x_handler__set_state(eA6X_sm_state state);

void a6x_handler__on_data(uint8_t second, uint8_t third);

void a6x_handler__run_sm(void);

#ifdef __cplusplus
}
#endif

#endif //STARTRACKER_A6X_HANDLER_H
