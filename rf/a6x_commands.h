//
// Created by vgol on 29/11/2022.
//

#ifndef STARTRACKER_A6X_COMMANDS_H
#define STARTRACKER_A6X_COMMANDS_H

typedef enum {
    ble_a6x_app_update_focus_up      = 0x106u,
    ble_a6x_app_update_focus_down    = 0x107u,
    ble_a6x_app_update_shutter_up    = 0x108u,
    ble_a6x_app_update_shutter_down  = 0x109u,
} a6x_app_commands_t;

#endif //STARTRACKER_A6X_COMMANDS_H
