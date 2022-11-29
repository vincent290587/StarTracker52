//
// Created by vgol on 29/11/2022.
//

#ifndef STARTRACKER_APP_BLE_CENTRAL_H
#define STARTRACKER_APP_BLE_CENTRAL_H

#include "a6x_commands.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_ble_central__init(void);

void scan_start(void);

void app_ble_central__take_pic(bool start);

void app_ble_central__send_a6x_command(a6x_app_commands_t command);

#ifdef __cplusplus
}
#endif

#endif //STARTRACKER_APP_BLE_CENTRAL_H
