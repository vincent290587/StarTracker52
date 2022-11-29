//
// Created by vgol on 29/11/2022.
//

#ifndef STARTRACKER_APP_BLE_CENTRAL_H
#define STARTRACKER_APP_BLE_CENTRAL_H


#ifdef __cplusplus
extern "C" {
#endif

void app_ble_central__init(void);

void scan_start(void);

void app_ble_central__take_pic(bool start);

#ifdef __cplusplus
}
#endif

#endif //STARTRACKER_APP_BLE_CENTRAL_H
