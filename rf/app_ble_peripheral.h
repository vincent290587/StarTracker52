/*
 * ble_api.h
 *
 *  Created on: 21 dec. 2018
 *      Author: Vincent
 */

#ifndef RF_BLE_API_BASE_H_
#define RF_BLE_API_BASE_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ble_init(void);

uint16_t ble_get_mtu(void);

void app_ble_advertising_start(void);

uint32_t app_ble_nus_data_send(uint8_t *p_data, uint16_t length);


#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_API_BASE_H_ */
