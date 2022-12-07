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

bool app_ble_peripheral__is_connected();

#ifdef __cplusplus
}
#endif

#endif /* RF_BLE_API_BASE_H_ */
