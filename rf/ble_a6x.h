//
// Created by vgol on 29/11/2022.
//

#ifndef STARTRACKER_BLE_A6X_H
#define STARTRACKER_BLE_A6X_H

#include <stdint.h>
#include <stdbool.h>

#include "ble.h"
#include "ble_srv_common.h"
#include "nrf_sdh_ble.h"

/**@brief   Macro for defining an instance.
 *
 * @param   _name   Name of the instance.
 * @hide initializer
 */
#define BLE_A6X_C_DEF(_name)                                                                          \
static ble_a6x_srv_t _name;                                                                         \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_LBS_BLE_OBSERVER_PRIO,                                                     \
                     ble_a6x_c_on_ble_evt, &_name)

// 8000FF00-FF00-FFFF-FFFF-FFFFFFFFFFFF
#define A6X_UUID_BASE        {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, \
                              0xff, 0xff, 0x00, 0xff, 0x00, 0xff, 0x00, 0x80}

#define A6X_UUID_SRV    0xFF00
#define A6X_UUID_RC     0xFF01 // write
#define A6X_UUID_RA     0xFF02 // notify

// Forward declaration of the type.
typedef struct ble_a6x_srv_s ble_a6x_srv_t;

typedef void (*ble_a6x_write_handler_t) (uint16_t conn_handle, const uint8_t *p_data, uint16_t len);

typedef enum
{
    BLE_A6X_C_EVT_DISCOVERY_COMPLETE,  /**< Event indicating that Service has been discovered at the peer. */
    BLE_A6X_C_EVT_DISCOVERY_FAILED,
    BLE_A6X_C_EVT_DATA,
} ble_a6x_c_evt_type_t;

/**@brief Structure containing the handles */
typedef struct
{
    uint16_t cccd_handle;  /**< Handle of the CCCD of the characteristic. */
    uint16_t ra_handle;       /**< Handle of the characteristic as provided by the SoftDevice. */
    uint16_t rc_handle;       /**< Handle of the characteristic as provided by the SoftDevice. */
} a6x_db_t;

/**@brief SRV Event structure. */
typedef struct
{
    ble_a6x_c_evt_type_t evt_type;    /**< Type of the event. */
    uint16_t             conn_handle; /**< Connection handle on which the Heart Rate service was discovered on the peer device..*/
    union
    {
        a6x_db_t  peer_db;            /**< KOMOOT related handles found on the peer device.. This will be filled if the evt_type is @ref BLE_KOMOOT_C_EVT_DISCOVERY_COMPLETE.*/
        uint8_t   a6x_data[4];                /**< KOMOOT measurement received. This will be filled if the evt_type is @ref BLE_KOMOOT_C_EVT_NOTIFICATION. */
    } params;
} ble_a6x_c_evt_t;

typedef void (*ble_a6x_evt_handler_t) (ble_a6x_srv_t * p_ble_a6x_c, ble_a6x_c_evt_t * p_evt);

/** @brief Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_a6x_evt_handler_t     event_handler;
//    ble_a6x_write_handler_t   write_handler; /**< Event handler to be called when the LED Characteristic is written. */
    ble_srv_error_handler_t   error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t            * p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
} ble_a6x_c_init_t;

typedef enum {
    ble_a6x_app_update_shutter_released = 0x106u,
    ble_a6x_app_update_focus_transition = 0x107u,
    ble_a6x_app_update_focus_hold       = 0x108u,
    ble_a6x_app_update_shutter_pressed  = 0x109u,
} ble_a6x_app_update_t;

/**@brief Service structure. This structure contains various status information for the service. */
struct ble_a6x_srv_s
{
    uint16_t                    conn_handle;          /**< Peripheral handle */
    uint16_t                    service_handle;       /**< Handle of the Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    rc_handles;           /**< Handles related to the RC Characteristic. */
    ble_gatts_char_handles_t    ra_handles;           /**< Handles related to the RA Characteristic. */
    uint8_t                     uuid_type;            /**< UUID type for the LED Button Service. */
//    ble_a6x_write_handler_t     write_handler;    /**< Event handler to be called when the LED Characteristic is written. */

    a6x_db_t                    peer_db;      /**< Handles related to KOMOOT on the peer*/
    ble_a6x_evt_handler_t       evt_handler;      /**< Application event handler to be called when there is an event related to the heart rate service. */
    ble_srv_error_handler_t     error_handler;  /**< Function to be called in case of an error. */
    nrf_ble_gq_t               *p_gatt_queue;   /**< Pointer to BLE GATT Queue instance. */
};

#ifdef __cplusplus
extern "C" {
#endif

/**@brief Function for initializing the Service.
 *
 * @param[out] p_cfg      LED Button Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_cfg_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_a6x_c_init(ble_a6x_srv_t * p_cfg, const ble_a6x_c_init_t * p_cfg_init);


/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the Service.
 *
 * @param[in] p_ble_evt  Event received from the BLE stack.
 * @param[in] p_context  LED Button Service structure.
 */
void ble_a6x_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


/**@brief Function for sending a command
 *
 * @param[in] p_cfg         LED Button Service structure.
 * @param[in] cmd_update    Command
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_a6x_c_update(ble_a6x_srv_t * p_cfg, ble_a6x_app_update_t cmd_update);

void ble_a6x_c_on_db_disc_evt(ble_a6x_srv_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt);

uint32_t ble_a6x_c_notif_enable(ble_a6x_srv_t * p_cfg);

uint32_t ble_a6x_c_handles_assign(ble_a6x_srv_t *   p_ble_rscs_c,
                                   uint16_t          conn_handle,
                                   a6x_db_t *        p_peer_handles);

#ifdef __cplusplus
}
#endif

#endif //STARTRACKER_BLE_A6X_H
