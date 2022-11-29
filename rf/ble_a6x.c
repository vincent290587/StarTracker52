//
// Created by vgol on 29/11/2022.
//

#include <ble_db_discovery.h>
#include "ble_a6x.h"

#include "sdk_common.h"
#include "ble_srv_common.h"
#include "segger_wrapper.h"

/**@brief Function for handling the Write event.
 *
 * @param[in] p_cfg      Service structure pointer
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_a6x_srv_t * p_cfg, ble_evt_t const * p_ble_evt)
{
    ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if (   (p_evt_write->handle == p_cfg->ra_handles.value_handle)
           && (p_cfg->write_handler != NULL))
    {
        p_cfg->write_handler(p_ble_evt->evt.gap_evt.conn_handle, p_evt_write->data, p_evt_write->len);
    }
}

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect(ble_a6x_srv_t * p_cfg, ble_evt_t const *p_ble_evt) {

    if (p_cfg) {
        p_cfg->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    }

}

/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_cus       Custom Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect(ble_a6x_srv_t * p_cfg, ble_evt_t const *p_ble_evt) {

    UNUSED_PARAMETER(p_ble_evt);

    if (p_cfg) {
        p_cfg->conn_handle = BLE_CONN_HANDLE_INVALID;
    }
}


void ble_a6x_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_a6x_srv_t * p_cfg = (ble_a6x_srv_t *)p_context;

    NRF_LOG_DEBUG("ble_a6x_on_ble_evt %u", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GATTS_EVT_WRITE:
            on_write(p_cfg, p_ble_evt);
            break;

        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_cfg, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_cfg, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


uint32_t ble_a6x_c_init(ble_a6x_srv_t * p_cfg, const ble_a6x_c_init_t * p_cfg_init)
{
    uint32_t              err_code;
    ble_uuid_t            ble_uuid;
    ble_uuid128_t base_uuid = { A6X_UUID_BASE };

    VERIFY_PARAM_NOT_NULL(p_cfg);
    VERIFY_PARAM_NOT_NULL(p_cfg_init);

    p_cfg->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Initialize service structure.
    p_cfg->write_handler = p_cfg_init->write_handler;

    // Add service.
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cfg->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cfg->uuid_type;
    ble_uuid.uuid = A6X_UUID_SRV;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cfg->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add RC characteristic.
    {
        ble_add_char_params_t add_char_params;
        memset(&add_char_params, 0, sizeof(add_char_params));
        add_char_params.uuid = A6X_UUID_RC;
        add_char_params.uuid_type = p_cfg->uuid_type;
//        add_char_params.p_init_value = (uint8_t *) &p_cfg_init->app_init;
        add_char_params.init_len = 2;
        add_char_params.max_len = 2;
//        add_char_params.char_props.read = 1;
        add_char_params.char_props.write = 1;
//    add_char_params.char_props.notify = 1;

        add_char_params.read_access = SEC_OPEN;
        add_char_params.write_access = SEC_OPEN;
//    add_char_params.cccd_write_access = SEC_OPEN;

        err_code = characteristic_add(p_cfg->service_handle,
                                      &add_char_params,
                                      &p_cfg->rc_handles);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }
    }

    // Add RA characteristic.
    {
        ble_add_char_params_t add_char_params;
        memset(&add_char_params, 0, sizeof(add_char_params));
        add_char_params.uuid = A6X_UUID_RA;
        add_char_params.uuid_type = p_cfg->uuid_type;
//        add_char_params.p_init_value = (uint8_t *) &p_cfg_init->app_init;
        add_char_params.init_len = 3;
        add_char_params.max_len = 8;
        add_char_params.char_props.read = 1;
//        add_char_params.char_props.write = 1;
        add_char_params.char_props.notify = 1;

        add_char_params.read_access = SEC_OPEN;
        add_char_params.write_access = SEC_OPEN;
//    add_char_params.cccd_write_access = SEC_OPEN;

        err_code = characteristic_add(p_cfg->service_handle,
                                      &add_char_params,
                                      &p_cfg->ra_handles);
        if (err_code != NRF_SUCCESS) {
            return err_code;
        }
    }

    return err_code;
}

///**@brief Function for creating a message for writing to the CCCD. */
//static uint32_t cccd_configure(ble_a6x_srv_t * p_ble_srv_c, bool notification_enable)
//{
//    nrf_ble_gq_req_t cccd_req;
//    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
//    uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;
//
//    memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));
//
//    cccd[0] = LSB_16(cccd_val);
//    cccd[1] = MSB_16(cccd_val);
//
//    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
//    cccd_req.error_handler.cb            = gatt_error_handler;
//    cccd_req.error_handler.p_ctx         = p_ble_nus_c;
//    cccd_req.params.gattc_write.handle   = p_ble_nus_c->handles.nus_tx_cccd_handle;
//    cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
//    cccd_req.params.gattc_write.offset   = 0;
//    cccd_req.params.gattc_write.p_value  = cccd;
//    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
//    cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;
//
//    return nrf_ble_gq_item_add(p_ble_srv_c->p_gatt_queue, &cccd_req, p_ble_srv_c->conn_handle);
//}
//
//uint32_t ble_a6x_c_ra_notif_enable(ble_a6x_srv_t * p_ble_srv_c)
//{
//    VERIFY_PARAM_NOT_NULL(p_ble_srv_c);
//
//    if ( (p_ble_srv_c->conn_handle == BLE_CONN_HANDLE_INVALID)
//         ||(p_ble_srv_c->handles.nus_tx_cccd_handle == BLE_GATT_HANDLE_INVALID)
//            )
//    {
//        return NRF_ERROR_INVALID_STATE;
//    }
//    return cccd_configure(p_ble_srv_c, true);
//}

uint32_t ble_a6x_c_update(ble_a6x_srv_t * p_cfg, ble_a6x_app_update_t cmd_update)
{
    uint32_t err_code = NRF_SUCCESS;

    if (!p_cfg) {
        return 0;
    }

    ble_gatts_value_t gatts_value;

    // Initialize value struct.
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(ble_a6x_app_update_t);
    gatts_value.offset  = 0;
    gatts_value.p_value = (uint8_t*)cmd_update;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cfg->conn_handle, p_cfg->rc_handles.value_handle, &gatts_value);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }

//	if (p_cfg->conn_handle != BLE_CONN_HANDLE_INVALID) {
//
//		ble_gatts_hvx_params_t hvx_params;
//		memset(&hvx_params, 0, sizeof(hvx_params));
//		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
//		hvx_params.handle = p_cfg->button_char_handles.value_handle;
//		hvx_params.offset = gatts_value.offset;
//		hvx_params.p_len = &gatts_value.len;
//		hvx_params.p_data = gatts_value.p_value;
//		// BLE_ERROR_GATTS_SYS_ATTR_MISSING
//		err_code = sd_ble_gatts_hvx(p_cfg->conn_handle, &hvx_params);
//		NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
//	}

    return err_code;
}


void ble_a6x_c_on_db_disc_evt(ble_a6x_srv_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt)
{
//    ble_nus_c_evt_t nus_c_evt;
//    memset(&nus_c_evt,0,sizeof(ble_nus_c_evt_t));

    ble_gatt_db_char_t * p_chars = p_evt->params.discovered_db.charateristics;

    // Check if the service was discovered.
    if (    (p_evt->evt_type == BLE_DB_DISCOVERY_COMPLETE)
            &&  (p_evt->params.discovered_db.srv_uuid.uuid == A6X_UUID_SRV)
            &&  (p_evt->params.discovered_db.srv_uuid.type == p_ble_nus_c->uuid_type))
    {
        for (uint32_t i = 0; i < p_evt->params.discovered_db.char_count; i++)
        {
            switch (p_chars[i].characteristic.uuid.uuid)
            {
                case A6X_UUID_RA:
                    //nus_c_evt.handles.nus_rx_handle = p_chars[i].characteristic.handle_value;
                    p_ble_nus_c->ra_handles.value_handle = p_chars[i].characteristic.handle_value;
                    break;

                case A6X_UUID_RC:
                    //nus_c_evt.handles.nus_tx_handle = p_chars[i].characteristic.handle_value;
                    //nus_c_evt.handles.nus_tx_cccd_handle = p_chars[i].cccd_handle;
                    p_ble_nus_c->rc_handles.value_handle = p_chars[i].characteristic.handle_value;
                    p_ble_nus_c->rc_handles.cccd_handle = p_chars[i].cccd_handle;
                    break;

                default:
                    break;
            }
        }

        p_ble_nus_c->conn_handle = p_evt->conn_handle;

        NRF_LOG_INFO("A6X discovery complete");
    }
}
