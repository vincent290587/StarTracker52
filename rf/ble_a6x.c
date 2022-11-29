//
// Created by vgol on 29/11/2022.
//

#include <ble_db_discovery.h>
#include "ble_a6x.h"

#include "sdk_common.h"
#include "ble_srv_common.h"
#include "segger_wrapper.h"

#ifdef VERIFY_SUCCESS
#undef VERIFY_SUCCESS
#endif

#define VERIFY_SUCCESS(statement)                       \
do                                                      \
{                                                       \
    uint32_t _err_code = (uint32_t) (statement);        \
    if (_err_code != NRF_SUCCESS)                       \
    {                                                   \
        NRF_LOG_ERROR("Error %lu line %lu",_err_code, __LINE__); \
        return _err_code;                               \
    }                                                   \
} while(0)


/**@brief     Function for handling write response events.
 *
 * @param[in] p_ble_a6x   Pointer to the Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_read_rsp(ble_a6x_srv_t * p_ble_a6x, const ble_evt_t * p_ble_evt)
{
    const ble_gattc_evt_read_rsp_t * p_response;
    uint32_t        err_code = NRF_SUCCESS;

    // Check if the event is on the same connection as this cts instance
    if (p_ble_a6x->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    p_response = &p_ble_evt->evt.gattc_evt.params.read_rsp;

    if (p_ble_evt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
        NRF_LOG_INFO("on_read_rsp len: %u", p_response->len);
        NRF_LOG_HEXDUMP_INFO(p_response->data, p_response->len);

        if (err_code == NRF_SUCCESS)
        {
//            uint32_t        index = 0;
//            ble_a6x_c_evt_t ble_c_evt;
//
//            // The data length was invalid, decoding was not completed.
//            ble_komoot_c_evt.evt_type = BLE_KOMOOT_C_EVT_KOMOOT_NAVIGATION;
//
//            ble_komoot_c_evt.params.komoot.identifier = uint32_decode(&(p_response->data[index]));
//            index += sizeof(uint32_t);
//
//            ble_komoot_c_evt.params.komoot.direction  = p_response->data[index++];
//
//            ble_komoot_c_evt.params.komoot.distance   = uint32_decode(&(p_response->data[index]));
//            index += sizeof(uint32_t);
//
//            p_ble_komoot_c->evt_handler(p_ble_komoot_c, &ble_komoot_c_evt);
        }
    }
}

/**@brief Function for handling Handle Value Notification received from the SoftDevice.
 *
 * @details This function uses the Handle Value Notification received from the SoftDevice
 *          and checks whether it is a notification of Button state from the peer. If
 *          it is, this function decodes the state of the button and sends it to the
 *          application.
 *
 * @param[in] p_ble_lbs_c Pointer to the Led Button Client structure.
 * @param[in] p_ble_evt   Pointer to the BLE event received.
 */
static void on_hvx(ble_a6x_srv_t * p_cfg, ble_evt_t const * p_ble_evt)
{
    // Check if the event is on the link for this instance.
    if (p_cfg->conn_handle != p_ble_evt->evt.gattc_evt.conn_handle)
    {
        return;
    }

    if (p_ble_evt->evt.gattc_evt.params.hvx.handle == p_cfg->peer_db.ra_handle)
    {
        NRF_LOG_DEBUG("received HVX on handle 0x%x, ra_handle 0x%x\r\n",
                     p_ble_evt->evt.gattc_evt.params.hvx.handle,
                     p_cfg->peer_db.ra_handle);

        NRF_LOG_INFO("on_hvx len: %u", p_ble_evt->evt.gattc_evt.params.hvx.len);
        NRF_LOG_HEXDUMP_INFO(p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);

        ble_a6x_c_evt_t ble_c_evt;

        // The data length was invalid, decoding was not completed.
        ble_c_evt.evt_type = BLE_A6X_C_EVT_DATA;

        if (3u == p_ble_evt->evt.gattc_evt.params.hvx.len) {

            ble_c_evt.params.a6x_data.first = p_ble_evt->evt.gattc_evt.params.hvx.data[0];
            ble_c_evt.params.a6x_data.second = p_ble_evt->evt.gattc_evt.params.hvx.data[1];
            ble_c_evt.params.a6x_data.third = p_ble_evt->evt.gattc_evt.params.hvx.data[2];

            p_cfg->evt_handler(p_cfg, &ble_c_evt);
        }

    } else {

        NRF_LOG_INFO("received HVX on handle 0x%x, rc_handle 0x%x\r\n",
                     p_ble_evt->evt.gattc_evt.params.hvx.handle,
                     p_cfg->peer_db.rc_handle);
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
        p_cfg->peer_db.cccd_handle = BLE_CONN_HANDLE_INVALID;
        p_cfg->peer_db.ra_handle = BLE_CONN_HANDLE_INVALID;
        p_cfg->peer_db.rc_handle = BLE_CONN_HANDLE_INVALID;
    }
}


void ble_a6x_c_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_a6x_srv_t * p_cfg = (ble_a6x_srv_t *)p_context;

    NRF_LOG_DEBUG("ble_a6x_on_ble_evt %u", p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
//        case BLE_GATTS_EVT_WRITE:
//            on_write(p_cfg, p_ble_evt);
//            break;

        case BLE_GATTC_EVT_HVX:
            on_hvx(p_cfg, p_ble_evt);
            break;

        case BLE_GATTC_EVT_READ_RSP:
            on_read_rsp(p_cfg, p_ble_evt);
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
    VERIFY_PARAM_NOT_NULL(p_cfg_init->p_gatt_queue);

    p_cfg->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service.
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_cfg->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_cfg->uuid_type;
    ble_uuid.uuid = A6X_UUID_SRV;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_cfg->service_handle);
    VERIFY_SUCCESS(err_code);

    // Initialize service structure.
    p_cfg->evt_handler   = p_cfg_init->event_handler;
    p_cfg->error_handler = p_cfg_init->error_handler;

    p_cfg->p_gatt_queue           = p_cfg_init->p_gatt_queue;
    p_cfg->conn_handle            = BLE_CONN_HANDLE_INVALID;
    p_cfg->peer_db.cccd_handle    = BLE_GATT_HANDLE_INVALID;
    p_cfg->peer_db.ra_handle      = BLE_GATT_HANDLE_INVALID;
    p_cfg->peer_db.rc_handle      = BLE_GATT_HANDLE_INVALID;

    return ble_db_discovery_evt_register(&ble_uuid);
}

/**@brief Function for intercepting the errors of GATTC and the BLE GATT Queue.
 *
 * @param[in] nrf_error   Error code.
 * @param[in] p_ctx       Parameter from the event handler.
 * @param[in] conn_handle Connection handle.
 */
static void gatt_error_handler(uint32_t   nrf_error,
                               void     * p_ctx,
                               uint16_t   conn_handle)
{
    ble_a6x_srv_t * p_ble_srv_c = (ble_a6x_srv_t *)p_ctx;

    NRF_LOG_DEBUG("A GATT Client error has occurred on conn_handle: 0X%X", conn_handle);

    if (p_ble_srv_c->error_handler != NULL)
    {
        p_ble_srv_c->error_handler(nrf_error);
    }
}

/**@brief Function for creating a message for writing to the CCCD. */
static uint32_t cccd_configure(ble_a6x_srv_t * p_ble_srv_c, bool notification_enable)
{
    nrf_ble_gq_req_t cccd_req;
    uint8_t          cccd[BLE_CCCD_VALUE_LEN];
    uint16_t         cccd_val = notification_enable ? BLE_GATT_HVX_NOTIFICATION : 0;

    memset(&cccd_req, 0, sizeof(nrf_ble_gq_req_t));

    cccd[0] = LSB_16(cccd_val);
    cccd[1] = MSB_16(cccd_val);

    cccd_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    cccd_req.error_handler.cb            = gatt_error_handler;
    cccd_req.error_handler.p_ctx         = p_ble_srv_c;
    cccd_req.params.gattc_write.handle   = p_ble_srv_c->peer_db.cccd_handle;
    cccd_req.params.gattc_write.len      = BLE_CCCD_VALUE_LEN;
    cccd_req.params.gattc_write.offset   = 0;
    cccd_req.params.gattc_write.p_value  = cccd;
    cccd_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_REQ;
    cccd_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    return nrf_ble_gq_item_add(p_ble_srv_c->p_gatt_queue, &cccd_req, p_ble_srv_c->conn_handle);
}

uint32_t ble_a6x_c_notif_enable(ble_a6x_srv_t * p_ble_srv_c)
{
    VERIFY_PARAM_NOT_NULL(p_ble_srv_c);

    if ( (p_ble_srv_c->conn_handle == BLE_CONN_HANDLE_INVALID)
         || (p_ble_srv_c->peer_db.cccd_handle == BLE_GATT_HANDLE_INVALID))
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return cccd_configure(p_ble_srv_c, true);
}

uint32_t ble_a6x_c_update(ble_a6x_srv_t * p_cfg, ble_a6x_app_update_t cmd_update)
{
    uint32_t err_code = NRF_SUCCESS;

    if (!p_cfg) {
        return 0;
    }

    uint8_t cmd_value[2] = {
            [0] = (cmd_update & 0xff00) >> 8u,
            [1] = cmd_update & 0x00ff,
    };

    NRF_LOG_INFO("ble_a6x_c_update %04X (handle= %u)",cmd_update, p_cfg->peer_db.rc_handle);

#if 0
    // Initialize value struct.
    ble_gatts_value_t gatts_value;
    memset(&gatts_value, 0, sizeof(gatts_value));

    gatts_value.len     = sizeof(cmd_value);
    gatts_value.offset  = 0;
    gatts_value.p_value = cmd_value;

    // Update database.
    err_code = sd_ble_gatts_value_set(p_cfg->conn_handle, p_cfg->peer_db.rc_handle, &gatts_value);
    if (err_code != NRF_SUCCESS) {
        return err_code;
    }
#else
    nrf_ble_gq_req_t write_req;

    memset(&write_req, 0, sizeof(nrf_ble_gq_req_t));

    write_req.type                        = NRF_BLE_GQ_REQ_GATTC_WRITE;
    write_req.error_handler.cb            = gatt_error_handler;
    write_req.error_handler.p_ctx         = p_cfg;
    write_req.params.gattc_write.handle   = p_cfg->peer_db.rc_handle;
    write_req.params.gattc_write.len      = sizeof(cmd_value);
    write_req.params.gattc_write.p_value  = cmd_value;
    write_req.params.gattc_write.offset   = 0;
    write_req.params.gattc_write.write_op = BLE_GATT_OP_WRITE_CMD;
    write_req.params.gattc_write.flags    = BLE_GATT_EXEC_WRITE_FLAG_PREPARED_WRITE;

    err_code = nrf_ble_gq_item_add(p_cfg->p_gatt_queue, &write_req, p_cfg->conn_handle);
    APP_ERROR_CHECK(err_code);
#endif

#if 0
	if (p_cfg->conn_handle != BLE_CONN_HANDLE_INVALID) {

        uint16_t cmdLen = sizeof(cmd_value);
		ble_gatts_hvx_params_t hvx_params;
		memset(&hvx_params, 0, sizeof(hvx_params));
		hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.handle = p_cfg->peer_db.rc_handle;
		hvx_params.offset = 0;
		hvx_params.p_len = &cmdLen;
		hvx_params.p_data = cmd_value;
		// BLE_ERROR_GATTS_SYS_ATTR_MISSING
		err_code = sd_ble_gatts_hvx(p_cfg->conn_handle, &hvx_params);
		NRF_LOG_INFO("sd_ble_gatts_hvx result: %x. \r\n", err_code);
	}
#endif

    return err_code;
}


void ble_a6x_c_on_db_disc_evt(ble_a6x_srv_t * p_ble_nus_c, ble_db_discovery_evt_t * p_evt)
{
    ble_a6x_c_evt_t evt;
    memset(&evt, 0, sizeof(ble_a6x_c_evt_t));

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
                    evt.params.peer_db.ra_handle = p_chars[i].characteristic.handle_value;
                    NRF_LOG_INFO("A6X RA handle: %u", evt.params.peer_db.ra_handle);
                    evt.params.peer_db.cccd_handle = p_chars[i].cccd_handle;
                    NRF_LOG_INFO("A6X CCCD handle: %u", evt.params.peer_db.cccd_handle);
                    break;

                case A6X_UUID_RC:
                    evt.params.peer_db.rc_handle = p_chars[i].characteristic.handle_value;
                    NRF_LOG_INFO("A6X RC handle: %u", evt.params.peer_db.rc_handle);
                    break;

                default:
                    break;
            }
        }

        //If the instance has been assigned prior to db_discovery, assign the db_handles
        if (p_ble_nus_c->conn_handle != BLE_CONN_HANDLE_INVALID)
        {
            if ((p_ble_nus_c->peer_db.cccd_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_nus_c->peer_db.rc_handle == BLE_GATT_HANDLE_INVALID)&&
                (p_ble_nus_c->peer_db.ra_handle == BLE_GATT_HANDLE_INVALID))
            {
                p_ble_nus_c->peer_db = evt.params.peer_db;
            }
        }

        p_ble_nus_c->evt_handler(p_ble_nus_c, &evt);

        NRF_LOG_INFO("A6X discovery complete");
    }
}

uint32_t ble_a6x_c_handles_assign(ble_a6x_srv_t *   p_cfg,
                                   uint16_t          conn_handle,
                                   a6x_db_t *        p_peer_handles)
{
    VERIFY_PARAM_NOT_NULL(p_cfg);

    p_cfg->conn_handle = conn_handle;
    if (p_peer_handles != NULL)
    {
        p_cfg->peer_db = *p_peer_handles;
    }

    return nrf_ble_gq_conn_handle_register(p_cfg->p_gatt_queue, conn_handle);
}

bool ble_a6x_c_is_connected(ble_a6x_srv_t * p_cfg)
{
    VERIFY_PARAM_NOT_NULL(p_cfg);

    //NRF_LOG_WARNING("ble_a6x_c_is_connected %u %u", p_cfg->conn_handle, p_cfg->peer_db.rc_handle);

    return (p_cfg->conn_handle != BLE_CONN_HANDLE_INVALID) && (p_cfg->peer_db.rc_handle != BLE_CONN_HANDLE_INVALID);
}

