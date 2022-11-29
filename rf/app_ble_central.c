//
// Created by vgol on 29/11/2022.
//

#define NRF_BLE_GQ_QUEUE_SIZE 10

#include <ble_nus_c.h>
#include <peer_manager.h>
#include <nrf_ble_qwr.h>
#include "app_ble_central.h"
#include "nordic_common.h"
#include "app_error.h"
#include "bsp_btn_ble.h"
#include "ble_db_discovery.h"
#include "ble_conn_params.h"
#include "nrf_ble_scan.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_gq.h"
#include "ble_a6x.h"

#include "segger_wrapper.h"

BLE_A6X_C_DEF(m_ble_a6x_c);
BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */


static uint16_t m_conn_handle_a6x_c = BLE_CONN_HANDLE_INVALID;     /**< Connection handle for the RSC central application */

/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    ble_a6x_c_on_db_disc_evt(&m_ble_a6x_c, p_evt);
}

/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to start scanning. */
void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
}

void on_ble_central_evt(ble_evt_t const * p_ble_evt) {

    ble_a6x_c_on_ble_evt(p_ble_evt, &m_ble_a6x_c);

    ret_code_t            err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, check which peripheral is connected (HR or RSC), initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Central connected");

//            m_retry_db_disc = false;
//            m_pending_db_disc_conn = p_gap_evt->conn_handle;

            err_code = ble_db_discovery_start(&m_db_disc[p_gap_evt->conn_handle],
                                              p_gap_evt->conn_handle);
            if (err_code == NRF_ERROR_BUSY)
            {
                LOG_WARNING("ble_db_discovery_start() returned busy, will retry later.");
//                m_retry_db_disc = true;
            }
            else
            {
                APP_ERROR_CHECK(err_code);
            }

            // Assign connection handle to the QWR module.
            extern void multi_qwr_conn_handle_assign(uint16_t conn_handle);
            multi_qwr_conn_handle_assign(p_gap_evt->conn_handle);

        } break; // BLE_GAP_EVT_CONNECTED

            // Upon disconnection, reset the connection handle of the peer that disconnected,
            // update the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Central disconnected");

            // Start scanning.
            scan_start();

            m_conn_handle_a6x_c = BLE_CONN_HANDLE_INVALID;

        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
        {
            // No timeout for scanning is specified, so only connection attemps can timeout.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
        } break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        {
            // Accept parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
                    {
                            .rx_phys = BLE_GAP_PHY_AUTO,
                            .tx_phys = BLE_GAP_PHY_AUTO,
                    };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling Scanning Module events.
 */
static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
            ble_gap_evt_connected_t const * p_connected =
                    p_scan_evt->params.connected.p_connected;
            // Scan is automatically stopped by the connection.
            NRF_LOG_INFO("Connecting to target %02x%02x%02x%02x%02x%02x",
                         p_connected->peer_addr.addr[0],
                         p_connected->peer_addr.addr[1],
                         p_connected->peer_addr.addr[2],
                         p_connected->peer_addr.addr[3],
                         p_connected->peer_addr.addr[4],
                         p_connected->peer_addr.addr[5]
            );
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("Scan timed out.");
            scan_start();
        } break;

        default:
            break;
    }
}

/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          err_code;
    nrf_ble_scan_init_t init_scan;

    memset(&init_scan, 0, sizeof(init_scan));

    /**@brief NUS UUID. */
    static ble_uuid_t m_nus_uuid =
    {
        .uuid = A6X_UUID_SRV,
    };
    m_nus_uuid.type = m_ble_a6x_c.uuid_type;

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    err_code = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(err_code);

#if 0
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);
#else
    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, "SONYA6600_VINCE");
    APP_ERROR_CHECK(err_code);
#endif

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the LED Button Service client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void _service_c_error_handler(uint32_t nrf_error)
{
    if (nrf_error == NRF_ERROR_RESOURCES) {
        return;
    }
    APP_ERROR_HANDLER(nrf_error);
}

static volatile bool m_focus_acquired = false;
static volatile bool m_shutter_triggered = false;

static void _a6x_c_evt_handler(ble_a6x_srv_t * p_ble_a6x_c, ble_a6x_c_evt_t * p_evt) {

    LOG_INFO("A6X event: 0x%X\r\n", p_evt->evt_type);

    switch (p_evt->evt_type) {
        case BLE_A6X_C_EVT_DISCOVERY_COMPLETE: {
            if (m_conn_handle_a6x_c == BLE_CONN_HANDLE_INVALID) {
                ret_code_t err_code;

                m_conn_handle_a6x_c = p_evt->conn_handle;
                NRF_LOG_INFO("A6X discovered on conn_handle 0x%x",
                             m_conn_handle_a6x_c);

                err_code = ble_a6x_c_handles_assign(p_ble_a6x_c,
                                                    m_conn_handle_a6x_c,
                                                    &p_evt->params.peer_db);
                APP_ERROR_CHECK(err_code);

                // Initiate bonding.
                err_code = pm_conn_secure(m_conn_handle_a6x_c, false);
                if (err_code != NRF_ERROR_BUSY) {
                    APP_ERROR_CHECK(err_code);
                }

                // Enable notifications.
                err_code = ble_a6x_c_notif_enable(p_ble_a6x_c);
                APP_ERROR_CHECK(err_code);

                app_ble_central__take_pic(true);
            }
        }
            break; // BLE_RSCS_C_EVT_DISCOVERY_COMPLETE:

        case BLE_A6X_C_EVT_DATA: {
            if (p_evt->params.a6x_data.first == 0x02) {
                switch (p_evt->params.a6x_data.second) {
                    case 0x3F: {
                        uint8_t _focusStatus = p_evt->params.a6x_data.third;

                        if (_focusStatus == 0x20) {
                            m_focus_acquired = true;
//                            rs->set(Status::FOCUS_ACQUIRED);
                        } else {
                            m_focus_acquired = false;
//                            rs->set(Status::READY);
                        }

                    } break;

                    case 0xA0: {
                        uint8_t _shutterStatus = p_evt->params.a6x_data.third;

                        if (_shutterStatus == 0x20) {
//                            rs->set(Status::SHUTTER);
                            m_shutter_triggered = true;
                        } else {
//                            rs->set(Status::READY);
                            m_shutter_triggered = false;
                        }

                    } break;

                    case 0xD5:
                    {   uint8_t _recordingStatus = p_evt->params.a6x_data.third;
                    } break;
                }
            }
        } break; // BLE_A6X_C_EVT_DATA

        default:
            // No implementation needed.
            break;
    }
}

static void _a6x_write_handler_t(uint16_t conn_handle, const uint8_t *p_data, uint16_t len) {

    (void)conn_handle;

    NRF_LOG_INFO("A6X char write len=2");
    NRF_LOG_RAW_HEXDUMP_INFO(p_data, len);
}

/**@brief Function for handling Queued Write module errors.
 *
 * @details A pointer to this function is passed to each service that may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code that contains information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void a6x_c_init(void)
{
    ret_code_t       err_code;
    ble_a6x_c_init_t init_a6x;

    init_a6x.p_gatt_queue  = &m_ble_gatt_queue;
//    init_a6x.write_handler = _a6x_write_handler_t;
    init_a6x.event_handler = _a6x_c_evt_handler;
    init_a6x.error_handler = _service_c_error_handler;

    err_code = ble_a6x_c_init(&m_ble_a6x_c, &init_a6x);
    APP_ERROR_CHECK(err_code);
}

void app_ble_central__take_pic(bool start) {

    static int _state = 0;

    // on / off
    if (start) {
        if (!_state) {
            _state = 1;
        }
        return; // HAVE TO RETURN HERE
    } else if (!ble_a6x_c_is_connected(&m_ble_a6x_c) || m_conn_handle_a6x_c == BLE_CONN_HANDLE_INVALID) {
        _state = 0;
    }

    // focus
    if (m_focus_acquired && _state == 1) {
        _state = 2;
    }
    // shutter
    if (m_shutter_triggered && _state == 3) {
        _state = 4;
    }

    uint32_t err_code = NRF_SUCCESS;
    switch (_state) {
        case 0u: // nothing
            break;

        case 1u:
        {
            m_focus_acquired = false;
            m_shutter_triggered = false;

            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x106);
            APP_ERROR_CHECK(err_code);

            w_task_delay(100);

            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x108);
            APP_ERROR_CHECK(err_code);

            w_task_delay(100);

            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x107);
            APP_ERROR_CHECK(err_code);
        } break;

        case 2u:
        {
//            err_code = ble_a6x_c_update(&m_ble_a6x_c, ble_a6x_app_update_focus_hold);
//            APP_ERROR_CHECK(err_code);

            _state = 3u;
        } break;

        case 3u:
        {
            if (!m_focus_acquired) {
                _state = 1u;
                return;
            }
            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x109);
            APP_ERROR_CHECK(err_code);

            w_task_delay(2000);

            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x108);
            APP_ERROR_CHECK(err_code);

            w_task_delay(100);

            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x109); // releases shutter : end BULB with that one
            APP_ERROR_CHECK(err_code);

            w_task_delay(100);

            err_code = ble_a6x_c_update(&m_ble_a6x_c, 0x106);
            APP_ERROR_CHECK(err_code);
        } break;

        case 4u:
        {
            NRF_LOG_INFO("Picture taken !");

            m_focus_acquired = false;
            m_shutter_triggered = false;
        } break;

        default:
            break;
    }

}

void app_ble_central__init(void)
{
    db_discovery_init();
    a6x_c_init();
    scan_init();
    scan_start();
}