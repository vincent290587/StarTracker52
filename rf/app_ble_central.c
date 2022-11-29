//
// Created by vgol on 29/11/2022.
//

#define NRF_BLE_GQ_QUEUE_SIZE 10

#include <ble_nus_c.h>
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
BLE_DB_DISCOVERY_DEF(m_db_disc);                                        /**< Database discovery module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

#define APP_BLE_CONN_CFG_TAG    1                                       /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */

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
static void scan_start(void)
{
    ret_code_t ret;

    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);

    ret = bsp_indication_set(BSP_INDICATE_SCANNING);
    APP_ERROR_CHECK(ret);
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

    err_code = nrf_ble_scan_filter_set(&m_scan, SCAN_UUID_FILTER, &m_nus_uuid);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(err_code);
}


static void _a6x_write_handler_t(uint16_t conn_handle, const uint8_t *p_data, uint16_t len) {

    (void)conn_handle;

    NRF_LOG_INFO("A6X char write len=2");
    NRF_LOG_RAW_HEXDUMP_INFO(p_data, len);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void a6x_c_init(void)
{
    ret_code_t       err_code;
    ble_a6x_c_init_t init_a6x;

    init_a6x.write_handler = _a6x_write_handler_t;

    err_code = ble_a6x_c_init(&m_ble_a6x_c, &init_a6x);
    APP_ERROR_CHECK(err_code);
}

void app_ble_central__init(void)
{
    db_discovery_init();
    a6x_c_init();
    scan_init();
    scan_start();
}