/**
 * Copyright (c) 2016 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "ble_nus_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "packet_error_rate.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define APP_BLE_CONN_CFG_TAG 1  /**< Tag that refers to the BLE stack configuration set with @ref sd_ble_cfg_set. The default tag is @ref BLE_CONN_CFG_TAG_DEFAULT. */
#define APP_BLE_OBSERVER_PRIO 3 /**< BLE observer priority of the application. There is no need to modify this value. */

#define UART_TX_BUF_SIZE 256 /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256 /**< UART RX buffer size. */

#define NUS_SERVICE_UUID_TYPE BLE_UUID_TYPE_VENDOR_BEGIN /**< UUID type for the Nordic UART Service (vendor specific). */

#define ECHOBACK_BLE_UART_DATA 1 /**< Echo the UART data that is received over the Nordic UART Service (NUS) back to the sender. */

#define SCAN_INTERVAL 160 //0x00F0                         /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW 80    // 0x0050                         /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT 0    //0x0200                              /**< Timout when scanning. 0x0000 disables timeout. */

#ifdef NRF52840_XXAA
#define TX_POWER_LEVEL (8) /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
#else
#define TX_POWER_LEVEL (4) /**< TX Power Level value. This will be set both in the TX Power service, in the advertising data, and also used to set the radio transmit power. */
#endif

#define RSSI_SHOW_EVERY_COUNT 50
static uint8_t m_rssi_count = 0;


BLE_NUS_C_DEF(m_ble_nus_c);      /**< BLE Nordic UART Service (NUS) client instance. */
NRF_BLE_GATT_DEF(m_gatt);        /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_disc); /**< Database discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);        /**< Scanning Module instance. */

static uint16_t m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH; /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

static char const m_target_periph_name[] = "LRange811"; /**< Name of the device we try to connect to. This name is searched for in the scan report data*/
static uint8_t m_scan_buffer_data[BLE_GAP_SCAN_BUFFER_EXTENDED_MIN]; /**< buffer where advertising reports will be stored by the SoftDevice. */

typedef enum {DEVTYPE_NONE, DEVTYPE_BLINKY, DEVTYPE_THINGY} device_type_t;

#define MIN_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)     /**< Determines minimum connection interval in milliseconds. */
#define MAX_CONNECTION_INTERVAL     MSEC_TO_UNITS(7.5, UNIT_1_25_MS)    /**< Determines maximum connection interval in milliseconds. */
#define SLAVE_LATENCY               0                                   /**< Determines slave latency in terms of connection events. */
#define SUPERVISION_TIMEOUT         MSEC_TO_UNITS(10000, UNIT_10_MS)     /**< Determines supervision time-out in units of 10 milliseconds. */


/**@brief Connection parameters requested for connection. */
static ble_gap_conn_params_t const m_connection_param =
{
    (uint16_t)MIN_CONNECTION_INTERVAL,
    (uint16_t)MAX_CONNECTION_INTERVAL,
    (uint16_t)SLAVE_LATENCY,
    (uint16_t)SUPERVISION_TIMEOUT
};


typedef struct
{
    uint32_t dev_type;
    const char *dev_name;
    uint32_t phy;
}connected_device_info_t;

char m_device_name_being_connected_to[30];
connected_device_info_t m_device_being_connected_info = {DEVTYPE_NONE, m_device_name_being_connected_to, 0};

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */

/**@brief Pointer to the buffer where advertising reports will be stored by the SoftDevice. */
static ble_data_t m_scan_buffer =
{
    m_scan_buffer_data,
    BLE_GAP_SCAN_BUFFER_EXTENDED_MIN
};

/**@brief Scan parameters requested for scanning and connection. */
static ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .report_incomplete_evts = 0, 
    .extended = 1,
    .timeout           = SCAN_TIMEOUT,
    .scan_phys         = BLE_GAP_PHY_CODED,//BLE_GAP_PHY_1MBPS,
    .filter_policy     = BLE_GAP_SCAN_FP_ACCEPT_ALL,
    .channel_mask      = {0,0,0,0,0},
};


/**@brief NUS UUID. */
static ble_uuid_t const m_nus_uuid =
    {
        .uuid = BLE_UUID_NUS_SERVICE,
        .type = NUS_SERVICE_UUID_TYPE
    };

static void scan_start(void)
{
    ret_code_t ret;
    //(void) sd_ble_gap_scan_stop();
    {
        NRF_LOG_INFO("Scan start: Name - %s, phy - %s", (uint32_t)m_target_periph_name, "Coded");
        m_scan_buffer.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MIN;
        m_scan_params.scan_phys = BLE_GAP_PHY_CODED;
        m_scan_params.extended = 1;
        ret = sd_ble_gap_scan_start(&m_scan_params, &m_scan_buffer);
        if(ret == NRF_ERROR_INVALID_STATE)
        {
            NRF_LOG_INFO("scan start invalid state");
        }
        else 
            APP_ERROR_CHECK(ret);


        ret = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_SCAN_INIT, 0, TX_POWER_LEVEL);
        APP_ERROR_CHECK(ret);

        ret = bsp_indication_set(BSP_INDICATE_SCANNING);
        APP_ERROR_CHECK(ret);
        

    }
}

static void scan_stop(void)
{
    NRF_LOG_DEBUG("scan_stop()");
    uint32_t err_code = sd_ble_gap_scan_stop();
    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("scan_stop() failed with error code: %x", err_code);
    }  
    //scan_led_state_set(false, false);
}


/**
 * @brief Parses advertisement data, providing length and location of the field in case
 *        matching data is found.
 *
 * @param[in]  type       Type of data to be looked for in advertisement data.
 * @param[in]  p_advdata  Advertisement report length and pointer to report.
 * @param[out] p_typedata If data type requested is found in the data report, type data length and
 *                        pointer to data will be populated here.
 *
 * @retval NRF_SUCCESS if the data type is found in the report.
 * @retval NRF_ERROR_NOT_FOUND if the data type could not be found.
 */
static uint32_t adv_report_parse(uint8_t type, uint8_array_t * p_advdata, uint8_array_t * p_typedata)
{
    uint32_t  index = 0;
    uint8_t * p_data;

    p_data = p_advdata->p_data;

    while (index < p_advdata->size)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if (field_type == type)
        {
            p_typedata->p_data = &p_data[index + 2];
            p_typedata->size   = field_length - 1;
            return NRF_SUCCESS;
        }
        index += field_length + 1;
    }
    return NRF_ERROR_NOT_FOUND;
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_evt_t const * p_ble_evt)
{
    uint32_t      err_code;
    uint8_array_t adv_data;
    uint8_array_t dev_name;
    uint8_array_t service_uuid;

    //if (m_device_being_connected_info.dev_type == DEVTYPE_NONE)
    {
        // For readibility.
        ble_gap_evt_t  const * p_gap_evt  = &p_ble_evt->evt.gap_evt;
        ble_gap_addr_t const * peer_addr  = &p_gap_evt->params.adv_report.peer_addr;

        // Prepare advertisement report for parsing.
        adv_data.p_data = (uint8_t *)p_gap_evt->params.adv_report.data.p_data;
        adv_data.size   = p_gap_evt->params.adv_report.data.len;

        // Search for advertising names.
        bool found_name = false, found_blinky_name = false;
        err_code = adv_report_parse(BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME,
                                    &adv_data,
                                    &dev_name);
        if (err_code != NRF_SUCCESS)
        {
            // Look for the short local name if it was not found as complete.
            err_code = adv_report_parse(BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, &adv_data, &dev_name);
            if (err_code != NRF_SUCCESS)
            {
                // If we can't parse the data, then exit.
                //return;
            }
            else
            {
                found_name = true;
            }
        }
        else
        {
            found_name = true;
        }

        if (found_name)
        {
            //NRF_LOG_INFO("ADV name: %s", dev_name.p_data);
            // Check if the device name matches the BLINKY name filter
            if (strlen(m_target_periph_name) != 0)
            {
                NRF_LOG_INFO("Length %d, %d", strlen(m_target_periph_name), adv_data.size);
                //NRF_LOG_INFO("%d", adv_data.size);

                if (memcmp(m_target_periph_name, dev_name.p_data, strlen(m_target_periph_name)) == 0)
                {
                    NRF_LOG_INFO("BCDE");
                    // Copy the name to a static variable, to pass it on to the smart phone later
                    if(dev_name.size > strlen(m_target_periph_name))
                    {
                        memcpy(m_device_name_being_connected_to, 
                               dev_name.p_data + strlen(m_target_periph_name), 
                               dev_name.size - strlen(m_target_periph_name));
                        m_device_name_being_connected_to[dev_name.size - strlen(m_target_periph_name)] = 0;
                    }
                    m_device_being_connected_info.dev_type = DEVTYPE_BLINKY;
                }
            }
        }
        
        if (m_device_being_connected_info.dev_type != DEVTYPE_NONE)
        {
            m_device_being_connected_info.phy = m_scan_params.scan_phys;
            
            // Initiate connection.
            err_code = sd_ble_gap_connect(peer_addr, &m_scan_params, &m_connection_param, APP_BLE_CONN_CFG_TAG);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("Connection Request Failed, reason %d", err_code);
            }
        }
    }
    if(m_device_being_connected_info.dev_type == DEVTYPE_NONE)
    {
        // As of SoftDevice version 6.0 scanning must be started manually after each received packet
        err_code = sd_ble_gap_scan_start(NULL, &m_scan_buffer);
        if(err_code == NRF_ERROR_INVALID_STATE)
        {
            NRF_LOG_ERROR("scan_start invalid state!!");
        }
        else APP_ERROR_CHECK(err_code);
    }
    else
    {
        //scan_led_state_set(false, false);
    }
}

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t *p_evt)
{
    ble_nus_c_on_db_disc_evt(&m_ble_nus_c, p_evt);
}

/**@brief Function for handling characters received by the Nordic UART Service (NUS).
 *
 * @details This function takes a list of characters of length data_len and prints the characters out on UART.
 *          If @ref ECHOBACK_BLE_UART_DATA is set, the data is sent back to sender.
 */
static void ble_nus_chars_received_uart_print(uint8_t *p_data, uint16_t data_len)
{
    ret_code_t ret_val;

    NRF_LOG_DEBUG("Receiving data.");
    NRF_LOG_HEXDUMP_DEBUG(p_data, data_len);

    for (uint32_t i = 0; i < data_len; i++)
    {
        do
        {
            ret_val = app_uart_put(p_data[i]);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("app_uart_put failed for index 0x%04x.", i);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
    if (p_data[data_len - 1] == '\r')
    {
        while (app_uart_put('\n') == NRF_ERROR_BUSY)
            ;
    }
    if (ECHOBACK_BLE_UART_DATA)
    {
        // Send data back to the peripheral.
        do
        {
            ret_val = ble_nus_c_string_send(&m_ble_nus_c, p_data, data_len);
            if ((ret_val != NRF_SUCCESS) && (ret_val != NRF_ERROR_BUSY))
            {
                NRF_LOG_ERROR("Failed sending NUS message. Error 0x%x. ", ret_val);
                APP_ERROR_CHECK(ret_val);
            }
        } while (ret_val == NRF_ERROR_BUSY);
    }
}

/**@brief   Function for handling app_uart events.
 *
 * @details This function receives a single character from the app_uart module and appends it to
 *          a string. The string is sent over BLE when the last character received is a
 *          'new line' '\n' (hex 0x0A) or if the string reaches the maximum data length.
 */
void uart_event_handle(app_uart_evt_t *p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint16_t index = 0;
    uint32_t ret_val;

    switch (p_event->evt_type)
    {
    /**@snippet [Handling data from UART] */
    case APP_UART_DATA_READY:
        UNUSED_VARIABLE(app_uart_get(&data_array[index]));
        index++;

        if ((data_array[index - 1] == '\n') || (index >= (m_ble_nus_max_data_len)))
        {
            NRF_LOG_DEBUG("Ready to send data over BLE NUS");
            NRF_LOG_HEXDUMP_DEBUG(data_array, index);

            do
            {
                ret_val = ble_nus_c_string_send(&m_ble_nus_c, data_array, index);
                if ((ret_val != NRF_ERROR_INVALID_STATE) && (ret_val != NRF_ERROR_RESOURCES))
                {
                    APP_ERROR_CHECK(ret_val);
                }
            } while (ret_val == NRF_ERROR_RESOURCES);

            index = 0;
        }
        break;

    /**@snippet [Handling data from UART] */
    case APP_UART_COMMUNICATION_ERROR:
        NRF_LOG_ERROR("Communication error occurred while handling UART.");
        APP_ERROR_HANDLER(p_event->data.error_communication);
        break;

    case APP_UART_FIFO_ERROR:
        NRF_LOG_ERROR("Error occurred in FIFO module used by UART.");
        APP_ERROR_HANDLER(p_event->data.error_code);
        break;

    default:
        break;
    }
}

/**@brief Callback handling Nordic UART Service (NUS) client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS client handle. This identifies the NUS client.
 * @param[in]   p_ble_nus_evt Pointer to the NUS client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t *p_ble_nus_c, ble_nus_c_evt_t const *p_ble_nus_evt)
{
    ret_code_t err_code;

    switch (p_ble_nus_evt->evt_type)
    {
    case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
        NRF_LOG_INFO("Discovery complete.");
        err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
        APP_ERROR_CHECK(err_code);

        err_code = ble_nus_c_tx_notif_enable(p_ble_nus_c);
        APP_ERROR_CHECK(err_code);
        NRF_LOG_INFO("Connected to device with Nordic UART Service.");
        break;

    case BLE_NUS_C_EVT_NUS_TX_EVT:
        ble_nus_chars_received_uart_print(p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);
        break;

    case BLE_NUS_C_EVT_DISCONNECTED:
        NRF_LOG_INFO("Disconnected.");
//        scan_start();
        break;
    }
}

/**@snippet [Handling events from the ble_nus_c module] */

/**
 * @brief Function for handling shutdown events.
 *
 * @param[in]   event       Shutdown type.
 */
static bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    ret_code_t err_code;

    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    switch (event)
    {
    case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
        // Prepare wakeup buttons.
        err_code = bsp_btn_ble_sleep_mode_prepare();
        APP_ERROR_CHECK(err_code);
        break;

    default:
        break;
    }

    return true;
}

NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, APP_SHUTDOWN_HANDLER_PRIORITY);

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    ret_code_t err_code;
    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        err_code = ble_nus_c_handles_assign(&m_ble_nus_c, p_ble_evt->evt.gap_evt.conn_handle, NULL);
        APP_ERROR_CHECK(err_code);

        err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
        APP_ERROR_CHECK(err_code);

                m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;

        // start discovery of services. The NUS Client waits for a discovery result
        err_code = ble_db_discovery_start(&m_db_disc, p_ble_evt->evt.gap_evt.conn_handle);
        APP_ERROR_CHECK(err_code);

        err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, p_ble_evt->evt.gap_evt.conn_handle, TX_POWER_LEVEL);
        APP_ERROR_CHECK(err_code);

                packet_error_rate_reset_counter();
                packet_error_rate_detect_enable();

                err_code = sd_ble_gap_rssi_start(m_conn_handle, 0, 0);
                APP_ERROR_CHECK(err_code);

                printf("Connected\n");

        break;

    case BLE_GAP_EVT_DISCONNECTED:

        NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                     p_gap_evt->conn_handle,
                     p_gap_evt->params.disconnected.reason);

                printf("Disconnected. conn_handle: 0x%x, reason: 0x%x\n",
                       p_gap_evt->conn_handle,
                       p_gap_evt->params.disconnected.reason);

                m_conn_handle = BLE_CONN_HANDLE_INVALID;

                packet_error_rate_detect_disable();

        // Start scanning, in case the disconnect happened during service discovery
        scan_start();

        break;

    case BLE_GAP_EVT_TIMEOUT:
        if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
        {
            NRF_LOG_INFO("Connection Request timed out.");
        }
        break;

    case BLE_GAP_EVT_ADV_REPORT:
        on_adv_report(p_ble_evt);
        break;

    case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
        // Pairing not supported.
        err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
        APP_ERROR_CHECK(err_code);
        break;

    case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
        // Accepting parameters requested by peer.
        err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                &p_gap_evt->params.conn_param_update_request.conn_params);
        APP_ERROR_CHECK(err_code);
        break;

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
    }
    break;

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

        case BLE_GAP_EVT_RSSI_CHANGED:
                if (m_rssi_count % RSSI_SHOW_EVERY_COUNT == 0)
                {
                        packet_error_rate_timeout_handler();
                        NRF_LOG_INFO("RSSI = %d (dBm), PSR = %03d%%", p_ble_evt->evt.gap_evt.params.rssi_changed.rssi, get_packet_success_rate());
                        printf("RSSI = %d (dBm), Packet Success Rate = %03d%%\n", p_ble_evt->evt.gap_evt.params.rssi_changed.rssi, get_packet_success_rate());
                }
                m_rssi_count++;
                m_rssi_count = m_rssi_count % RSSI_SHOW_EVERY_COUNT;
                break;
    default:
        break;
    }
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_mode_set(NRF_POWER_MODE_LOWPWR);
    APP_ERROR_CHECK(err_code);

    err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("ATT MTU exchange completed.");

        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Ble NUS max data length set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    else if (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED)
    {
            NRF_LOG_INFO("Data length updated to %u bytes.", p_evt->params.data_length);
    }
}

/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_central_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, NRF_SDH_BLE_GAP_DATA_LENGTH);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling events from the BSP module.
 *
 * @param[in] event  Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
    case BSP_EVENT_SLEEP:
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
        break;

    case BSP_EVENT_DISCONNECT:
        err_code = sd_ble_gap_disconnect(m_ble_nus_c.conn_handle,
                                         BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
        break;

    case BSP_EVENT_KEY_3:
        NRF_LOG_INFO("Press the button 3 to send 244 byte data!");

        break;

    default:
        break;
    }
}

/**@brief Function for initializing the UART. */
static void uart_init(void)
{
    ret_code_t err_code;

    app_uart_comm_params_t const comm_params =
        {
            .rx_pin_no = RX_PIN_NUMBER,
            .tx_pin_no = TX_PIN_NUMBER,
            .rts_pin_no = RTS_PIN_NUMBER,
            .cts_pin_no = CTS_PIN_NUMBER,
            .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
            .use_parity = false,
            .baud_rate = UART_BAUDRATE_BAUDRATE_Baud115200};

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Nordic UART Service (NUS) client. */
static void nus_c_init(void)
{
    ret_code_t err_code;
    ble_nus_c_init_t init;

    init.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c, &init);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds. */
static void buttons_leds_init(void)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the timer. */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the nrf log module. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/** @brief Function for initializing the database discovery module. */
static void db_discovery_init(void)
{
    ret_code_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details Handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

int main(void)
{
    // Initialize.
    log_init();
    timer_init();
    uart_init();
    buttons_leds_init();
    db_discovery_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    nus_c_init();
    // scan_init();

    // Start execution.
//        printf("BLE UART Central:\n");
//        printf("BLE UART central example started.");
        printf("Long Range with Central Example started.\n");
    // scan_start();

    scan_start(); // Used the Coded PHY

    // Enter main loop.
    for (;;)
    {
        idle_state_handle();
    }
}
