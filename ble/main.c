/**
 * Copyright (c) 2016 - 2017, Nordic Semiconductor ASA
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
#include <string.h>
#include "nordic_common.h"
#include "app_error.h"
#include "app_uart.h"
#include "ble_db_discovery.h"
#include "app_timer.h"
#include "app_util.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "boards.h"
#include "ble.h"
#include "ble_gap.h"
#include "ble_hci.h"
#include "softdevice_handler.h"
#include "ble_advdata.h"
#include "ble_nus_c.h"
#include "ble_conn_state.h"

#include "nrf.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "app_button.h"
#include "ble_nus.h"
#include "app_util_platform.h"
#include "app_timer.h"

#include "SEGGER_RTT.h"
#include "SEGGER_RTT_Conf.h"
#include "bin_tree_node.h"
#include "nec_remoter.h"

#define CENTRAL_LINK_COUNT      2                               /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT   1                               /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#if (NRF_SD_BLE_API_VERSION == 3)
#define NRF_BLE_MAX_MTU_SIZE    GATT_MTU_SIZE_DEFAULT           /**< MTU size used in the softdevice enabling and to reply to a BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST event. */
#endif

#define UART_TX_BUF_SIZE        256                             /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE        256                             /**< UART RX buffer size. */

#define CENTRAL_SCANNING_LED    BSP_BOARD_LED_1

#define NUS_SERVICE_UUID_TYPE   BLE_UUID_TYPE_VENDOR_BEGIN      /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_TIMER_PRESCALER     0                               /**< Value of the RTC1 PRESCALER register. */
#define APP_TIMER_OP_QUEUE_SIZE 6                               /**< Size of timer operation queues. */

#define SCAN_INTERVAL           0x00A0                          /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             0x0050                          /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_ACTIVE             1                               /**< If 1, performe active scanning (scan requests). */
#define SCAN_SELECTIVE          0                               /**< If 1, ignore unknown devices (non whitelisted). */
#define SCAN_TIMEOUT            0x0000                          /**< Timout when scanning. 0x0000 disables timeout. */

#define MIN_CONNECTION_INTERVAL MSEC_TO_UNITS(7.5, UNIT_1_25_MS) /**< Determines minimum connection interval in millisecond. */
#define MAX_CONNECTION_INTERVAL MSEC_TO_UNITS(30, UNIT_1_25_MS) /**< Determines maximum connection interval in millisecond. */
#define SLAVE_LATENCY           0                               /**< Determines slave latency in counts of connection events. */
#define SUPERVISION_TIMEOUT     MSEC_TO_UNITS(4000, UNIT_10_MS) /**< Determines supervision time-out in units of 10 millisecond. */

#define UUID16_SIZE             2                               /**< Size of 16 bit UUID */
#define UUID32_SIZE             4                               /**< Size of 32 bit UUID */
#define UUID128_SIZE            16                              /**< Size of 128 bit UUID */

/**@brief Macro to unpack 16bit unsigned UUID from an octet stream. */
#define UUID16_EXTRACT(DST, SRC) \
    do                           \
    {                            \
        (*(DST))   = (SRC)[1];   \
        (*(DST)) <<= 8;          \
        (*(DST))  |= (SRC)[0];   \
    } while (0)

/**@brief Variable length data encapsulation in terms of length and pointer to data. */
typedef struct
{
	uint8_t 	* p_data;	 /**< Pointer to data. */
	uint16_t	  data_len;  /**< Length of data. */
} data_t;

static ble_nus_c_t              m_ble_nus_c0,m_ble_nus_c1;                    /**< Instance of NUS service. Must be passed to all NUS_C API calls. */
static ble_db_discovery_t       m_ble_db_discovery[CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT];             /**< Instance of database discovery module. Must be passed to all db_discovert API calls */

/**
 * @brief Connection parameters requested for connection.
 */
static const ble_gap_conn_params_t m_connection_param =
  {
    (uint16_t)MIN_CONNECTION_INTERVAL,  // Minimum connection
    (uint16_t)MAX_CONNECTION_INTERVAL,  // Maximum connection
    (uint16_t)SLAVE_LATENCY,            // Slave latency
    (uint16_t)SUPERVISION_TIMEOUT       // Supervision time-out
  };

/**
 * @brief Parameters used when scanning.
 */
static const ble_gap_scan_params_t m_scan_params =
{
    .active   = 1,
    .interval = SCAN_INTERVAL,
    .window   = SCAN_WINDOW,
    .timeout  = SCAN_TIMEOUT,
    #if (NRF_SD_BLE_API_VERSION == 2)
        .selective   = 0,
        .p_whitelist = NULL,
    #endif
    #if (NRF_SD_BLE_API_VERSION == 3)
        .use_whitelist = 0,
    #endif
};

/**
 * @brief NUS uuid
 */
static const ble_uuid_t m_nus_uuid =
  {
    .uuid = BLE_UUID_NUS_SERVICE,
    .type = NUS_SERVICE_UUID_TYPE
  };
    APP_TIMER_DEF(m_scan_timer_id);
    APP_TIMER_DEF(m_conn_timer_id);







  /* Peripheral related. */
#define APP_FEATURE_NOT_SUPPORTED        BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */
#define PERIPHERAL_ADVERTISING_LED       BSP_BOARD_LED_0
#define DEVICE_NAME                     "Nordic_UART"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */
#define APP_ADV_INTERVAL                64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                         /**< The advertising timeout (in units of seconds). */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000, APP_TIMER_PRESCALER)  /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000, APP_TIMER_PRESCALER) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */
#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(30, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
static ble_nus_t                        m_nus;                                      /**< Structure to identify the Nordic UART Service. */
static uint16_t                         m_conn_handle = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */

static ble_uuid_t                       m_adv_uuids[] = {{BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE}};  /**< Universally unique service identifier. */

static node_mac_rssi_t m_mac_rssis[SCAN_NUM_MAX];
node_c_status m_node_c_status = node_c_idle;
static uint8_t m_reply_buf[BLE_NUS_MAX_DATA_LEN];
#define L_NODE_LED       BSP_BOARD_LED_2
#define R_NODE_LED       BSP_BOARD_LED_3





/**@brief Function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing ASSERT call.
 * @param[in] p_file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
}




/**@brief Function to start scanning.
 */
static void scan_start(void)
{
    SEGGER_RTT_WriteString(0, "scan start\r\n");
	uint32_t err_code;
	app_timer_start(m_scan_timer_id, APP_TIMER_TICKS(3000,APP_TIMER_PRESCALER), NULL);
	(void) sd_ble_gap_scan_stop();
    err_code = sd_ble_gap_scan_start(&m_scan_params);
    bsp_board_led_on(CENTRAL_SCANNING_LED);
    
    // It is okay to ignore this error since we are stopping the scan anyway.
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
    SEGGER_RTT_printf(0, "scan start err code %d\r\n",err_code);
}
static void adv_start(void)
{
    ret_code_t err_code;
    // Start advertising.
    err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
	if(p_evt->conn_handle == 2)
	{	
        ble_nus_c_on_db_disc_evt(&m_ble_nus_c0, p_evt);
	}
	else if(p_evt->conn_handle == 1)
	{
	    ble_nus_c_on_db_disc_evt(&m_ble_nus_c1, p_evt);
	}
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' i.e '\r\n' (hex 0x0D) or if the string has reached a length of
 *          @ref NUS_MAX_DATA_LENGTH.
 */
void uart_event_handle(app_uart_evt_t * p_event)
{
//    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
//    static uint8_t index = 0;

    switch (p_event->evt_type)
    {
        /**@snippet [Handling data from UART] */
        case APP_UART_DATA_READY:
//            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
//            index++;

//            if ((data_array[index - 1] == '\n') || (index >= (BLE_NUS_MAX_DATA_LEN)))
//            {
//                while (ble_nus_c_string_send(&m_ble_nus_c0, data_array, index) != NRF_SUCCESS)
//                {
//                    // repeat until sent.
//                }
//                index = 0;
//            }
            break;
        /**@snippet [Handling data from UART] */
        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}



/**@brief Callback handling NUS Client events.
 *
 * @details This function is called to notify the application of NUS client events.
 *
 * @param[in]   p_ble_nus_c   NUS Client Handle. This identifies the NUS client
 * @param[in]   p_ble_nus_evt Pointer to the NUS Client event.
 */

/**@snippet [Handling events from the ble_nus_c module] */
static void ble_nus_c_evt_handler(ble_nus_c_t * p_ble_nus_c, const ble_nus_c_evt_t * p_ble_nus_evt)
{
    uint32_t err_code;
	SEGGER_RTT_WriteString( 0,"nus_handler\r\n");
    switch (p_ble_nus_evt->evt_type)
    {
        case BLE_NUS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_nus_c_handles_assign(p_ble_nus_c, p_ble_nus_evt->conn_handle, &p_ble_nus_evt->handles);
            APP_ERROR_CHECK(err_code);
            err_code = ble_nus_c_rx_notif_enable(p_ble_nus_c);
            APP_ERROR_CHECK(err_code);
            SEGGER_RTT_printf( 0,"%d discovery ok\r\n",p_ble_nus_evt->conn_handle);
            m_node_c_status = node_c_idle;
            node_status_send();
            break;

        case BLE_NUS_C_EVT_NUS_RX_EVT:
//            for (uint32_t i = 0; i < p_ble_nus_evt->data_len; i++)
//            {
//                while (app_uart_put( p_ble_nus_evt->p_data[i]) != NRF_SUCCESS);
//            }
//            while (ble_nus_c_string_send(p_ble_nus_c, p_ble_nus_evt->p_data, p_ble_nus_evt->data_len) != NRF_SUCCESS)
//            {
//                    // repeat until sent.
//            }
        {
            uint8_t *add;
            if(m_conn_handle == BLE_CONN_HANDLE_INVALID)//break if iphone is not connected
              break;
            add = p_ble_nus_evt->p_data + node_add_idx;
            SET_DIR(*add,p_ble_nus_c->conn_handle%2);//edit the address 
            err_code = ble_nus_string_send(&m_nus, p_ble_nus_evt->p_data, p_ble_nus_evt->data_len);//send data to iphone
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
              APP_ERROR_CHECK(err_code);
            }
         }
            break;

        case BLE_NUS_C_EVT_DISCONNECTED:
            SEGGER_RTT_WriteString(0,"Disconnected\r\n");
            node_status_send();
            break;
    }
}



/**@brief Reads an advertising report and checks if a uuid is present in the service list.
 *
 * @details The function is able to search for 16-bit, 32-bit and 128-bit service uuids.
 *          To see the format of a advertisement packet, see
 *          https://www.bluetooth.org/Technical/AssignedNumbers/generic_access_profile.htm
 *
 * @param[in]   p_target_uuid The uuid to search fir
 * @param[in]   p_adv_report  Pointer to the advertisement report.
 *
 * @retval      true if the UUID is present in the advertisement report. Otherwise false
 */
static bool is_uuid_present(const ble_uuid_t *p_target_uuid,
                            const ble_gap_evt_adv_report_t *p_adv_report)
{
    uint32_t err_code;
    uint32_t index = 0;
    uint8_t *p_data = (uint8_t *)p_adv_report->data;
    ble_uuid_t extracted_uuid;

    while (index < p_adv_report->dlen)
    {
        uint8_t field_length = p_data[index];
        uint8_t field_type   = p_data[index + 1];

        if ( (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE)
           || (field_type == BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_COMPLETE)
           )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID16_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(  UUID16_SIZE,
                                                &p_data[u_index * UUID16_SIZE + index + 2],
                                                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_32BIT_SERVICE_UUID_COMPLETE)
                )
        {
            for (uint32_t u_index = 0; u_index < (field_length / UUID32_SIZE); u_index++)
            {
                err_code = sd_ble_uuid_decode(UUID16_SIZE,
                &p_data[u_index * UUID32_SIZE + index + 2],
                &extracted_uuid);
                if (err_code == NRF_SUCCESS)
                {
                    if ((extracted_uuid.uuid == p_target_uuid->uuid)
                        && (extracted_uuid.type == p_target_uuid->type))
                    {
                        return true;
                    }
                }
            }
        }

        else if ( (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_MORE_AVAILABLE)
                || (field_type == BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE)
                )
        {
            err_code = sd_ble_uuid_decode(UUID128_SIZE,
                                          &p_data[index + 2],
                                          &extracted_uuid);
            if (err_code == NRF_SUCCESS)
            {
                if ((extracted_uuid.uuid == p_target_uuid->uuid)
                    && (extracted_uuid.type == p_target_uuid->type))
                {
                    return true;
                }
            }
        }
        index += field_length + 1;
    }
    return false;
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void on_ble_central_evt(ble_evt_t * p_ble_evt)
{
    uint32_t              err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_ADV_REPORT:
        {
            const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;
            
            if (is_uuid_present(&m_nus_uuid, p_adv_report))
            {
                SEGGER_RTT_printf(0,"evt_report \r\n");
                node_add_mac_rssi(p_adv_report->rssi,p_adv_report->peer_addr.addr);
//                err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
//                                              &m_scan_params,
//                                              &m_connection_param);
//
//                if (err_code == NRF_SUCCESS)
//                {
//                    // scan is automatically stopped by the connect
//                    SEGGER_RTT_printf(0,"Connecting to target %02x%02x%02x%02x%02x%02x\r\n",
//                             p_adv_report->peer_addr.addr[0],
//                             p_adv_report->peer_addr.addr[1],
//                             p_adv_report->peer_addr.addr[2],
//                             p_adv_report->peer_addr.addr[3],
//                             p_adv_report->peer_addr.addr[4],
//                             p_adv_report->peer_addr.addr[5]
//                             );
//                }
            }
        }
        break; // BLE_GAP_EVT_ADV_REPORT

        case BLE_GAP_EVT_CONNECTED:
            //NRF_LOG_DEBUG("Connected to target\r\n");
					SEGGER_RTT_printf(0,"try to find NUS on conn_handle 0x%x\r\n", p_gap_evt->conn_handle);
				
					APP_ERROR_CHECK_BOOL(p_gap_evt->conn_handle < CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT);
					err_code = ble_db_discovery_start(&m_ble_db_discovery[p_gap_evt->conn_handle], p_gap_evt->conn_handle);
					APP_ERROR_CHECK(err_code);
				
                    if (p_gap_evt->conn_handle == 1)
                    {
                        bsp_board_led_on(R_NODE_LED);
                    }
                    else if(p_gap_evt->conn_handle == 2)
                    {
                        bsp_board_led_on(L_NODE_LED);
                    }

              break; // BLE_GAP_EVT_CONNECTED
        case BLE_GAP_EVT_DISCONNECTED:
        {
//            uint8_t n_centrals;
//        
//            n_centrals = ble_conn_state_n_centrals();
//        
//            if (n_centrals == 0)
//            {
//                bsp_board_led_off(CENTRAL_CONNECTED_LED);
//            }
              if (p_gap_evt->conn_handle == 1)
              {
                  bsp_board_led_off(R_NODE_LED);
              }
              else if(p_gap_evt->conn_handle == 2)
              {
                  bsp_board_led_off(L_NODE_LED);
              }

        } break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_TIMEOUT:
            SEGGER_RTT_WriteString(0,"time out.\r\n");
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                m_node_c_status = node_c_idle;
                SEGGER_RTT_WriteString(0,"Connection Request timed out.\r\n");
                
            }
            else if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_SCAN)
            {
                SEGGER_RTT_printf(0,"Scan timed out.\r\n");
                //scan_timer_handler();
            }
            break; // BLE_GAP_EVT_TIMEOUT

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accepting parameters requested by peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                                    &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            //NRF_LOG_DEBUG("GATT Client Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            //NRF_LOG_DEBUG("GATT Server Timeout.\r\n");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            break;
    }
}

/**@brief Function for the application's SoftDevice event handler.
 *
 * @param[in] p_ble_evt SoftDevice event.
 */
static void on_ble_peripheral_evt(ble_evt_t * p_ble_evt)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            bsp_board_led_off(PERIPHERAL_ADVERTISING_LED);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break; // BLE_GAP_EVT_CONNECTED

        case BLE_GAP_EVT_DISCONNECTED:
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break; // BLE_GAP_EVT_DISCONNECTED

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GAP_EVT_SEC_PARAMS_REQUEST

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_SYS_ATTR_MISSING

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTC_EVT_TIMEOUT

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_TIMEOUT

        case BLE_EVT_USER_MEM_REQUEST:
            err_code = sd_ble_user_mem_reply(p_ble_evt->evt.gattc_evt.conn_handle, NULL);
            APP_ERROR_CHECK(err_code);
            break; // BLE_EVT_USER_MEM_REQUEST

        case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
        {
            ble_gatts_evt_rw_authorize_request_t  req;
            ble_gatts_rw_authorize_reply_params_t auth_reply;

            req = p_ble_evt->evt.gatts_evt.params.authorize_request;

            if (req.type != BLE_GATTS_AUTHORIZE_TYPE_INVALID)
            {
                if ((req.request.write.op == BLE_GATTS_OP_PREP_WRITE_REQ)     ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_NOW) ||
                    (req.request.write.op == BLE_GATTS_OP_EXEC_WRITE_REQ_CANCEL))
                {
                    if (req.type == BLE_GATTS_AUTHORIZE_TYPE_WRITE)
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_WRITE;
                    }
                    else
                    {
                        auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
                    }
                    auth_reply.params.write.gatt_status = APP_FEATURE_NOT_SUPPORTED;
                    err_code = sd_ble_gatts_rw_authorize_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                               &auth_reply);
                    APP_ERROR_CHECK(err_code);
                }
            }
        } break; // BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST

#if (NRF_SD_BLE_API_VERSION == 3)
        case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
            err_code = sd_ble_gatts_exchange_mtu_reply(p_ble_evt->evt.gatts_evt.conn_handle,
                                                       NRF_BLE_MAX_MTU_SIZE);
            APP_ERROR_CHECK(err_code);
            break; // BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST
#endif

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling advertising events.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            bsp_board_led_on(PERIPHERAL_ADVERTISING_LED);
            break;//BLE_ADV_EVT_FAST

        case BLE_ADV_EVT_IDLE:
        {
            ret_code_t err_code;
            err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
            APP_ERROR_CHECK(err_code);
        } break;//BLE_ADV_EVT_IDLE

        default:
            // No implementation needed.
            break;
    }
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the scheduler in the main loop after a BLE stack event has
 *          been received.
 *
 * @param[in] p_ble_evt  Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    uint16_t conn_handle;
    uint16_t role;
    ble_conn_state_on_ble_evt(p_ble_evt);

    // The connection handle should really be retrievable for any event type.
    conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    role        = ble_conn_state_role(conn_handle);
//	if(conn_handle < 3)
//	{
//	    SEGGER_RTT_printf(0,"pack %d %d %d\r\n",conn_handle,role,p_ble_evt->header.evt_id);
//	}
    // Based on the role this device plays in the connection, dispatch to the right applications.
    SEGGER_RTT_WriteString(0, "evt ");
    if (role == BLE_GAP_ROLE_PERIPH)
    {
        // Manages peripheral LEDs.
        SEGGER_RTT_WriteString(0, "periph\r\n");
        on_ble_peripheral_evt(p_ble_evt);

        ble_advertising_on_ble_evt(p_ble_evt);
        ble_conn_params_on_ble_evt(p_ble_evt);

        // Dispatch to peripheral applications.
        ble_nus_on_ble_evt(&m_nus, p_ble_evt);
        
    }
    else if ((role == BLE_GAP_ROLE_CENTRAL) || (p_ble_evt->header.evt_id == BLE_GAP_EVT_ADV_REPORT))
    {
        /** on_ble_central_evt will update the connection handles, so we want to execute it
         * after dispatching to the central applications upon disconnection. */
         SEGGER_RTT_WriteString(0, "central\r\n");
        if (p_ble_evt->header.evt_id != BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }

        if (conn_handle < CENTRAL_LINK_COUNT + PERIPHERAL_LINK_COUNT)
        {
            ble_db_discovery_on_ble_evt(&m_ble_db_discovery[conn_handle], p_ble_evt);
        }
        if(p_ble_evt->evt.gap_evt.conn_handle==2)
            ble_nus_c_on_ble_evt(&m_ble_nus_c0,p_ble_evt);
        else if(p_ble_evt->evt.gap_evt.conn_handle==1)
            ble_nus_c_on_ble_evt(&m_ble_nus_c1,p_ble_evt);
        // If the peer disconnected, we update the connection handles last.
        if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
        {
            on_ble_central_evt(p_ble_evt);
        }
    }
    
}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    //Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT,PERIPHERAL_LINK_COUNT);

    // Enable BLE stack.
#if (NRF_SD_BLE_API_VERSION == 3)
    ble_enable_params.gatt_enable_params.att_mtu = NRF_BLE_MAX_MTU_SIZE;
#endif
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Register with the SoftDevice handler module for BLE events.
    err_code = softdevice_ble_evt_handler_set(ble_evt_dispatch);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the UART.
 */
static void uart_init(void)
{
    uint32_t err_code;

    const app_uart_comm_params_t comm_params =
      {
        .rx_pin_no    = RX_PIN_NUMBER,
        .tx_pin_no    = TX_PIN_NUMBER,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_ENABLED,
        .use_parity   = false,
        .baud_rate    = UART_BAUDRATE_BAUDRATE_Baud115200
      };

    APP_UART_FIFO_INIT(&comm_params,
                        UART_RX_BUF_SIZE,
                        UART_TX_BUF_SIZE,
                        uart_event_handle,
                        APP_IRQ_PRIORITY_LOWEST,
                        err_code);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the NUS Client.
 */
static void nus_c_init(void)
{
    uint32_t         err_code;
    ble_nus_c_init_t nus_c_init_t;

    nus_c_init_t.evt_handler = ble_nus_c_evt_handler;

    err_code = ble_nus_c_init(&m_ble_nus_c0, &nus_c_init_t);
	err_code = ble_nus_c_init(&m_ble_nus_c1, &nus_c_init_t);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing buttons and leds.
 */
static void buttons_leds_init(void)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LED | BSP_INIT_BUTTONS,
                                 APP_TIMER_TICKS(100, APP_TIMER_PRESCALER),
                                 NULL);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
}


/** @brief Function for initializing the Database Discovery Module.
 */
static void db_discovery_init(void)
{
    uint32_t err_code = ble_db_discovery_init(db_disc_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_nus    Nordic UART Service structure.
 * @param[in] p_data   Data to be send to UART module.
 * @param[in] length   Length of the data.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
//    for (uint32_t i = 0; i < length; i++)
//    {
//        while (app_uart_put(p_data[i]) != NRF_SUCCESS);
//    }
//    while (app_uart_put('\r') != NRF_SUCCESS);
//    while (app_uart_put('\n') != NRF_SUCCESS);

//	uint32_t err_code = ble_nus_string_send(p_nus, p_data, length);
//	if (err_code != NRF_ERROR_INVALID_STATE)
//	{
//		APP_ERROR_CHECK(err_code);
//	}
    SEGGER_RTT_WriteString(0, "nus_data_hander start\r\n");
  uint8_t add=p_data[node_add_idx];

  if( IS_MY_PACK(add) )
  {
    node_com_hander(p_nus,p_data,length);
    SEGGER_RTT_WriteString(0, "my pack\r\n");
  }
  else
  {
    relay_pack(p_nus,p_data,length);
  }

//  if(m_ble_nus_c0.conn_handle != BLE_CONN_HANDLE_INVALID)
//  {
//    while (ble_nus_c_string_send(&m_ble_nus_c0, p_data, length) != NRF_SUCCESS)
//    {
//            // repeat until sent.
//    }
//    SEGGER_RTT_WriteString(0, "send by nus_c0");
//  }
//  if(m_ble_nus_c1.conn_handle != BLE_CONN_HANDLE_INVALID)
//  {
//    while (ble_nus_c_string_send(&m_ble_nus_c1, p_data, length) != NRF_SUCCESS)
//    {
//            // repeat until sent.
//    }
//    SEGGER_RTT_WriteString(0, "send by nus_c1");
//  }
//
//    SEGGER_RTT_Write(0, p_data, length);
}
/**@snippet [Handling the data received over BLE] */

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t       err_code;
    ble_nus_init_t nus_init;

    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = on_conn_params_evt;//turn from NULL to this ????? why???
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality..
 */
	static void advertising_init(void)
	{
		uint32_t			   err_code;
		ble_advdata_t		   advdata;
		ble_advdata_t		   scanrsp;
		ble_adv_modes_config_t options;
	
		// Build advertising data struct to pass into @ref ble_advertising_init.
		memset(&advdata, 0, sizeof(advdata));
		advdata.name_type		   = BLE_ADVDATA_FULL_NAME;
		advdata.include_appearance = false;
		advdata.flags			   = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
	
		memset(&scanrsp, 0, sizeof(scanrsp));
		scanrsp.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
		scanrsp.uuids_complete.p_uuids	= m_adv_uuids;
	
		memset(&options, 0, sizeof(options));
		options.ble_adv_fast_enabled  = true;
		options.ble_adv_fast_interval = APP_ADV_INTERVAL;
		options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
	
		err_code = ble_advertising_init(&advdata, &scanrsp, &options, on_adv_evt, NULL);
		APP_ERROR_CHECK(err_code);
	}


/** @brief Function for the Power manager.
 */
static void power_manage(void)
{
    uint32_t err_code = sd_app_evt_wait();
    APP_ERROR_CHECK(err_code);
}
 
int main(void)
{
    SEGGER_RTT_Init();
    APP_TIMER_INIT(APP_TIMER_PRESCALER, APP_TIMER_OP_QUEUE_SIZE, NULL);
    buttons_leds_init();
    ble_stack_init();
    db_discovery_init();
    uart_init();
    nus_c_init();
    
    gap_params_init();
    conn_params_init();
    services_init();
    advertising_init();
    // Start scanning for peripherals and initiate connection
    // with devices that advertise NUS UUID.
    SEGGER_RTT_WriteString(0,"ble relay started\r\n");
    adv_start();
    app_timer_create(&m_scan_timer_id,APP_TIMER_MODE_SINGLE_SHOT,scan_timer_handler);
    app_timer_create(&m_conn_timer_id,APP_TIMER_MODE_SINGLE_SHOT,conn_timer_handler);
    nec_init();
    
    //scan_start();
    for (;;)
    {
        power_manage();
    }
}

static void node_add_mac_rssi(uint8_t rssi,const uint8_t* add)
{
    uint8_t i;
    for(i = 0;i<SCAN_NUM_MAX;i++)
    {
        if(m_mac_rssis[i].rssi == 0)
            break;
        else if(memcmp(m_mac_rssis[i].mac, add,MAC_LEN)==0)
            break;
    }
    if(i < SCAN_NUM_MAX)
    {
        m_mac_rssis[i].rssi = rssi;
        memcpy(m_mac_rssis[i].mac,add,MAC_LEN);
        SEGGER_RTT_printf(0,"insert %d %d %02x%02x%02x%02x%02x%02x\r\n",i,rssi,add[0],add[1],add[2],add[3],add[4],add[5]);
    }
    
}

static void scan_timer_handler(void * p_context)//report the scaned signals to host
{
    uint32_t err_code;
    SEGGER_RTT_WriteString(0,"*****scan_timer_handler start \r\n");
    bsp_board_led_off(CENTRAL_SCANNING_LED);
    (void) sd_ble_gap_scan_stop();
    uint8_t* com = m_reply_buf + node_com_idx;
    uint8_t *add = m_reply_buf + node_add_idx;
    (void) sd_ble_gap_scan_stop();
    m_node_c_status = node_c_idle;
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
        return;
    *com = node_scan;
    *add = 0;
    node_d2h_scan_t* scan_data = (node_d2h_scan_t*) (m_reply_buf+node_data_idx);
    for(uint8_t i = 0;i<SCAN_NUM_MAX;i++)
        scan_data->rssi[i] = m_mac_rssis[i].rssi;
        
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID)//break if iphone is not connected
      return;
    err_code = ble_nus_string_send(&m_nus , m_reply_buf, NODE_PACK_HEADER_SIZE+SCAN_NUM_MAX);
    if (err_code != NRF_SUCCESS)
    {
        APP_ERROR_CHECK(err_code);
    }
    SEGGER_RTT_printf(0,"scan_timer_handler err_code %d \r\n",err_code);
}
static void conn_timer_handler(void * p_context)//report the scaned signals to host
{
    m_node_c_status = node_c_idle;
    int32_t err_code = sd_ble_gap_connect_cancel();
    SEGGER_RTT_printf(0, "conn timeout err_code %d\r\n",err_code);
}

static void node_status_send()
{
    SEGGER_RTT_WriteString(0, "node_status_send start\r\n");
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID)//break if iphone is not connected
      return;
    uint32_t err_code;
    uint8_t reply_len = 0;
    uint8_t *com = m_reply_buf + node_com_idx;
    uint8_t *add = m_reply_buf + node_add_idx;
    *com = node_getStatus;
    *add = 0;
    node_d2h_getStatus_t* node_status = (node_d2h_getStatus_t*)(m_reply_buf + node_data_idx);
    node_status->type = board_button;
    node_status->left_status = (uint8_t)(m_ble_nus_c0.conn_handle==BLE_CONN_HANDLE_INVALID)?node_p_idle:node_p_connected;
    node_status->right_status = (uint8_t)(m_ble_nus_c1.conn_handle==BLE_CONN_HANDLE_INVALID)?node_p_idle:node_p_connected;
    reply_len = NODE_PACK_HEADER_SIZE + sizeof(node_d2h_getStatus_t);
    
    if(m_conn_handle == BLE_CONN_HANDLE_INVALID)//break if iphone is not connected
      return;
    err_code = ble_nus_string_send(&m_nus, m_reply_buf, reply_len);
    if (err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
    SEGGER_RTT_printf(0, "node_status_send %d bytes type %d\r\n",reply_len,node_status->type);
}

static void node_com_hander(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{

    uint8_t com = p_data[node_com_idx];
    uint8_t *add = m_reply_buf + node_add_idx;
    uint8_t reply_len = 0;
    bool reply_now = false;
    m_reply_buf[node_com_idx] = com;
    *add = 0;
    uint32_t err_code;
    SEGGER_RTT_WriteString(0, "node_com_hander start\r\n");
    switch(com)
    {
        case node_getStatus:
            node_status_send();
        break;

        case node_scan:
            if(m_node_c_status != node_c_idle)
                break;
            if(m_ble_nus_c0.conn_handle != BLE_CONN_HANDLE_INVALID && m_ble_nus_c1.conn_handle != BLE_CONN_HANDLE_INVALID)
                break;
            m_node_c_status = node_c_scanning;
            memset(m_mac_rssis,0, sizeof(m_mac_rssis));
            SEGGER_RTT_printf(0, "memset size %d\r\n",sizeof(m_mac_rssis));
            scan_start();
        break;

        case node_connect:
        {
            SEGGER_RTT_WriteString(0, "in node_com_hander node_connect\r\n");
            if(m_node_c_status != node_c_idle)
                break;
            if(m_ble_nus_c0.conn_handle != BLE_CONN_HANDLE_INVALID && m_ble_nus_c1.conn_handle != BLE_CONN_HANDLE_INVALID)
                break;
            ble_gap_addr_t gap_addr;
            node_h2d_connect_t* con_idx;
            uint8_t idx;
            uint8_t *mac;
            m_node_c_status = node_c_connecting;
            con_idx = (node_h2d_connect_t*)(p_data+node_data_idx);
            idx = con_idx->idx;
            mac = m_mac_rssis[idx].mac;
            memset(&gap_addr,0,sizeof(ble_gap_addr_t));
            gap_addr.addr_type = 1;
            memcpy(gap_addr.addr,mac,MAC_LEN);
            app_timer_start(m_conn_timer_id, APP_TIMER_TICKS(3000,APP_TIMER_PRESCALER), NULL);
            sd_ble_gap_connect(&gap_addr,&m_scan_params,&m_connection_param);
        }
        break;

        case node_disconnect:
            if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
        break;

        case node_loopback:
            SEGGER_RTT_WriteString(0, "in node_loopback\r\n");
            reply_now = true;
            reply_len = length;
            memcpy(m_reply_buf, p_data, reply_len);
        break;

        case node_remoter:
            SEGGER_RTT_WriteString(0, "in node_remoter\r\n");
            node_h2d_remoter_t* remoter = (node_h2d_remoter_t*)(p_data+node_data_idx);
            nec_send(remoter->add, remoter->data);
        break;
        default:

        break;
    }
    
    if(reply_now)
    {
        if(m_conn_handle == BLE_CONN_HANDLE_INVALID)//break if iphone is not connected
          return;
        SEGGER_RTT_WriteString(0, "ready to reply now\r\n");
        reply_len = (reply_len>=NODE_PAYLOAD_SIZE)?NODE_PAYLOAD_SIZE:reply_len;
        uint32_t err_code = ble_nus_string_send(p_nus, m_reply_buf, reply_len);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
    SEGGER_RTT_WriteString(0, "node_com_hander end\r\n");
}

static void relay_pack(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length)
{
    uint8_t *add=p_data + node_add_idx;
    uint8_t dir = SEND_DIR(*add);
    *add = COUNTDOWN_ADD(*add);
    SEGGER_RTT_printf(0, "relay_pack dir %d nus0 %d nus1 %d\r\n",dir,m_ble_nus_c0.conn_handle,m_ble_nus_c1.conn_handle);
    if(dir==0 && m_ble_nus_c0.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        SEGGER_RTT_WriteString(0, "send by nus_c0\r\n");
        while (ble_nus_c_string_send(&m_ble_nus_c0, p_data, length) != NRF_SUCCESS)
        {
                // repeat until sent.
        }
        
    }
    else if(dir!=0 && m_ble_nus_c1.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        SEGGER_RTT_WriteString(0, "send by nus_c1\r\n");
        while (ble_nus_c_string_send(&m_ble_nus_c1, p_data, length) != NRF_SUCCESS)
        {
                // repeat until sent.
        }
        
    }
}
