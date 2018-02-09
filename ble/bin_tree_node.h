#ifndef BIN_TREE_NODE_H__
#define BIN_TREE_NODE_H__
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "nordic_common.h"
#include "ble_nus.h"

#define SCAN_NUM_MAX 18
#define MAC_LEN 6
#define IS_MY_PACK(add)  ( ((add)&0x03) == 0 )
#define SEND_DIR(add)  ( (add) & (0x10<<((add)&0x03)) )
#define SET_DIR(add,dir) ( (add) |= (dir)<<4<<((++add)&0x03) )
#define COUNTDOWN_ADD(add) ((add)-1)
#define COUNTUP_ADD(add) ((add)+1)
#define NODE_PACK_HEADER_SIZE 2
#define NODE_PAYLOAD_SIZE 20
typedef enum
{
    node_add_idx = 0,
    node_com_idx,
    node_data_idx,
} node_idx;

typedef enum
{
    node_left = 0,
    node_right,
} node_dir;//direction

typedef enum
{
    board_led,
    board_button,
}node_board_type;

typedef enum
{
    node_getStatus,
    node_scan,
    node_connect,
    node_disconnect,
    node_loopback,
    node_remoter,
    node_gpio,
    
} node_com_t ;

typedef enum
{
    node_p_idle,
    node_p_connected,
} node_p_status;

typedef enum
{
    node_c_idle,
    node_c_scanning,
    node_c_connecting,
} node_c_status;

typedef struct
{
    uint8_t   type;
    uint8_t   left_status;
    uint8_t   right_status;
} node_d2h_getStatus_t;

typedef struct
{
    uint8_t rssi[SCAN_NUM_MAX];
} node_d2h_scan_t;

typedef struct
{
    uint8_t rssi;
    uint8_t mac[MAC_LEN];
} node_mac_rssi_t;

typedef struct
{
    uint8_t idx;
} node_h2d_connect_t;

typedef struct
{
    uint8_t add;
    uint8_t data;
} node_h2d_remoter_t;


typedef struct
{
    uint8_t dir;
} node_h2d_disconnect_t;

typedef struct
{
    uint32_t idx;
    uint32_t rw;
    uint32_t value;
} node_h2d_gpio_t;

typedef struct
{
    uint32_t idx;
    uint32_t value;
} node_d2h_gpio_t;
static void node_add_mac_rssi(uint8_t rssi,const uint8_t* add);
static void scan_timer_handler(void * p_context);
static void conn_timer_handler(void * p_context);

static void node_status_send(void);
static void node_com_hander(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
static void relay_pack(ble_nus_t * p_nus, uint8_t * p_data, uint16_t length);
#endif//BIN_TREE_NODE_H__
