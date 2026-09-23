/******************************************************************************
 * @file    common.h
 * @brief   公共定义、日志宏、模块接口声明
 ******************************************************************************/
#pragma once

#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"

/*============================================================================
 * 日志宏 - 通过 UART 输出（各模块共用）
 *============================================================================*/
#define LOG_TAG_UART    "UART"
#define LOG_TAG_GATT    "GATT"
#define LOG_TAG_MESH    "MESH"
#define LOG_TAG_CFG     "CFG"
#define LOG_TAG_AT      "AT"
#define LOG_TAG_MAIN    "MAIN"

/* 日志级别控制（可通过 AT 指令动态调整） */
typedef enum {
    LOG_LEVEL_NONE = 0,
    LOG_LEVEL_ERROR,
    LOG_LEVEL_WARN,
    LOG_LEVEL_INFO,
    LOG_LEVEL_DEBUG,
} log_level_t;

extern log_level_t g_log_level;

#define LOGI(tag, fmt, ...) do { \
    if (g_log_level >= LOG_LEVEL_INFO)  uart_log_printf("[" tag "] " fmt "\r\n", ##__VA_ARGS__); \
} while(0)

#define LOGW(tag, fmt, ...) do { \
    if (g_log_level >= LOG_LEVEL_WARN)  uart_log_printf("[" tag "] WARN: " fmt "\r\n", ##__VA_ARGS__); \
} while(0)

#define LOGE(tag, fmt, ...) do { \
    if (g_log_level >= LOG_LEVEL_ERROR) uart_log_printf("[" tag "] ERROR: " fmt "\r\n", ##__VA_ARGS__); \
} while(0)

#define LOGD(tag, fmt, ...) do { \
    if (g_log_level >= LOG_LEVEL_DEBUG) uart_log_printf("[" tag "] " fmt "\r\n", ##__VA_ARGS__); \
} while(0)

/*============================================================================
 * 固件信息
 *============================================================================*/
#define FW_PREFIX       "dh_device"
#define FW_DATETIME     "20260923_090300"
#define FW_VERSION      "1.0.0"

/*============================================================================
 * 模块接口声明
 *============================================================================*/

/* (1) UART 通信模块 */
void hal_uart_init(void);
void hal_uart_process(void);
void uart_log_print(const char *str);
void uart_log_printf(const char *fmt, ...);
u8   uart_rx_available(void);
u8   uart_rx_read(void);

/* (2) BLE GATT 通信模块 */
void ble_gatt_init(u8 *mac);
void ble_gatt_start_adv(void);
void ble_gatt_stop_adv(void);
void ble_gatt_get_status(char *buf, int bufsize);
void ble_gatt_get_info(char *buf, int bufsize);
u8   ble_gatt_is_connected(void);
u8   ble_gatt_is_adv_running(void);
void ble_gatt_set_device_name(const char *name);
void ble_gatt_notify(u16 connHandle, u16 attrHandle, u8 *data, u8 len);

/* (3) BLE Mesh 通信模块 */
void ble_mesh_init(void);
void ble_mesh_process(void);
void ble_mesh_get_status(char *buf, int bufsize);
u8   ble_mesh_is_active(void);

/* (4) 配置管理模块 */
void config_init(void);
void config_save(void);
void config_reset(void);

typedef struct {
    /* BLE GATT 配置 */
    u16 adv_interval_ms;        /* 广播间隔(ms) */
    u8  adv_type;               /* 广播类型 */
    u16 conn_interval_min;      /* 最小连接间隔(×1.25ms) */
    u16 conn_interval_max;      /* 最大连接间隔(×1.25ms) */
    u16 conn_latency;           /* 从机延迟 */
    u16 conn_timeout;           /* 连接超时(×10ms) */
    u16 mtu;                    /* MTU 大小 */
    u8  phy_mode;               /* PHY: 1=1M, 2=2M, 3=Coded */
    u8  security_enable;        /* SMP 使能 */
    char device_name[32];       /* 设备名称 */

    /* BLE Mesh 配置 */
    u16 mesh_net_key[8];        /* Mesh 网络密钥 */
    u16 mesh_app_key[8];        /* Mesh 应用密钥 */
    u16 mesh_addr;              /* Mesh 本机地址 */
    u8  mesh_ttl;               /* Mesh TTL */
    u8  mesh_enable;            /* Mesh 使能标志 */

    /* UART 配置 */
    u32 uart_baud;              /* 波特率 */
    u8  log_level;              /* 日志级别 */
} config_data_t;

extern config_data_t g_config;

/* (5) AT 指令管理模块 */
void at_cmd_init(void);
void at_cmd_process(void);

/* 设备 MAC（全局） */
extern u8 g_mac[6];
