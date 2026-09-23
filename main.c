/******************************************************************************
 * @file    main.c
 * @brief   DH BLE Device 主程序
 *
 *  模块架构：
 *    (1) hal_uart   - UART 通信（RX=PA0, TX=PD7）
 *    (2) ble_gatt   - BLE GATT 通信（广播 + 手机蓝牙）
 *    (3) ble_mesh   - BLE Mesh 通信（模组间通信，预留接口）
 *    (4) config     - 配置管理（Flash 存储）
 *    (5) at_cmd     - AT 指令管理
 *    (6) main       - 主程序入口 + 调度
 *
 *  设计原则：
 *    - UART / BLE GATT / BLE Mesh 互不影响
 *    - GATT 和 Mesh 可通过 UART 打印日志
 *    - Mesh 空间信息可通过 GATT 进行配置
 ******************************************************************************/
#include "common.h"
#include "app_config.h"
#include "app_buffer.h"
#include "app_att.h"

/*============================================================================
 * 全局变量
 *============================================================================*/
u8 g_mac[6] = {0};
log_level_t g_log_level = LOG_LEVEL_INFO;
config_data_t g_config;

/******************************************************************************
 *                         (1) UART 通信模块
 *
 *  功能：硬件 UART 驱动、日志输出、RX 接收
 *  引脚：TX=PD7, RX=PA0, 115200 baud
 *  特点：轮询 TX，轮询 RX（在主循环中调用）
 ******************************************************************************/

#define UART_TX_PIN     UART_TX_PD7
#define UART_RX_PIN     UART_RX_PA0

/* RX 环形缓冲区 */
#define UART_RX_BUF_SZ  256
static u8 uart_rx_buf[UART_RX_BUF_SZ];
static volatile u16 uart_rx_head = 0;
static volatile u16 uart_rx_tail = 0;

void hal_uart_init(void)
{
    uart_gpio_set(UART_TX_PIN, UART_RX_PIN);
    uart_init_baudrate(g_config.uart_baud, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
    uart_TxIndex = 0;
    uart_RxIndex = 0;
    LOGI(LOG_TAG_UART, "UART init: TX=PD7 RX=PA0 %d baud", g_config.uart_baud);
}

static void uart_tx_byte(u8 c)
{
    while ((reg_uart_buf_cnt >> 4) > 7);
    reg_uart_data_buf(uart_TxIndex) = c;
    uart_TxIndex = (uart_TxIndex + 1) & 0x03;
}

void uart_log_print(const char *str)
{
    if (!str) return;
    while (*str) uart_tx_byte((u8)*str++);
}

static void uart_print_num(u32 val, int base, int uppercase)
{
    char buf[10]; int i = 0;
    if (val == 0) { uart_tx_byte('0'); return; }
    while (val > 0) {
        int d = val % base;
        buf[i++] = (d < 10) ? ('0' + d) : ((uppercase ? 'A' : 'a') + d - 10);
        val /= base;
    }
    while (i > 0) uart_tx_byte((u8)buf[--i]);
}

void uart_log_printf(const char *fmt, ...)
{
    if (!fmt) return;
    __builtin_va_list args;
    __builtin_va_start(args, fmt);
    const char *p = fmt;
    while (*p) {
        if (*p != '%') { uart_tx_byte((u8)*p++); continue; }
        p++;
        switch (*p) {
            case 's': { const char *s = __builtin_va_arg(args, const char*); if(!s)s="(null)"; while(*s) uart_tx_byte((u8)*s++); break; }
            case 'd': { int v = __builtin_va_arg(args, int); if(v<0){uart_tx_byte('-');v=-v;} uart_print_num(v,10,0); break; }
            case 'u': { uart_print_num(__builtin_va_arg(args,u32),10,0); break; }
            case 'x': { uart_print_num(__builtin_va_arg(args,u32),16,0); break; }
            case 'X': { uart_print_num(__builtin_va_arg(args,u32),16,1); break; }
            case 'c': { uart_tx_byte((u8)__builtin_va_arg(args,int)); break; }
            case '%': { uart_tx_byte('%'); break; }
            default:  { uart_tx_byte('%'); uart_tx_byte((u8)*p); break; }
        }
        p++;
    }
    __builtin_va_end(args);
}

void hal_uart_process(void)
{
    /* 从硬件 FIFO 读取到环形缓冲区 */
    while (reg_uart_buf_cnt & FLD_UART_RX_BUF_CNT) {
        u8 c = reg_uart_data_buf(uart_RxIndex);
        uart_RxIndex = (uart_RxIndex + 1) & 0x03;
        u16 next = (uart_rx_head + 1) & (UART_RX_BUF_SZ - 1);
        if (next != uart_rx_tail) {
            uart_rx_buf[uart_rx_head] = c;
            uart_rx_head = next;
        }
    }
}

u8 uart_rx_available(void)
{
    return (uart_rx_head != uart_rx_tail) ? 1 : 0;
}

u8 uart_rx_read(void)
{
    u8 c = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1) & (UART_RX_BUF_SZ - 1);
    return c;
}

static void delay_ms(u32 ms)
{
    u32 t0 = clock_time();
    while ((u32)(clock_time() - t0) < ms * CLOCK_SYS_CLOCK_HZ / 1000);
}

/******************************************************************************
 *                         (2) BLE GATT 通信模块
 *
 *  功能：BLE 广播、GAP/GATT 服务、手机连接与数据交互
 *  服务：GAP / GATT / Device Info / DH Custom(SPP-like) / OTA
 ******************************************************************************/

static u8 g_adv_enabled = 0;
static u8 g_connected = 0;
static char g_devname[32] = "dh_device_0000";
_attribute_ble_data_retention_ static u8 ota_is_working = 0;

static void app_enter_ota_mode(void) { ota_is_working = 1; }

static int app_controller_event_callback(u32 h, u8 *p, int n)
{
    if (h & HCI_FLAG_EVENT_BT_STD) {
        u8 evt = h & 0xff;
        if (evt == HCI_EVT_DISCONNECTION_COMPLETE) {
            g_connected = 0;
            LOGI(LOG_TAG_GATT, "Disconnected");
        } else if (evt == HCI_EVT_LE_META) {
            if (p[0] == HCI_SUB_EVT_LE_CONNECTION_COMPLETE) {
                g_connected = 1;
                LOGI(LOG_TAG_GATT, "Connected");
                hci_le_connectionCompleteEvt_t *pc = (hci_le_connectionCompleteEvt_t *)p;
                if (pc->role == ACL_ROLE_PERIPHERAL)
                    bls_l2cap_requestConnParamUpdate(pc->connHandle,
                        g_config.conn_interval_min, g_config.conn_interval_max,
                        g_config.conn_latency, g_config.conn_timeout);
            } else if (p[0] == HCI_SUB_EVT_LE_CONNECTION_UPDATE_COMPLETE) {
                LOGI(LOG_TAG_GATT, "ConnParam updated");
            }
        }
    }
    return 0;
}

static int app_host_event_callback(u32 h, u8 *para, int n)
{
    u8 evt = h & 0xFF;
    if (evt == GAP_EVT_ATT_EXCHANGE_MTU) LOGI(LOG_TAG_GATT, "MTU exchanged");
    return 0;
}

static int app_gatt_data_handler(u16 connHandle, u8 *pkt)
{
    rf_packet_att_t *pAtt = (rf_packet_att_t *)pkt;
    if (!(pAtt->opcode & 0x01)) {
        switch (pAtt->opcode) {
            case ATT_OP_FIND_INFO_REQ:
            case ATT_OP_FIND_BY_TYPE_VALUE_REQ:
            case ATT_OP_READ_BY_TYPE_REQ:
            case ATT_OP_READ_BY_GROUP_TYPE_REQ:
                blc_gatt_pushErrResponse(connHandle, pAtt->opcode, pAtt->handle, ATT_ERR_ATTR_NOT_FOUND);
                break;
            case ATT_OP_READ_REQ:
            case ATT_OP_READ_BLOB_REQ:
            case ATT_OP_READ_MULTI_REQ:
            case ATT_OP_WRITE_REQ:
            case ATT_OP_PREPARE_WRITE_REQ:
                blc_gatt_pushErrResponse(connHandle, pAtt->opcode, pAtt->handle, ATT_ERR_INVALID_HANDLE);
                break;
        }
    }
    return 0;
}

void task_suspend_exit(u8 e, u8 *p, int n) { rf_set_power_level_index(RF_POWER_P0dBm); }

void ble_gatt_init(u8 *mac)
{
    /* 生成设备名: dh_device_XXXX */
    const char hx[] = "0123456789ABCDEF";
    g_devname[0]='d'; g_devname[1]='h'; g_devname[2]='_'; g_devname[3]='d';
    g_devname[4]='e'; g_devname[5]='v'; g_devname[6]='i'; g_devname[7]='c';
    g_devname[8]='e'; g_devname[9]='_';
    g_devname[10]=hx[(mac[1]>>4)&0xF]; g_devname[11]=hx[mac[1]&0xF];
    g_devname[12]=hx[(mac[0]>>4)&0xF]; g_devname[13]=hx[mac[0]&0xF];
    g_devname[14]='\0';

    /* BLE Link Layer */
    blc_ll_initBasicMCU();
    blc_ll_initStandby_module(mac);
    blc_ll_initLegacyAdvertising_module();
    blc_ll_initAclConnection_module();
    blc_ll_initAclPeriphrRole_module();
    blc_ll_setMaxConnectionNumber(ACL_CENTRAL_MAX_NUM, ACL_PERIPHR_MAX_NUM);
    blc_ll_setAclConnMaxOctetsNumber(ACL_CONN_MAX_RX_OCTETS, ACL_MASTER_MAX_TX_OCTETS, ACL_SLAVE_MAX_TX_OCTETS);
    blc_ll_initAclConnRxFifo(app_acl_rxfifo, ACL_RX_FIFO_SIZE, ACL_RX_FIFO_NUM);
    blc_ll_initAclPeriphrTxFifo(app_acl_slvTxfifo, ACL_SLAVE_TX_FIFO_SIZE, ACL_SLAVE_TX_FIFO_NUM, ACL_PERIPHR_MAX_NUM);
    LOGI(LOG_TAG_GATT, "LL init OK");

    /* HCI */
    blc_hci_registerControllerDataHandler(blc_l2cap_pktHandler);
    blc_hci_registerControllerEventHandler(app_controller_event_callback);
    blc_hci_setEventMask_cmd(HCI_EVT_MASK_DISCONNECTION_COMPLETE);
    blc_hci_le_setEventMask_cmd(HCI_LE_EVT_MASK_CONNECTION_COMPLETE | HCI_LE_EVT_MASK_CONNECTION_UPDATE_COMPLETE);

    /* GAP/GATT */
    blc_gap_init();
    blc_l2cap_initAclConnSlaveMtuBuffer(mtu_s_rx_fifo, MTU_S_BUFF_SIZE_MAX, mtu_s_tx_fifo, MTU_S_BUFF_SIZE_MAX);
    blc_att_setSlaveRxMTUSize(ATT_MTU_SLAVE_RX_MAX_SIZE);
    my_gatt_init();
    blc_gatt_register_data_handler(app_gatt_data_handler);
    LOGI(LOG_TAG_GATT, "GAP/GATT init OK");

    /* SMP */
    if (g_config.security_enable) {
        blc_smp_setSecurityLevel_slave(LE_Security_Mode_1_Level_1);
    } else {
        blc_smp_setSecurityLevel_slave(No_Security);
    }
    blc_smp_smpParamInit();

    /* Host callbacks */
    blc_gap_registerHostEventHandler(app_host_event_callback);
    blc_gap_setEventMask(GAP_EVT_MASK_SMP_PAIRING_BEGIN | GAP_EVT_MASK_SMP_PAIRING_SUCCESS |
                          GAP_EVT_MASK_SMP_PAIRING_FAIL | GAP_EVT_MASK_SMP_CONN_ENCRYPTION_DONE);

    blc_app_checkControllerHostInitialization();
    rf_set_power_level_index(RF_POWER_P0dBm);
    blc_pm_setSleepMask(PM_SLEEP_DISABLE);

    /* OTA */
    blc_ota_initOtaServer_module();
    blc_ota_registerOtaStartCmdCb(app_enter_ota_mode);
    blc_ota_setOtaProcessTimeout(30);

    LOGI(LOG_TAG_GATT, "Device: %s, MAC: %02X:%02X:%02X:%02X:%02X:%02X",
         g_devname, mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
}

void ble_gatt_start_adv(void)
{
    if (g_adv_enabled) { LOGW(LOG_TAG_GATT, "ADV already running"); return; }
    u8 nlen = 0; while (g_devname[nlen]) nlen++;
    u8 adv[31]; u8 al = 0;
    adv[al++]=0x02; adv[al++]=0x01; adv[al++]=0x05;
    adv[al++]=1+nlen; adv[al++]=0x09;
    for (u8 i=0;i<nlen;i++) adv[al++]=(u8)g_devname[i];
    u8 sr[31]; u8 sl = 0;
    sr[sl++]=1+nlen; sr[sl++]=0x09;
    for (u8 i=0;i<nlen;i++) sr[sl++]=(u8)g_devname[i];
    blc_ll_setAdvData(adv, al);
    blc_ll_setScanRspData(sr, sl);
    blc_ll_setAdvParam(g_config.adv_interval_ms * 8 / 5, g_config.adv_interval_ms * 8 / 5,
                        ADV_TYPE_CONNECTABLE_UNDIRECTED, OWN_ADDRESS_PUBLIC, 0, NULL,
                        BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
    blc_ll_setAdvEnable(BLC_ADV_ENABLE);
    g_adv_enabled = 1;
    LOGI(LOG_TAG_GATT, "ADV started: %s (interval=%dms)", g_devname, g_config.adv_interval_ms);
}

void ble_gatt_stop_adv(void)
{
    if (!g_adv_enabled) { LOGW(LOG_TAG_GATT, "ADV already stopped"); return; }
    blc_ll_setAdvEnable(BLC_ADV_DISABLE);
    g_adv_enabled = 0;
    LOGI(LOG_TAG_GATT, "ADV stopped");
}

u8 ble_gatt_is_connected(void) { return g_connected; }
u8 ble_gatt_is_adv_running(void) { return g_adv_enabled; }

void ble_gatt_set_device_name(const char *name)
{
    int i = 0;
    while (name[i] && i < 31) { g_devname[i] = name[i]; i++; }
    g_devname[i] = '\0';
}

void ble_gatt_get_status(char *buf, int bufsize)
{
    uart_log_printf("  BLE State   : %s\r\n", g_connected ? "CONNECTED" : "IDLE");
    uart_log_printf("  ADV State   : %s\r\n", g_adv_enabled ? "RUNNING" : "STOPPED");
    uart_log_printf("  Device Name : %s\r\n", g_devname);
    uart_log_printf("  MAC         : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    g_mac[5], g_mac[4], g_mac[3], g_mac[2], g_mac[1], g_mac[0]);
}

void ble_gatt_get_info(char *buf, int bufsize)
{
    uart_log_printf("  Device Name   : %s\r\n", g_devname);
    uart_log_printf("  MAC           : %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                    g_mac[5], g_mac[4], g_mac[3], g_mac[2], g_mac[1], g_mac[0]);
    uart_log_printf("  Adv Interval  : %d ms\r\n", g_config.adv_interval_ms);
    uart_log_printf("  Conn Interval : %d-%d (x1.25ms)\r\n", g_config.conn_interval_min, g_config.conn_interval_max);
    uart_log_printf("  Conn Latency  : %d\r\n", g_config.conn_latency);
    uart_log_printf("  Conn Timeout  : %d (x10ms)\r\n", g_config.conn_timeout);
    uart_log_printf("  MTU           : %d\r\n", g_config.mtu);
    uart_log_printf("  PHY           : %s\r\n", g_config.phy_mode==2?"2M":g_config.phy_mode==3?"Coded":"1M");
    uart_log_printf("  Security      : %s\r\n", g_config.security_enable?"Enabled":"Disabled");
    uart_log_printf("  OTA           : Enabled\r\n");
}

/******************************************************************************
 *                         (3) BLE Mesh 通信模块（预留接口）
 *
 *  功能：模组间 Mesh 通信
 *  状态：需要 Telink SIG Mesh SDK，当前为 stub
 ******************************************************************************/

void ble_mesh_init(void)
{
    if (!g_config.mesh_enable) {
        LOGI(LOG_TAG_MESH, "Mesh disabled");
        return;
    }
    /* TODO: 接入 Telink SIG Mesh SDK
     *   mesh_init(g_config.mesh_net_key, g_config.mesh_app_key,
     *             g_config.mesh_addr, g_config.mesh_ttl);
     */
    LOGI(LOG_TAG_MESH, "Mesh init (stub - SDK not available)");
    LOGI(LOG_TAG_MESH, "  Net Key : %04X...", g_config.mesh_net_key[0]);
    LOGI(LOG_TAG_MESH, "  App Key : %04X...", g_config.mesh_app_key[0]);
    LOGI(LOG_TAG_MESH, "  Address : 0x%04X", g_config.mesh_addr);
    LOGI(LOG_TAG_MESH, "  TTL     : %d", g_config.mesh_ttl);
}

void ble_mesh_process(void)
{
    if (!g_config.mesh_enable) return;
    /* TODO: mesh_process(); */
}

u8 ble_mesh_is_active(void)
{
    return g_config.mesh_enable;
}

void ble_mesh_get_status(char *buf, int bufsize)
{
    uart_log_printf("  Mesh State  : %s\r\n", g_config.mesh_enable ? "ENABLED" : "DISABLED");
    if (g_config.mesh_enable) {
        uart_log_printf("  Net Key     : %04X%04X...\r\n", g_config.mesh_net_key[0], g_config.mesh_net_key[1]);
        uart_log_printf("  App Key     : %04X%04X...\r\n", g_config.mesh_app_key[0], g_config.mesh_app_key[1]);
        uart_log_printf("  Address     : 0x%04X\r\n", g_config.mesh_addr);
        uart_log_printf("  TTL         : %d\r\n", g_config.mesh_ttl);
    }
}

/******************************************************************************
 *                         (4) 配置管理模块
 *
 *  功能：Flash 存储配置数据，支持读取/保存/恢复默认
 ******************************************************************************/

/* Flash 存储地址（使用最后一个 sector，避免与固件/OTA 冲突） */
#define CONFIG_FLASH_ADDR       0x7E000
#define CONFIG_FLASH_MAGIC      0x44484346  /* "DHCF" */

typedef struct {
    u32 magic;
    config_data_t data;
    u32 checksum;
} config_flash_t;

static u32 config_checksum(config_data_t *d)
{
    u32 sum = 0;
    u8 *p = (u8 *)d;
    for (u32 i = 0; i < sizeof(config_data_t); i++) sum += p[i];
    return sum;
}

static void config_set_defaults(void)
{
    g_config.adv_interval_ms    = 30;
    g_config.adv_type           = ADV_TYPE_CONNECTABLE_UNDIRECTED;
    g_config.conn_interval_min  = 20;
    g_config.conn_interval_max  = 40;
    g_config.conn_latency       = 99;
    g_config.conn_timeout       = 400;
    g_config.mtu                = 23;
    g_config.phy_mode           = 1;
    g_config.security_enable    = 0;
    g_config.uart_baud          = 115200;
    g_config.log_level          = LOG_LEVEL_INFO;

    /* 默认设备名（MAC 初始化后会更新） */
    g_config.device_name[0] = '\0';

    /* Mesh 默认值 */
    memset(g_config.mesh_net_key, 0, sizeof(g_config.mesh_net_key));
    memset(g_config.mesh_app_key, 0, sizeof(g_config.mesh_app_key));
    g_config.mesh_addr   = 0x0001;
    g_config.mesh_ttl    = 5;
    g_config.mesh_enable = 0;
}

void config_init(void)
{
    config_set_defaults();

    /* 尝试从 Flash 读取 */
    config_flash_t fc;
    flash_read_page(CONFIG_FLASH_ADDR, sizeof(fc), (u8 *)&fc);

    if (fc.magic == CONFIG_FLASH_MAGIC && fc.checksum == config_checksum(&fc.data)) {
        memcpy(&g_config, &fc.data, sizeof(config_data_t));
        LOGI(LOG_TAG_CFG, "Config loaded from Flash");
    } else {
        LOGW(LOG_TAG_CFG, "No valid config in Flash, using defaults");
    }
}

void config_save(void)
{
    config_flash_t fc;
    fc.magic = CONFIG_FLASH_MAGIC;
    memcpy(&fc.data, &g_config, sizeof(config_data_t));
    fc.checksum = config_checksum(&fc.data);

    flash_erase_sector(CONFIG_FLASH_ADDR);
    flash_write_page(CONFIG_FLASH_ADDR, sizeof(fc), (u8 *)&fc);
    LOGI(LOG_TAG_CFG, "Config saved to Flash");
}

void config_reset(void)
{
    config_set_defaults();
    config_save();
    LOGI(LOG_TAG_CFG, "Config reset to defaults");
}

/******************************************************************************
 *                         (5) AT 指令管理模块
 *
 *  功能：解析 UART 接收的 AT 指令，分发到对应模块处理
 ******************************************************************************/

/* AT 指令列表 */
static void at_handle_basic(void)   { uart_log_print("OK\r\n"); }

static void at_handle_info(void)
{
    uart_log_print("=== Module Info ===\r\n");
    uart_log_printf("  Firmware    : %s_%s\r\n", FW_PREFIX, FW_DATETIME);
    uart_log_printf("  Version     : %s\r\n", FW_VERSION);
    uart_log_printf("  Chip        : TLSR8258F512ET32\r\n");
    uart_log_printf("  Clock       : %d MHz\r\n", CLOCK_SYS_CLOCK_HZ / 1000000);
    uart_log_printf("  UART        : TX=PD7 RX=PA0 %d baud\r\n", g_config.uart_baud);
    uart_log_printf("  Log Level   : %d\r\n", g_config.log_level);

    uart_log_print("\r\n=== BLE GATT ===\r\n");
    char buf[16];
    ble_gatt_get_info(buf, sizeof(buf));

    uart_log_print("\r\n=== BLE Mesh ===\r\n");
    ble_mesh_get_status(buf, sizeof(buf));

    uart_log_print("\r\n=== GATT Attributes ===\r\n");
    uart_log_printf("  Total Handles : %d\r\n", ATT_END_H - 1);
    uart_log_printf("  [0x0001] GAP Service\r\n");
    uart_log_printf("  [0x0008] GATT Service\r\n");
    uart_log_printf("  [0x000C] Device Information\r\n");
    uart_log_printf("  [0x000F] DH Custom (SPP-like)\r\n");
    uart_log_printf("  [0x0017] OTA Service\r\n");
    uart_log_print("OK\r\n");
}

static void at_handle_status(void)
{
    uart_log_print("=== Module Status ===\r\n");
    char buf[16];
    ble_gatt_get_status(buf, sizeof(buf));
    ble_mesh_get_status(buf, sizeof(buf));
    uart_log_printf("  UART RX Buf : head=%d tail=%d\r\n", uart_rx_head, uart_rx_tail);
    uart_log_print("OK\r\n");
}

static void at_handle_bc(void)      { ble_gatt_start_adv(); uart_log_print("OK\r\n"); }
static void at_handle_offbc(void)   { ble_gatt_stop_adv();  uart_log_print("OK\r\n"); }

static void at_handle_mesh_on(void)
{
    g_config.mesh_enable = 1;
    config_save();
    ble_mesh_init();
    uart_log_print("OK\r\n");
}

static void at_handle_mesh_off(void)
{
    g_config.mesh_enable = 0;
    config_save();
    uart_log_print("[MESH] Disabled\r\nOK\r\n");
}

static void at_handle_log(void)
{
    uart_log_printf("Log level: %d (0=NONE 1=ERR 2=WARN 3=INFO 4=DEBUG)\r\n", g_config.log_level);
    uart_log_print("OK\r\n");
}

static void at_process_line(u8 *cmd, u8 len)
{
    cmd[len] = 0;
    while (len > 0 && (cmd[len-1]=='\r'||cmd[len-1]=='\n'||cmd[len-1]==' ')) cmd[--len]=0;
    if (len == 0) return;

    if (len==2 && cmd[0]=='A' && cmd[1]=='T')                    { at_handle_basic(); }
    else if (len>=8 && memcmp(cmd,"AT+INFO",7)==0)                { at_handle_info(); }
    else if (len>=10 && memcmp(cmd,"AT+STATUS",9)==0)             { at_handle_status(); }
    else if (len>=5 && memcmp(cmd,"AT+BC",5)==0)                  { at_handle_bc(); }
    else if (len>=8 && memcmp(cmd,"AT+OFFBC",8)==0)               { at_handle_offbc(); }
    else if (len>=10 && memcmp(cmd,"AT+MESH=ON",10)==0)           { at_handle_mesh_on(); }
    else if (len>=11 && memcmp(cmd,"AT+MESH=OFF",11)==0)          { at_handle_mesh_off(); }
    else if (len>=5 && memcmp(cmd,"AT+LOG",6)==0)                 { at_handle_log(); }
    else if (len>=9 && memcmp(cmd,"AT+LOG=",7)==0) {
        g_config.log_level = cmd[7] - '0';
        g_config.log_level = (g_config.log_level > LOG_LEVEL_DEBUG) ? LOG_LEVEL_DEBUG : g_config.log_level;
        config_save();
        uart_log_printf("Log level set to %d\r\nOK\r\n", g_config.log_level);
    }
    else if (len>=12 && memcmp(cmd,"AT+MESH=ADDR,",13)==0) {
        /* AT+MESH=ADDR,0x0001 */
        /* TODO: parse and set mesh address */
        uart_log_print("OK\r\n");
    }
    else { uart_log_print("ERROR: Unknown command\r\n"); }
}

void at_cmd_init(void)
{
    LOGI(LOG_TAG_AT, "AT command module ready");
    LOGI(LOG_TAG_AT, "Commands: AT AT+INFO AT+STATUS AT+BC AT+OFFBC AT+MESH=ON/OFF AT+LOG");
}

void at_cmd_process(void)
{
    static u8 line_buf[256];
    static u8 line_len = 0;

    while (uart_rx_available()) {
        u8 c = uart_rx_read();
        if (c == '\r' || c == '\n') {
            if (line_len > 0) {
                at_process_line(line_buf, line_len);
                line_len = 0;
            }
        } else if (line_len < sizeof(line_buf) - 1) {
            line_buf[line_len++] = c;
        }
    }
}

/******************************************************************************
 *                         (6) 主程序入口
 *
 *  初始化顺序：系统时钟 → UART → 配置 → BLE GATT → Mesh → AT
 *  主循环：BLE 调度 + UART 轮询 + AT 处理 + Mesh 处理
 ******************************************************************************/

_attribute_ram_code_ void irq_handler(void)
{
    blc_sdk_irq_handler();
}

int main(void)
{
    /* ---- 系统初始化 ---- */
    cpu_wakeup_init();
    clock_init(SYS_CLK_TYPE);
    rf_drv_ble_init();
    gpio_init(1);

    /* ---- (4) 配置管理（先加载配置，后续模块使用） ---- */
    config_init();

    /* ---- (1) UART 通信 ---- */
    hal_uart_init();
    delay_ms(50);

    uart_log_print("\r\n========================================\r\n");
    uart_log_printf("  DH BLE Device v%s\r\n", FW_VERSION);
    uart_log_printf("  Firmware: %s_%s\r\n", FW_PREFIX, FW_DATETIME);
    uart_log_printf("  Chip: TLSR8258F512ET32\r\n");
    uart_log_printf("  UART: TX=PD7 RX=PA0 %d baud\r\n", g_config.uart_baud);
    uart_log_print("========================================\r\n");

    /* ---- MAC 地址 ---- */
    blc_readFlashSize_autoConfigCustomFlashSector();
    blc_app_loadCustomizedParameters_normal();
    blc_initMacAddress(flash_sector_mac_address, g_mac, NULL);
    LOGI(LOG_TAG_MAIN, "MAC: %02X:%02X:%02X:%02X:%02X:%02X",
         g_mac[5], g_mac[4], g_mac[3], g_mac[2], g_mac[1], g_mac[0]);

    /* ---- (2) BLE GATT 通信 ---- */
    ble_gatt_init(g_mac);
    ble_gatt_start_adv();

    /* ---- (3) BLE Mesh 通信 ---- */
    ble_mesh_init();

    /* ---- (5) AT 指令管理 ---- */
    at_cmd_init();

    /* ---- 启动完成 ---- */
    uart_log_print("\r\n[MAIN] === INIT COMPLETE ===\r\n");
    uart_log_print("[MAIN] Modules: UART | BLE_GATT | BLE_Mesh | Config | AT\r\n");
    uart_log_print("[MAIN] Ready.\r\n\r\n");

    irq_enable();

    /* ---- 主循环 ---- */
    while (1) {
        /* BLE GATT 调度 */
        blc_sdk_main_loop();

        /* (1) UART 接收处理 */
        hal_uart_process();

        /* (5) AT 指令处理 */
        at_cmd_process();

        /* (3) Mesh 处理 */
        ble_mesh_process();
    }

    return 0;
}
