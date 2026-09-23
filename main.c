/******************************************************************************
 * @file    main.c
 * @brief   DH BLE Device - TLSR8258F512ET32
 *          BLE peripheral + UART AT commands
 *          Based on working acl_peripheral_demo structure
 ******************************************************************************/
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "app_config.h"
#include "app_buffer.h"
#include "app_att.h"

/* ---- UART config ---- */
#define UART_TX_PIN   UART_TX_PD7
#define UART_RX_PIN   UART_RX_PA0
#define UART_BAUD     115200
#define AT_BUF_SIZE   256

/* ---- Firmware info ---- */
#define FW_PREFIX     "dh_device"
#define FW_DATETIME   "20260923_090300"
#define FW_VERSION    "1.0.0"

/* ---- Global state ---- */
static u8 g_mac[6];
static char g_devname[32] = "dh_device_0000";
static u8 g_adv_enabled = 0;
static u8 g_connected = 0;

/* ---- AT command buffer ---- */
static u8 at_buf[AT_BUF_SIZE];
static u8 at_len = 0;

/* ---- OTA ---- */
_attribute_ble_data_retention_ static u8 ota_is_working = 0;
static void app_enter_ota_mode(void) { ota_is_working = 1; }

/* ---- UART helpers ---- */
static void uart_putc(u8 c)
{
    while ((reg_uart_buf_cnt >> 4) > 7);
    reg_uart_data_buf(uart_TxIndex) = c;
    uart_TxIndex = (uart_TxIndex + 1) & 0x03;
}

static void uart_puts(const char *s)
{
    while (*s) { uart_putc((u8)*s); s++; }
}

static void uart_print_num(u32 val, int base, int uppercase)
{
    char buf[10];
    int i = 0;
    if (val == 0) { uart_putc('0'); return; }
    while (val > 0) {
        int d = val % base;
        buf[i++] = (d < 10) ? ('0' + d) : ((uppercase ? 'A' : 'a') + d - 10);
        val /= base;
    }
    while (i > 0) uart_putc((u8)buf[--i]);
}

static void uart_printf(const char *fmt, ...)
{
    __builtin_va_list args;
    __builtin_va_start(args, fmt);
    const char *p = fmt;
    while (*p) {
        if (*p != '%') { uart_putc((u8)*p++); continue; }
        p++;
        switch (*p) {
            case 's': { const char *s = __builtin_va_arg(args, const char*); if (!s) s="(null)"; while(*s) uart_putc((u8)*s++); break; }
            case 'd': { int v = __builtin_va_arg(args, int); if(v<0){uart_putc('-');v=-v;} uart_print_num(v,10,0); break; }
            case 'u': { uart_print_num(__builtin_va_arg(args,u32),10,0); break; }
            case 'x': { uart_print_num(__builtin_va_arg(args,u32),16,0); break; }
            case 'X': { uart_print_num(__builtin_va_arg(args,u32),16,1); break; }
            case 'c': { uart_putc((u8)__builtin_va_arg(args,int)); break; }
            case '%': { uart_putc('%'); break; }
            default: uart_putc('%'); uart_putc((u8)*p); break;
        }
        p++;
    }
    __builtin_va_end(args);
}

static void delay_ms(u32 ms)
{
    u32 t0 = clock_time();
    while ((u32)(clock_time() - t0) < ms * CLOCK_SYS_CLOCK_HZ / 1000);
}

/* ---- BLE callbacks ---- */
static int app_controller_event_callback(u32 h, u8 *p, int n)
{
    if (h & HCI_FLAG_EVENT_BT_STD) {
        u8 evt = h & 0xff;
        if (evt == HCI_EVT_DISCONNECTION_COMPLETE) {
            g_connected = 0;
            uart_puts("[BLE] Disconnected\r\n");
        } else if (evt == HCI_EVT_LE_META) {
            if (p[0] == HCI_SUB_EVT_LE_CONNECTION_COMPLETE) {
                g_connected = 1;
                uart_puts("[BLE] Connected\r\n");
                hci_le_connectionCompleteEvt_t *pc = (hci_le_connectionCompleteEvt_t *)p;
                if (pc->role == ACL_ROLE_PERIPHERAL)
                    bls_l2cap_requestConnParamUpdate(pc->connHandle, 20, 40, 99, 400);
            } else if (p[0] == HCI_SUB_EVT_LE_CONNECTION_UPDATE_COMPLETE) {
                uart_puts("[BLE] ConnParam updated\r\n");
            }
        }
    }
    return 0;
}

static int app_host_event_callback(u32 h, u8 *para, int n)
{
    u8 evt = h & 0xFF;
    if (evt == GAP_EVT_ATT_EXCHANGE_MTU) uart_puts("[BLE] MTU exchanged\r\n");
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

/* ---- AT command handlers ---- */
static void at_cmd_info(void)
{
    uart_printf("=== Module Info ===\r\n");
    uart_printf("  Device Name : %s\r\n", g_devname);
    uart_printf("  MAC         : %02X:%02X:%02X:%02X:%02X:%02X\r\n", g_mac[5],g_mac[4],g_mac[3],g_mac[2],g_mac[1],g_mac[0]);
    uart_printf("  MAC ID      : %02X%02X\r\n", g_mac[1], g_mac[0]);
    uart_printf("  Chip        : TLSR8258F512ET32\r\n");
    uart_printf("  Firmware    : %s_%s\r\n", FW_PREFIX, FW_DATETIME);
    uart_printf("  Version     : %s\r\n", FW_VERSION);
    uart_printf("  Clock       : %d MHz\r\n", CLOCK_SYS_CLOCK_HZ / 1000000);
    uart_printf("\r\n=== BLE Config ===\r\n");
    uart_printf("  ACL Periph Max  : %d\r\n", ACL_PERIPHR_MAX_NUM);
    uart_printf("  SMP Enable      : %d\r\n", ACL_PERIPHR_SMP_ENABLE);
    uart_printf("  OTA Enable      : %d\r\n", BLE_OTA_SERVER_ENABLE);
    uart_printf("  ATT MTU         : 23\r\n");
    uart_printf("  RF Power        : P0dBm\r\n");
    uart_printf("\r\n=== GAP Config ===\r\n");
    uart_printf("  Adv Interval    : 30ms\r\n");
    uart_printf("  Adv Type        : connectable undirected\r\n");
    uart_printf("  Security        : No Security\r\n");
    uart_printf("\r\n=== GATT Attributes ===\r\n");
    uart_printf("  Total Handles   : %d\r\n", ATT_END_H - 1);
    uart_printf("  [0x0001] GAP Service\r\n");
    uart_printf("  [0x0008] GATT Service\r\n");
    uart_printf("  [0x000C] Device Information\r\n");
    uart_printf("  [0x000F] DH Custom (SPP-like)\r\n");
    uart_printf("  [0x0017] OTA Service\r\n");
    uart_puts("OK\r\n");
}

static void at_cmd_status(void)
{
    uart_printf("=== Module Status ===\r\n");
    uart_printf("  BLE State   : %s\r\n", g_connected ? "CONNECTED" : "IDLE");
    uart_printf("  ADV State   : %s\r\n", g_adv_enabled ? "RUNNING" : "STOPPED");
    uart_printf("  Device Name : %s\r\n", g_devname);
    uart_printf("  MAC         : %02X:%02X:%02X:%02X:%02X:%02X\r\n", g_mac[5],g_mac[4],g_mac[3],g_mac[2],g_mac[1],g_mac[0]);
    uart_printf("  UART        : TX=PD7 RX=PA0 %d baud\r\n", UART_BAUD);
    uart_puts("OK\r\n");
}

static void at_cmd_start_adv(void)
{
    if (g_adv_enabled) { uart_puts("[BLE] ADV already running\r\nOK\r\n"); return; }
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
    blc_ll_setAdvParam(ADV_INTERVAL_30MS, ADV_INTERVAL_30MS, ADV_TYPE_CONNECTABLE_UNDIRECTED,
                        OWN_ADDRESS_PUBLIC, 0, NULL, BLT_ENABLE_ADV_ALL, ADV_FP_NONE);
    blc_ll_setAdvEnable(BLC_ADV_ENABLE);
    g_adv_enabled = 1;
    uart_printf("[BLE] ADV started: %s\r\n", g_devname);
    uart_printf("[BLE] MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", g_mac[5],g_mac[4],g_mac[3],g_mac[2],g_mac[1],g_mac[0]);
    uart_puts("OK\r\n");
}

static void at_cmd_stop_adv(void)
{
    if (!g_adv_enabled) { uart_puts("[BLE] ADV already stopped\r\nOK\r\n"); return; }
    blc_ll_setAdvEnable(BLC_ADV_DISABLE);
    g_adv_enabled = 0;
    uart_puts("[BLE] ADV stopped\r\nOK\r\n");
}

static void at_process(u8 *cmd, u8 len)
{
    cmd[len] = 0;
    while (len > 0 && (cmd[len-1]=='\r'||cmd[len-1]=='\n'||cmd[len-1]==' ')) cmd[--len]=0;
    if (len == 0) return;
    if (len==2 && cmd[0]=='A' && cmd[1]=='T') { uart_puts("OK\r\n"); }
    else if (len>=8 && memcmp(cmd,"AT+INFO",7)==0) { at_cmd_info(); }
    else if (len>=10 && memcmp(cmd,"AT+STATUS",9)==0) { at_cmd_status(); }
    else if (len>=5 && memcmp(cmd,"AT+BC",5)==0) { at_cmd_start_adv(); }
    else if (len>=8 && memcmp(cmd,"AT+OFFBC",8)==0) { at_cmd_stop_adv(); }
    else { uart_puts("ERROR: Unknown command\r\n"); }
}

/* ---- IRQ handler ---- */
_attribute_ram_code_ void irq_handler(void)
{
    blc_sdk_irq_handler();
}

/* ---- Main ---- */
int main(void)
{
    /* === System init (same as acl_peripheral_demo) === */
    cpu_wakeup_init();
    clock_init(SYS_CLK_TYPE);
    rf_drv_ble_init();
    gpio_init(1);

    /* === UART init === */
    uart_gpio_set(UART_TX_PIN, UART_RX_PIN);
    uart_init_baudrate(UART_BAUD, CLOCK_SYS_CLOCK_HZ, PARITY_NONE, STOP_BIT_ONE);
    uart_TxIndex = 0;
    uart_RxIndex = 0;
    delay_ms(50);

    uart_puts("\r\n========================================\r\n");
    uart_puts("  DH BLE Device\r\n");
    uart_printf("  Firmware: %s_%s\r\n", FW_PREFIX, FW_DATETIME);
    uart_puts("  Chip: TLSR8258F512ET32\r\n");
    uart_puts("  UART: TX=PD7 RX=PA0 115200\r\n");
    uart_puts("========================================\r\n");

    /* === BLE initialization === */
    random_generator_init();
    blc_readFlashSize_autoConfigCustomFlashSector();
    blc_app_loadCustomizedParameters_normal();

    /* MAC address */
    blc_initMacAddress(flash_sector_mac_address, g_mac, NULL);
    uart_printf("[INIT] MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n", g_mac[5],g_mac[4],g_mac[3],g_mac[2],g_mac[1],g_mac[0]);

    /* Generate device name: dh_device_XXXX */
    { const char hx[]="0123456789ABCDEF";
      g_devname[0]='d';g_devname[1]='h';g_devname[2]='_';g_devname[3]='d';g_devname[4]='e';
      g_devname[5]='v';g_devname[6]='i';g_devname[7]='c';g_devname[8]='e';g_devname[9]='_';
      g_devname[10]=hx[(g_mac[1]>>4)&0xF]; g_devname[11]=hx[g_mac[1]&0xF];
      g_devname[12]=hx[(g_mac[0]>>4)&0xF]; g_devname[13]=hx[g_mac[0]&0xF];
      g_devname[14]='\0'; }
    uart_printf("[INIT] Device: %s\r\n", g_devname);

    /* BLE Link Layer */
    blc_ll_initBasicMCU();
    blc_ll_initStandby_module(g_mac);
    blc_ll_initLegacyAdvertising_module();
    blc_ll_initAclConnection_module();
    blc_ll_initAclPeriphrRole_module();
    blc_ll_setMaxConnectionNumber(ACL_CENTRAL_MAX_NUM, ACL_PERIPHR_MAX_NUM);
    blc_ll_setAclConnMaxOctetsNumber(ACL_CONN_MAX_RX_OCTETS, ACL_MASTER_MAX_TX_OCTETS, ACL_SLAVE_MAX_TX_OCTETS);
    blc_ll_initAclConnRxFifo(app_acl_rxfifo, ACL_RX_FIFO_SIZE, ACL_RX_FIFO_NUM);
    blc_ll_initAclPeriphrTxFifo(app_acl_slvTxfifo, ACL_SLAVE_TX_FIFO_SIZE, ACL_SLAVE_TX_FIFO_NUM, ACL_PERIPHR_MAX_NUM);
    uart_puts("[INIT] LL OK\r\n");

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
    uart_puts("[INIT] GAP/GATT OK\r\n");

    /* SMP */
    blc_smp_setSecurityLevel_slave(No_Security);
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

    /* Start advertising */
    at_cmd_start_adv();

    uart_puts("[INIT] ALL DONE\r\n");
    uart_puts("Commands: AT AT+INFO AT+STATUS AT+BC AT+OFFBC\r\n\r\n");

    irq_enable();

    /* === Main loop === */
    while (1) {
        blc_sdk_main_loop();

        /* Poll UART RX */
        while (reg_uart_buf_cnt & FLD_UART_RX_BUF_CNT) {
            u8 c = reg_uart_data_buf(uart_RxIndex);
            uart_RxIndex = (uart_RxIndex + 1) & 0x03;
            if (c == '\r' || c == '\n') {
                if (at_len > 0) { at_process(at_buf, at_len); at_len = 0; }
            } else if (at_len < AT_BUF_SIZE - 1) {
                at_buf[at_len++] = c;
            }
        }
    }
    return 0;
}
