/******************************************************************************
 * @file    app_buffer.c
 * @brief   BLE FIFO buffer allocation
 ******************************************************************************/
#include "tl_common.h"
#include "app_config.h"
#include "app_buffer.h"

/* ACL connection FIFOs */
u8 app_acl_rxfifo[ACL_RX_FIFO_SIZE * ACL_RX_FIFO_NUM] __attribute__((aligned(4)));
u8 app_acl_slvTxfifo[ACL_SLAVE_TX_FIFO_SIZE * ACL_SLAVE_TX_FIFO_NUM] __attribute__((aligned(4)));

/* L2CAP MTU FIFOs */
u8 mtu_s_rx_fifo[MTU_S_BUFF_SIZE_MAX] __attribute__((aligned(4)));
u8 mtu_s_tx_fifo[MTU_S_BUFF_SIZE_MAX] __attribute__((aligned(4)));
