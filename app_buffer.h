#ifndef APP_BUFFER_H_
#define APP_BUFFER_H_

#include "tl_common.h"
#include "app_config.h"

/* ACL connection parameters */
#define ACL_CONN_MAX_RX_OCTETS          27
#define ACL_MASTER_MAX_TX_OCTETS        27
#define ACL_SLAVE_MAX_TX_OCTETS         27

/* ACL RX FIFO */
#define ACL_RX_FIFO_SIZE                CAL_LL_ACL_RX_FIFO_SIZE(ACL_CONN_MAX_RX_OCTETS)
#define ACL_RX_FIFO_NUM                 8

/* ACL TX FIFO */
#define ACL_SLAVE_TX_FIFO_SIZE          CAL_LL_ACL_TX_FIFO_SIZE(ACL_SLAVE_MAX_TX_OCTETS)
#define ACL_SLAVE_TX_FIFO_NUM           8

extern u8 app_acl_rxfifo[];
extern u8 app_acl_slvTxfifo[];

/* L2CAP MTU buffers */
#define ATT_MTU_SLAVE_RX_MAX_SIZE       23
#define MTU_S_BUFF_SIZE_MAX             CAL_MTU_BUFF_SIZE(ATT_MTU_SLAVE_RX_MAX_SIZE)

extern u8 mtu_s_rx_fifo[];
extern u8 mtu_s_tx_fifo[];

#endif /* APP_BUFFER_H_ */
