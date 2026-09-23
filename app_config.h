#pragma once

#include "config.h"

/* BLE connection configuration */
#define ACL_CENTRAL_MAX_NUM                     0
#define ACL_PERIPHR_MAX_NUM                     1

/* Feature configuration - disable non-essential features */
#define ACL_PERIPHR_SMP_ENABLE                  0
#define BLE_OTA_SERVER_ENABLE                   1

#define BLE_APP_PM_ENABLE                       0
#define PM_DEEPSLEEP_RETENTION_ENABLE           0

#define APP_FLASH_PROTECTION_ENABLE             0
#define APP_BATT_CHECK_ENABLE                   0

/* Board select (TLSR825x) */
#if (CHIP_TYPE == CHIP_TYPE_825x)
    #define BOARD_SELECT                        BOARD_825X_EVK_C1T139A30
#endif

/* Disable UI */
#define UI_LED_ENABLE                           0
#define UI_KEYBOARD_ENABLE                      0

/* System clock: 32MHz */
#define CLOCK_SYS_CLOCK_HZ                      32000000

/* Disable SDK debug UART - we use our own hardware UART (PD7-TX, PA0-RX) */
#define UART_PRINT_DEBUG_ENABLE                 0

/* Deep sleep ana reg */
#define USED_DEEP_ANA_REG                       DEEP_ANA_REG0
#define LOW_BATT_FLG                            BIT(0)

#include "../common/default_config.h"
