#ifndef APP_ATT_H_
#define APP_ATT_H_

#include "tl_common.h"

typedef enum
{
    ATT_H_START = 0,

    /* GAP Service */
    GenericAccess_PS_H,
    GenericAccess_DeviceName_CD_H,
    GenericAccess_DeviceName_DP_H,
    GenericAccess_Appearance_CD_H,
    GenericAccess_Appearance_DP_H,
    CONN_PARAM_CD_H,
    CONN_PARAM_DP_H,

    /* GATT Service */
    GenericAttribute_PS_H,
    GenericAttribute_ServiceChanged_CD_H,
    GenericAttribute_ServiceChanged_DP_H,
    GenericAttribute_ServiceChanged_CCB_H,

    /* Device Information Service */
    DeviceInformation_PS_H,
    DeviceInformation_pnpID_CD_H,
    DeviceInformation_pnpID_DP_H,

    /* Custom DH Service (SPP-like data channel) */
    DH_SERVICE_PS_H,
    DH_S2C_CD_H,               /* Server->Client characteristic declaration */
    DH_S2C_DP_H,               /* Server->Client data value */
    DH_S2C_CCB_H,              /* Server->Client CCCD */
    DH_S2C_DESC_H,             /* Server->Client user description */
    DH_C2S_CD_H,               /* Client->Server characteristic declaration */
    DH_C2S_DP_H,               /* Client->Server data value */
    DH_C2S_DESC_H,             /* Client->Server user description */

    /* OTA Service */
    OTA_PS_H,
    OTA_CMD_OUT_CD_H,
    OTA_CMD_OUT_DP_H,
    OTA_CMD_OUT_CCB_H,
    OTA_CMD_OUT_DESC_H,

    ATT_END_H,
} ATT_HANDLE;

void my_gatt_init(void);

#endif /* APP_ATT_H_ */
