/******************************************************************************
 * @file    app_att.c
 * @brief   GATT attribute table for DH BLE device
 *          Services: GAP, GATT, Device Information, DH Custom (SPP-like), OTA
 ******************************************************************************/
#include "tl_common.h"
#include "drivers.h"
#include "stack/ble/ble.h"
#include "app_att.h"

/* ---- UUID constants ---- */
static const u16 my_primaryServiceUUID   = GATT_UUID_PRIMARY_SERVICE;
static const u16 my_characterUUID        = GATT_UUID_CHARACTER;
static const u16 clientCharacterCfgUUID  = GATT_UUID_CLIENT_CHAR_CFG;
static const u16 userdesc_UUID           = GATT_UUID_CHAR_USER_DESC;

/* GAP */
static const u16 my_gapServiceUUID       = SERVICE_UUID_GENERIC_ACCESS;
static const u16 my_devNameUUID          = GATT_UUID_DEVICE_NAME;
static const u16 my_appearanceUIID       = GATT_UUID_APPEARANCE;
static const u16 my_periConnParamUUID    = GATT_UUID_PERI_CONN_PARAM;
static const u16 my_appearance           = GAP_APPEARE_UNKNOWN;

typedef struct {
    u16 intervalMin;
    u16 intervalMax;
    u16 latency;
    u16 timeout;
} gap_periConnectParams_t;

static const gap_periConnectParams_t my_periConnParameters = {20, 40, 0, 1000};

/* GATT */
static const u16 my_gattServiceUUID      = SERVICE_UUID_GENERIC_ATTRIBUTE;
static const u16 serviceChangeUUID       = GATT_UUID_SERVICE_CHANGE;
static u16 serviceChangeVal[2]           = {0};
static u8  serviceChangeCCC[2]           = {0, 0};

/* Device Information */
static const u16 my_devServiceUUID       = SERVICE_UUID_DEVICE_INFORMATION;
static const u16 my_PnPUUID              = CHARACTERISTIC_UUID_PNP_ID;
static const u8  my_PnPtrs[]             = {0x02, 0x8a, 0x24, 0x66, 0x82, 0x01, 0x00};

/* Device name - will be overwritten at init time from g_device_name */
static u8 my_devName[32] = "dh_device_0000";

/* ---- GAP attribute values ---- */
static const u8 my_devNameCharVal[5] = {
    CHAR_PROP_READ,
    U16_LO(GenericAccess_DeviceName_DP_H), U16_HI(GenericAccess_DeviceName_DP_H),
    U16_LO(GATT_UUID_DEVICE_NAME), U16_HI(GATT_UUID_DEVICE_NAME)
};
static const u8 my_appearanceCharVal[5] = {
    CHAR_PROP_READ,
    U16_LO(GenericAccess_Appearance_DP_H), U16_HI(GenericAccess_Appearance_DP_H),
    U16_LO(GATT_UUID_APPEARANCE), U16_HI(GATT_UUID_APPEARANCE)
};
static const u8 my_periConnParamCharVal[5] = {
    CHAR_PROP_READ,
    U16_LO(CONN_PARAM_DP_H), U16_HI(CONN_PARAM_DP_H),
    U16_LO(GATT_UUID_PERI_CONN_PARAM), U16_HI(GATT_UUID_PERI_CONN_PARAM)
};

/* ---- GATT attribute values ---- */
static const u8 my_serviceChangeCharVal[5] = {
    CHAR_PROP_INDICATE,
    U16_LO(GenericAttribute_ServiceChanged_DP_H), U16_HI(GenericAttribute_ServiceChanged_DP_H),
    U16_LO(GATT_UUID_SERVICE_CHANGE), U16_HI(GATT_UUID_SERVICE_CHANGE)
};

/* ---- Device Info attribute values ---- */
static const u8 my_PnCharVal[5] = {
    CHAR_PROP_READ,
    U16_LO(DeviceInformation_pnpID_DP_H), U16_HI(DeviceInformation_pnpID_DP_H),
    U16_LO(CHARACTERISTIC_UUID_PNP_ID), U16_HI(CHARACTERISTIC_UUID_PNP_ID)
};

/* ---- DH Custom Service (SPP-like data channel) ---- */
static const u8 DH_ServiceUUID[16]           = WRAPPING_BRACES(TELINK_SPP_UUID_SERVICE);
static const u8 DH_S2C_UUID[16]              = WRAPPING_BRACES(TELINK_SPP_DATA_SERVER2CLIENT);
static const u8 DH_C2S_UUID[16]              = WRAPPING_BRACES(TELINK_SPP_DATA_CLIENT2SERVER);

static u8 DH_S2C_CCC[2]                      = {0};
static u8 DH_S2C_Data[1]                     = {0};
static u8 DH_C2S_Data[1]                     = {0};

static const u8 DH_S2C_Desc[]                = "DH BLE S2C";
static const u8 DH_C2S_Desc[]                = "DH BLE C2S";

static const u8 DH_S2C_CharVal[19] = {
    CHAR_PROP_READ | CHAR_PROP_NOTIFY,
    U16_LO(DH_S2C_DP_H), U16_HI(DH_S2C_DP_H),
    TELINK_SPP_DATA_SERVER2CLIENT
};
static const u8 DH_C2S_CharVal[19] = {
    CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP,
    U16_LO(DH_C2S_DP_H), U16_HI(DH_C2S_DP_H),
    TELINK_SPP_DATA_CLIENT2SERVER
};

/* DH service write callback */
int dh_onReceiveData(u16 connHandle, void *p)
{
    /* Data received from phone via DH service */
    return 0;
}

/* ---- OTA Service ---- */
static const u8 my_OtaServiceUUID[16]    = WRAPPING_BRACES(TELINK_OTA_UUID_SERVICE);
static const u8 my_OtaUUID[16]           = WRAPPING_BRACES(TELINK_SPP_DATA_OTA);
static u8 my_OtaDataCCC[2]               = {0};
static u8 my_OtaData                     = 0x00;
static const u8 my_OtaName[]             = "OTA";

static const u8 my_OtaCharVal[19] = {
    CHAR_PROP_READ | CHAR_PROP_WRITE_WITHOUT_RSP | CHAR_PROP_NOTIFY,
    U16_LO(OTA_CMD_OUT_DP_H), U16_HI(OTA_CMD_OUT_DP_H),
    TELINK_SPP_DATA_OTA,
};

/* ---- OTA write callback (extern) ---- */
extern int otaWrite(u16 connHandle, void *p);

/******************************************************************************
 * GATT Attribute Table
 *****************************************************************************/
static const attribute_t my_Attributes[] = {

    {ATT_END_H - 1, 0, 0, 0, 0, 0},   /* total attribute count */

    /*=== 0001-0007 GAP Service ===*/
    {7, ATT_PERMISSIONS_READ, 2, 2,
     (u8*)(&my_primaryServiceUUID), (u8*)(&my_gapServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_devNameCharVal),
     (u8*)(&my_characterUUID), (u8*)(my_devNameCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_devName),
     (u8*)(&my_devNameUUID), (u8*)(my_devName), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_appearanceCharVal),
     (u8*)(&my_characterUUID), (u8*)(my_appearanceCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_appearance),
     (u8*)(&my_appearanceUIID), (u8*)(&my_appearance), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_periConnParamCharVal),
     (u8*)(&my_characterUUID), (u8*)(my_periConnParamCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_periConnParameters),
     (u8*)(&my_periConnParamUUID), (u8*)(&my_periConnParameters), 0},

    /*=== 0008-000B GATT Service ===*/
    {4, ATT_PERMISSIONS_READ, 2, 2,
     (u8*)(&my_primaryServiceUUID), (u8*)(&my_gattServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_serviceChangeCharVal),
     (u8*)(&my_characterUUID), (u8*)(my_serviceChangeCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(serviceChangeVal),
     (u8*)(&serviceChangeUUID), (u8*)(&serviceChangeVal), 0},
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(serviceChangeCCC),
     (u8*)(&clientCharacterCfgUUID), (u8*)(serviceChangeCCC), 0},

    /*=== 000C-000E Device Information Service ===*/
    {3, ATT_PERMISSIONS_READ, 2, 2,
     (u8*)(&my_primaryServiceUUID), (u8*)(&my_devServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_PnCharVal),
     (u8*)(&my_characterUUID), (u8*)(my_PnCharVal), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_PnPtrs),
     (u8*)(&my_PnPUUID), (u8*)(my_PnPtrs), 0},

    /*=== 000F-0016 DH Custom Service (SPP-like) ===*/
    {8, ATT_PERMISSIONS_READ, 2, 16,
     (u8*)(&my_primaryServiceUUID), (u8*)(&DH_ServiceUUID), 0},
    /* Server -> Client */
    {0, ATT_PERMISSIONS_READ, 2, sizeof(DH_S2C_CharVal),
     (u8*)(&my_characterUUID), (u8*)(DH_S2C_CharVal), 0},
    {0, ATT_PERMISSIONS_READ, 16, sizeof(DH_S2C_Data),
     (u8*)(&DH_S2C_UUID), (u8*)(DH_S2C_Data), 0},
    {0, ATT_PERMISSIONS_RDWR, 2, 2,
     (u8*)(&clientCharacterCfgUUID), (u8*)(&DH_S2C_CCC), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(DH_S2C_Desc),
     (u8*)(&userdesc_UUID), (u8*)(DH_S2C_Desc), 0},
    /* Client -> Server */
    {0, ATT_PERMISSIONS_READ, 2, sizeof(DH_C2S_CharVal),
     (u8*)(&my_characterUUID), (u8*)(DH_C2S_CharVal), 0},
    {0, ATT_PERMISSIONS_RDWR, 16, sizeof(DH_C2S_Data),
     (u8*)(&DH_C2S_UUID), (u8*)(DH_C2S_Data), (att_readwrite_callback_t)&dh_onReceiveData},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(DH_C2S_Desc),
     (u8*)(&userdesc_UUID), (u8*)(DH_C2S_Desc), 0},

    /*=== 0017-001B OTA Service ===*/
    {5, ATT_PERMISSIONS_READ, 2, 16,
     (u8*)(&my_primaryServiceUUID), (u8*)(&my_OtaServiceUUID), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_OtaCharVal),
     (u8*)(&my_characterUUID), (u8*)(my_OtaCharVal), 0},
    {0, ATT_PERMISSIONS_RDWR, 16, sizeof(my_OtaData),
     (u8*)(&my_OtaUUID), (&my_OtaData), &otaWrite, NULL},
    {0, ATT_PERMISSIONS_RDWR, 2, sizeof(my_OtaDataCCC),
     (u8*)(&clientCharacterCfgUUID), (u8*)(my_OtaDataCCC), 0},
    {0, ATT_PERMISSIONS_READ, 2, sizeof(my_OtaName),
     (u8*)(&userdesc_UUID), (u8*)(my_OtaName), 0},
};

/******************************************************************************
 * Register ATT table with BLE stack and update device name
 *****************************************************************************/
void my_gatt_init(void)
{
    bls_att_setAttributeTable((u8 *)my_Attributes);
}
