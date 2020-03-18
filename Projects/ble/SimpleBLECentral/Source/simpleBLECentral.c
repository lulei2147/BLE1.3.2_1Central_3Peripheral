/**************************************************************************************************
  Filename:       simpleBLECentral.c
  Revised:        $Date: 2011-06-20 11:57:59 -0700 (Mon, 20 Jun 2011) $
  Revision:       $Revision: 28 $

  Description:    This file contains the Simple BLE Central sample application 
                  for use with the CC2540 Bluetooth Low Energy Protocol Stack.

  Copyright 2010 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED “AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
 * INCLUDES
 */

#include "bcomdef.h"
#include "OSAL.h"
#include "OSAL_PwrMgr.h"
#include "OnBoard.h"
#include "hal_led.h"
#include "hal_key.h"
#include "hal_lcd.h"
#include "gatt.h"
#include "ll.h"
#include "hci.h"
#include "gapgattserver.h"
#include "gattservapp.h"
#include "central.h"
#include "gapbondmgr.h"
#include "simpleGATTprofile.h"
#include "simpleBLECentral.h"

#include "npi.h"
#include "stdio.h"

/*********************************************************************
 * MACROS
 */

#define _BIT0                                 (0x01)
#define _BIT1                                 (0x02)
#define _BIT2                                 (0x04)
#define _BIT3                                 (0x08)
#define _BIT4                                 (0x10)
#define _BIT5                                 (0x20)
#define _BIT6                                 (0x40)
#define _BIT7                                 (0x80)

// Length of bd addr as a string
#define B_ADDR_STR_LEN                        15

#define BLE_DEV_MAC_ADDR_LEN                  12

#define BLE_PENDING_DATA_MAX                  32

#define SET_BLE_WRITE_CYCLE_FLAG()             (simpleBLEFlag |= _BIT0)
#define GET_BLE_WRITE_CYCLE_FLAG()             ((bool)(simpleBLEFlag & _BIT0))
#define CLR_BLE_WRITE_CYCLE_FLAG()             (simpleBLEFlag &= ~_BIT0)

// replace variable simpleBLEScanning
#define SET_SIMPLEBLE_SCANNING_FLAG()          (simpleBLEFlag |= _BIT1)
#define GET_SIMPLEBLE_SCANNING_FLAG()          ((bool)(simpleBLEFlag & _BIT1))
#define CLR_SIMPLEBLE_SCANNING_FLAG()          (simpleBLEFlag &= ~_BIT1)

/*********************************************************************
 * CONSTANTS
 */

#define SBP_PERIODIC_EVT_AUTO_CONNECT         2100

// Maximum number of scan responses
#define DEFAULT_MAX_SCAN_RES                  3//8

// Scan duration in ms
#define DEFAULT_SCAN_DURATION                 1000

// Discovey mode (limited, general, all)
#define DEFAULT_DISCOVERY_MODE                DEVDISC_MODE_ALL

// TRUE to use active scan
#define DEFAULT_DISCOVERY_ACTIVE_SCAN         TRUE

// TRUE to use white list during discovery
#define DEFAULT_DISCOVERY_WHITE_LIST          FALSE

// TRUE to use high scan duty cycle when creating link
#define DEFAULT_LINK_HIGH_DUTY_CYCLE          FALSE

// TRUE to use white list when creating link
#define DEFAULT_LINK_WHITE_LIST               FALSE

// Default RSSI polling period in ms
#define DEFAULT_RSSI_PERIOD                   1000

// Whether to enable automatic parameter update request when a connection is formed
#define DEFAULT_ENABLE_UPDATE_REQUEST         FALSE

// Minimum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MIN_CONN_INTERVAL      8//400

// Maximum connection interval (units of 1.25ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_MAX_CONN_INTERVAL      800

// Slave latency to use if automatic parameter update request is enabled
#define DEFAULT_UPDATE_SLAVE_LATENCY          0

// Supervision timeout value (units of 10ms) if automatic parameter update request is enabled
#define DEFAULT_UPDATE_CONN_TIMEOUT           100//200//300

// Default passcode
#define DEFAULT_PASSCODE                      19655

// Default GAP pairing mode
#define DEFAULT_PAIRING_MODE                  GAPBOND_PAIRING_MODE_WAIT_FOR_REQ

// Default MITM mode (TRUE to require passcode or OOB when pairing)
#define DEFAULT_MITM_MODE                     FALSE

// Default bonding mode, TRUE to bond
#define DEFAULT_BONDING_MODE                  TRUE

// Default GAP bonding I/O capabilities
#define DEFAULT_IO_CAPABILITIES               GAPBOND_IO_CAP_DISPLAY_ONLY

// Default service discovery timer delay in ms
#define DEFAULT_SVC_DISCOVERY_DELAY           1//1000

// TRUE to filter discovery results on desired service UUID
#define DEFAULT_DEV_DISC_BY_SVC_UUID          TRUE

// Application states
enum
{
  BLE_STATE_IDLE,
  BLE_STATE_CONNECTING,
  BLE_STATE_CONNECTED,
  BLE_STATE_DISCONNECTING
};

// Discovery states
enum
{
  BLE_DISC_STATE_IDLE,                // Idle
  BLE_DISC_STATE_SVC,                 // Service discovery
  BLE_DISC_STATE_CHAR1,                // Characteristic discovery
  BLE_DISC_STATE_CHAR2,
  BLE_DISC_STATE_CHAR3,
  BLE_DISC_STATE_CHAR4,
  BLE_DISC_STATE_CHAR5,
  BLE_DISC_STATE_CHAR6,
};

enum
{
  BLE_CHAR1 = 0,
  BLE_CHAR2,
  BLE_CHAR3,
  BLE_CHAR4,
  BLE_CHAR5,
  BLE_CHAR6 = 1,
};

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

// Task ID for internal task/event processing
static uint8 simpleBLETaskId;

// GAP GATT Attributes
static const uint8 simpleBLEDeviceName[GAP_DEVICE_NAME_LEN] = "Simple BLE Central";

// Number of scan results and scan result index
static uint8 simpleBLEScanRes;
static uint8 simpleBLEScanIdx;

// Scan result list
static gapDevRec_t simpleBLEDevList[DEFAULT_MAX_SCAN_RES];

// Scanning state
//static uint8 simpleBLEScanning = FALSE;

// Value to write
static uint8 simpleBLECharVal = 0;

static uint8 simpleBLEPendingDataBuffer[BLE_PENDING_DATA_MAX];
static uint8 simpleBLESendDataCycleNum = 0;
static uint8 simpleBLELastSendDataLen = 0;
static uint8 simpleBLECycleIdx = 0;
uint8 simpleBLEFlag = 0;

//#define BYTE_RATE_DEBUG

#ifdef BYTE_RATE_DEBUG
uint32 simpleByteRateTime[2];
uint8 simpleByteNum = 0;
float simpleBLEByteRate = 0;
#endif

typedef struct
{
  // Connection handle of current connection
  uint16 simpleBLEConnHandle;
  uint8 simpleBLEDevIdx;
  
  // Application state
  uint8 simpleBLEState;
  
  // Discovery state
  uint8 simpleBLEDiscState;
  
  // Discovered service start and end handle
  uint16 simpleBLESvcStartHdl;
  uint16 simpleBLESvcEndHdl;
  
  // Discovered characteristic handle
  // To save space, set the size of array based on the number of characteristic handle
  uint16 simpleBLECharHdl[2];
  
  // Value read/write procedure state
  bool simpleBLEDoWrite;
}BLE_DEV_INFO;

const BLE_DEV_INFO gDevInfoInitVal = 
{
  // Connection handle of current connection
  GAP_CONNHANDLE_INIT,
  0xff, // invaild value
  
  // Application state
  BLE_STATE_IDLE,
  
  // Discovery state
  BLE_DISC_STATE_IDLE,
  
  // Discovered service start and end handle
  0,
  0,
  
  // Discovered characteristic handle
  {0, 0},
  
  // Value read/write procedure state
  TRUE,
};

static BLE_DEV_INFO stDevInfo[DEFAULT_MAX_SCAN_RES] = {0};

/*********************************************************************
 * LOCAL FUNCTIONS
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg );
static void simpleBLECentralRssiCB( uint16 connHandle, int8  rssi );
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent );
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs );
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status );
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys );
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg );
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg );
static void simpleBLECentralStartDiscovery( void );
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen );
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType );
char *bdAddr2Str ( uint8 *pAddr );

static void performPeriodAutoConnect(void);
static BLE_DEV_INFO *simpleBLECentralGetDevHandle(uint16 connHandle);
void simpleBLE_NpiSerialCallback(uint8 port, uint8 events);
void simpleBLECentarlSendData2Peripheral(uint8 devIdx, uint8 charIdx, uint8 *pData, uint8 len);
bool simpleGetDeviceWriteFlag(void);
static bool simpleStrCmp(uint8 *pStr1, uint8 *pStr2, uint8 len);

/*********************************************************************
 * PROFILE CALLBACKS
 */

// GAP Role Callbacks
static const gapCentralRoleCB_t simpleBLERoleCB =
{
  simpleBLECentralRssiCB,       // RSSI callback
  simpleBLECentralEventCB       // Event callback
};

// Bond Manager Callbacks
static const gapBondCBs_t simpleBLEBondCB =
{
  simpleBLECentralPasscodeCB,
  simpleBLECentralPairStateCB
};

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SimpleBLECentral_Init
 *
 * @brief   Initialization function for the Simple BLE Central App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notification).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SimpleBLECentral_Init( uint8 task_id )
{
  simpleBLETaskId = task_id;
  
  // Clear LCD
#if(HAL_LCD == TRUE) 
  HalLcd_HW_Clear(0,127,0,7);
#endif
  
  // Init UART
  NPI_InitTransport(simpleBLE_NpiSerialCallback);
  
  // Init Device Info struct
  for(int i = 0; i < DEFAULT_MAX_SCAN_RES; i++)
  {
    osal_memcpy(&stDevInfo[i], &gDevInfoInitVal, sizeof(BLE_DEV_INFO));
  }

  // Setup Central Profile
  {
    uint8 scanRes = DEFAULT_MAX_SCAN_RES;
    GAPCentralRole_SetParameter ( GAPCENTRALROLE_MAX_SCAN_RES, sizeof( uint8 ), &scanRes );
  }
  
  // Setup GAP
  GAP_SetParamValue( TGAP_GEN_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GAP_SetParamValue( TGAP_LIM_DISC_SCAN, DEFAULT_SCAN_DURATION );
  GGS_SetParameter( GGS_DEVICE_NAME_ATT, GAP_DEVICE_NAME_LEN, (uint8 *) simpleBLEDeviceName );

  // Setup the GAP Bond Manager
  {
    uint32 passkey = DEFAULT_PASSCODE;
    uint8 pairMode = DEFAULT_PAIRING_MODE;
    uint8 mitm = DEFAULT_MITM_MODE;
    uint8 ioCap = DEFAULT_IO_CAPABILITIES;
    uint8 bonding = DEFAULT_BONDING_MODE;
    GAPBondMgr_SetParameter( GAPBOND_DEFAULT_PASSCODE, sizeof( uint32 ), &passkey );
    GAPBondMgr_SetParameter( GAPBOND_PAIRING_MODE, sizeof( uint8 ), &pairMode );
    GAPBondMgr_SetParameter( GAPBOND_MITM_PROTECTION, sizeof( uint8 ), &mitm );
    GAPBondMgr_SetParameter( GAPBOND_IO_CAPABILITIES, sizeof( uint8 ), &ioCap );
    GAPBondMgr_SetParameter( GAPBOND_BONDING_ENABLED, sizeof( uint8 ), &bonding );
  }  

  // Initialize GATT Client
  VOID GATT_InitClient();

  // Register to receive incoming ATT Indications/Notifications
  GATT_RegisterForInd( simpleBLETaskId );

  // Initialize GATT attributes
  GGS_AddService( GATT_ALL_SERVICES );         // GAP
  GATTServApp_AddService( GATT_ALL_SERVICES ); // GATT attributes

  // Register for all key events - This app will handle all key events
  RegisterForKeys( simpleBLETaskId );
  
  // makes sure LEDs are off
  HalLedSet( (HAL_LED_1 | HAL_LED_2), HAL_LED_MODE_OFF );
  
  // Setup a delayed profile startup
  osal_set_event( simpleBLETaskId, START_DEVICE_EVT );
}

/*********************************************************************
 * @fn      SimpleBLECentral_ProcessEvent
 *
 * @brief   Simple BLE Central Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  events not processed
 */
uint16 SimpleBLECentral_ProcessEvent( uint8 task_id, uint16 events )
{
  
  VOID task_id; // OSAL required parameter that isn't used in this function
  
  if ( events & SYS_EVENT_MSG )
  {
    uint8 *pMsg;

    if ( (pMsg = osal_msg_receive( simpleBLETaskId )) != NULL )
    {
      simpleBLECentral_ProcessOSALMsg( (osal_event_hdr_t *)pMsg );

      // Release the OSAL message
      VOID osal_msg_deallocate( pMsg );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  if ( events & START_DEVICE_EVT )
  {
    // Start the Device
    VOID GAPCentralRole_StartDevice( (gapCentralRoleCB_t *) &simpleBLERoleCB );

    // Register with bond manager after starting device
    GAPBondMgr_Register( (gapBondCBs_t *) &simpleBLEBondCB );

    return ( events ^ START_DEVICE_EVT );
  }
  
  if(events & SBP_AUTO_CONNECT_EVT)
  {
    if((simpleBLEScanRes > 0) && (simpleBLEScanIdx < simpleBLEScanRes))
    {
      simpleBLEScanIdx++;
      
      if(simpleBLEScanIdx >= simpleBLEScanRes)
      {
        simpleBLEScanIdx = 0;
      }
      else
      {
        performPeriodAutoConnect();
        osal_start_timerEx(simpleBLETaskId, SBP_AUTO_CONNECT_EVT, SBP_PERIODIC_EVT_AUTO_CONNECT);
      }
    }
    
    return (events ^ SBP_AUTO_CONNECT_EVT);
  }

  if ( events & START_DISCOVERY_EVT )
  {
    simpleBLECentralStartDiscovery( );
    
    return ( events ^ START_DISCOVERY_EVT );
  }
  
  // Discard unknown events
  return 0;
}

/*********************************************************************
 * @fn      simpleBLECentral_ProcessOSALMsg
 *
 * @brief   Process an incoming task message.
 *
 * @param   pMsg - message to process
 *
 * @return  none
 */
static void simpleBLECentral_ProcessOSALMsg( osal_event_hdr_t *pMsg )
{
  switch ( pMsg->event )
  {
    case KEY_CHANGE:
      simpleBLECentral_HandleKeys( ((keyChange_t *)pMsg)->state, ((keyChange_t *)pMsg)->keys );
      break;

    case GATT_MSG_EVENT:
      simpleBLECentralProcessGATTMsg( (gattMsgEvent_t *) pMsg );
      break;
  }
}

/*********************************************************************
 * @fn      simpleBLECentral_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
uint8 gStatus;
static void simpleBLECentral_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter

  if ( keys & HAL_KEY_UP )
  {
    NPI_PrintString("    [KEY UP pressed!]\r\n");
    
    // Start or stop discovery
    //if ( !simpleBLEScanning )
    if(!GET_SIMPLEBLE_SCANNING_FLAG())
    {
      //simpleBLEScanning = TRUE;
      SET_SIMPLEBLE_SCANNING_FLAG();
      simpleBLEScanRes = 0;
        
      LCD_WRITE_STRING( "Discovering...", HAL_LCD_LINE_1 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_2 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_3 );
      LCD_WRITE_STRING( "", HAL_LCD_LINE_4 );
        
      GAPCentralRole_StartDiscovery( DEFAULT_DISCOVERY_MODE,
                                     DEFAULT_DISCOVERY_ACTIVE_SCAN,
                                     DEFAULT_DISCOVERY_WHITE_LIST );      
    }
    else
    {
      GAPCentralRole_CancelDiscovery();
    }
  }

  if ( keys & HAL_KEY_LEFT )
  {

  }

  if ( keys & HAL_KEY_RIGHT )
  {
    // Connection update
    //if ( simpleBLEState == BLE_STATE_CONNECTED )
    {
      /*GAPCentralRole_UpdateLink( simpleBLEConnHandle,
                                 DEFAULT_UPDATE_MIN_CONN_INTERVAL,
                                 DEFAULT_UPDATE_MAX_CONN_INTERVAL,
                                 DEFAULT_UPDATE_SLAVE_LATENCY,
                                 DEFAULT_UPDATE_CONN_TIMEOUT );*/
    }
  }
  
  if ( keys & HAL_KEY_CENTER )
  {
    if(simpleBLEScanRes > 0)
    {
      simpleBLEScanIdx = 0;
      performPeriodAutoConnect();
      osal_start_timerEx(simpleBLETaskId, SBP_AUTO_CONNECT_EVT, SBP_PERIODIC_EVT_AUTO_CONNECT);
    }
  }
  
  if ( keys & HAL_KEY_DOWN )
  {

  }
}
//uint8 times = 0;
/*********************************************************************
 * @fn      simpleBLECentralProcessGATTMsg
 *
 * @brief   Process GATT messages
 *
 * @return  none
 */
static void simpleBLECentralProcessGATTMsg( gattMsgEvent_t *pMsg )
{
  BLE_DEV_INFO *pDev = simpleBLECentralGetDevHandle(pMsg->connHandle);
  
  //NPI_PrintValue("-->:pMsg->connHandle", pMsg->connHandle, 16);
  
  if(pDev == NULL)
  {
    NPI_PrintValue("Error at simpleBLECentral.c line=", __LINE__, 10);
    return;
  }
  
  if ( pDev->simpleBLEState != BLE_STATE_CONNECTED )
  {
    // In case a GATT message came after a connection has dropped,
    // ignore the message
    return;
  }
  
  if ( ( pMsg->method == ATT_READ_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_READ_REQ ) ) )
  {
    if ( pMsg->method == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      LCD_WRITE_STRING_VALUE( "Read Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      // After a successful read, display the read value
      uint8 valueRead = pMsg->msg.readRsp.value[0];

      LCD_WRITE_STRING_VALUE( "Read rsp:", valueRead, 10, HAL_LCD_LINE_1 );
    }
    
    //simpleBLEProcedureInProgress = FALSE;
  }
  else if ( ( pMsg->method == ATT_WRITE_RSP ) ||
       ( ( pMsg->method == ATT_ERROR_RSP ) &&
         ( pMsg->msg.errorRsp.reqOpcode == ATT_WRITE_REQ ) ) )
  {
    
    if ( pMsg->method == ATT_ERROR_RSP == ATT_ERROR_RSP )
    {
      uint8 status = pMsg->msg.errorRsp.errCode;
      
      //pDev->simpleBLEDoWrite = FALSE;
      
      LCD_WRITE_STRING_VALUE( "Write Error", status, 10, HAL_LCD_LINE_1 );
    }
    else
    {
      //NPI_PrintString( "Write Success.\r\n");
      
      if(GET_BLE_WRITE_CYCLE_FLAG() == TRUE)
      {
        simpleBLECycleIdx++;
        
        if(simpleBLECycleIdx != (simpleBLESendDataCycleNum - 1))
        {
          simpleBLECentarlSendData2Peripheral(pDev->simpleBLEDevIdx, 
                                              BLE_CHAR6, 
                                              simpleBLEPendingDataBuffer + simpleBLECycleIdx * SIMPLEPROFILE_CHAR6_LEN, 
                                              SIMPLEPROFILE_CHAR6_LEN);
        }
        else
        {
          simpleBLECentarlSendData2Peripheral(pDev->simpleBLEDevIdx, 
                                              BLE_CHAR6, 
                                              simpleBLEPendingDataBuffer + simpleBLECycleIdx * SIMPLEPROFILE_CHAR6_LEN, 
                                              simpleBLELastSendDataLen);
          
          pDev->simpleBLEDoWrite = TRUE;
          CLR_BLE_WRITE_CYCLE_FLAG();
          simpleBLECycleIdx = 0;
          
#ifdef BYTE_RATE_DEBUG
          uint8 str[30] = {'\0'};
          
          simpleByteRateTime[1] = osal_GetSystemClock();
          
          simpleBLEByteRate = simpleByteNum / ((float)(simpleByteRateTime[1] - simpleByteRateTime[0]) / 1000);
          
          sprintf((char *)str, "byte rate: %.2f byte/s\r\n\r\n", simpleBLEByteRate);
          
          NPI_PrintValue("bytes: ", simpleByteNum, 10);
          NPI_PrintValue("time: ", (simpleByteRateTime[1] - simpleByteRateTime[0]), 10);
          NPI_PrintString(str);
#endif
        }
      }
      else
      {
        pDev->simpleBLEDoWrite = TRUE;
      }
      
      // After a succesful write, display the value that was written and increment value
      LCD_WRITE_STRING_VALUE( "Write sent:", simpleBLECharVal++, 10, HAL_LCD_LINE_1 );      
    }
    
    //simpleBLEProcedureInProgress = FALSE;    

  }
  else if ( pDev->simpleBLEDiscState != BLE_DISC_STATE_IDLE )
  {
    simpleBLEGATTDiscoveryEvent( pMsg );
  }
  else if(pMsg->method == ATT_HANDLE_VALUE_NOTI)
  {
    //if(pMsg->msg.handleValueNoti.handle == 0x0035)
    {
      NPI_WriteTransport(pMsg->msg.handleValueNoti.value, pMsg->msg.handleValueNoti.len);
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralRssiCB
 *
 * @brief   RSSI callback.
 *
 * @param   connHandle - connection handle
 * @param   rssi - RSSI
 *
 * @return  none
 */
static void simpleBLECentralRssiCB( uint16 connHandle, int8 rssi )
{
    LCD_WRITE_STRING_VALUE( "RSSI -dB:", (uint8) (-rssi), 10, HAL_LCD_LINE_1 );
}

/*********************************************************************
 * @fn      simpleBLECentralEventCB
 *
 * @brief   Central event callback function.
 *
 * @param   pEvent - pointer to event structure
 *
 * @return  none
 */
static void simpleBLECentralEventCB( gapCentralRoleEvent_t *pEvent )
{
  switch ( pEvent->gap.opcode )
  {
    case GAP_DEVICE_INIT_DONE_EVENT:  
      {
        NPI_PrintString(">>>GAP_DEVICE_INIT_DONE_EVENT\r\n");
        LCD_WRITE_STRING( "BLE Central", HAL_LCD_LINE_1 );
        LCD_WRITE_STRING( bdAddr2Str( pEvent->initDone.devAddr ),  HAL_LCD_LINE_2 );
      }
      break;

    case GAP_DEVICE_INFO_EVENT:
      {
        NPI_PrintString(">>>GAP_DEVICE_INFO_EVENT\r\n");
        // if filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == TRUE )
        {
          if ( simpleBLEFindSvcUuid( SIMPLEPROFILE_SERV_UUID,
                                     pEvent->deviceInfo.pEvtData,
                                     pEvent->deviceInfo.dataLen ) )
          {
            simpleBLEAddDeviceInfo( pEvent->deviceInfo.addr, pEvent->deviceInfo.addrType );
          }
        }
      }
      break;
      
    case GAP_DEVICE_DISCOVERY_EVENT:
      {
        NPI_PrintString(">>>GAP_DEVICE_DISCOVERY_EVENT\r\n");
        // discovery complete
        //simpleBLEScanning = FALSE;
        CLR_SIMPLEBLE_SCANNING_FLAG();

        // if not filtering device discovery results based on service UUID
        if ( DEFAULT_DEV_DISC_BY_SVC_UUID == FALSE )
        {
          // Copy results
          simpleBLEScanRes = pEvent->discCmpl.numDevs;
          osal_memcpy( simpleBLEDevList, pEvent->discCmpl.pDevList,
                       (sizeof( gapDevRec_t ) * pEvent->discCmpl.numDevs) );
        }
        
        LCD_WRITE_STRING_VALUE( "Devices Found", simpleBLEScanRes,
                                10, HAL_LCD_LINE_1 );
#if(HAL_LCD == TRUE)       
        HalLcd_HW_Clear(0,127,3,6);
#endif
        
        if ( simpleBLEScanRes > 0 )
        {
          uint8 i;
          char str[32] = {0};
          //LCD_WRITE_STRING( "<- To Select", HAL_LCD_LINE_2 );
          
          for(i = 0; i < simpleBLEScanRes; i++)
          {
            
#if(HAL_LCD == TRUE)
            LCD_WRITE_STRING(str, HAL_LCD_LINE_3 + i);
#else
            sprintf(str, "[%d]=%s\r\n", i, ((uint8*) bdAddr2Str( simpleBLEDevList[i].addr )));
            NPI_PrintString((char *)str);  
#endif
            
          }
          
          LCD_WRITE_STRING("Key CENTER=Connect", HAL_LCD_LINE_7);
        }

        // initialize scan index to last device
        
        //simpleBLEScanIdx = simpleBLEScanRes;
      }
      break;

    case GAP_LINK_ESTABLISHED_EVENT:
      {
        BLE_DEV_INFO *pDev = &(stDevInfo[simpleBLEScanIdx]);
        
        pDev->simpleBLEDevIdx = simpleBLEScanIdx;
        
        NPI_PrintString(">>>GAP_LINK_ESTABLISHED_EVENT\r\n");
               
        if ( pEvent->gap.hdr.status == SUCCESS )
        { 
          pDev->simpleBLEState = BLE_STATE_CONNECTED;
          pDev->simpleBLEConnHandle = pEvent->linkCmpl.connectionHandle;

          if(pDev->simpleBLECharHdl[BLE_CHAR1] == 0)
          {
            osal_start_timerEx(simpleBLETaskId, START_DISCOVERY_EVT, DEFAULT_SVC_DISCOVERY_DELAY);
          }
          else
          {
            NPI_PrintString("simpleBLECharHdl[0] = 0\r\n");
          }
            
          NPI_PrintString("Connected\r\n");
        }
        else
        {
          //simpleBLEState = BLE_STATE_IDLE;
          //simpleBLEConnHandle = GAP_CONNHANDLE_INIT;
          //simpleBLERssi = FALSE;
          //simpleBLEDiscState = BLE_DISC_STATE_IDLE;
          
          osal_memcpy(pDev, &gDevInfoInitVal, sizeof(BLE_DEV_INFO));
          
          NPI_PrintString( "Connect Failed\r\n");
          NPI_PrintValue( "Reason:", pEvent->gap.hdr.status, 10);
        }
      }
      break;

    case GAP_LINK_TERMINATED_EVENT:
      { 
        NPI_PrintString(">>>GAP_LINK_TERMINATED_EVENT\r\n");
          
        //LCD_WRITE_STRING("Disconnected", HAL_LCD_LINE_1 );
        //LCD_WRITE_STRING_VALUE( "Reason:", pEvent->linkTerminate.reason,
          //                      10, HAL_LCD_LINE_2 );
        NPI_PrintValue("Reason: ", pEvent->linkTerminate.reason, 10);
      }
      break;

    case GAP_LINK_PARAM_UPDATE_EVENT:
      {
         NPI_PrintString(">>>GAP_LINK_PARAM_UPDATE_EVENT\r\n");
        LCD_WRITE_STRING( "Param Update", HAL_LCD_LINE_1 );
      }
      
      // enable notify
      {
        attWriteReq_t AttReq; 
        uint8 ValueBuf[2];
        
        AttReq.handle = (0x0035 + 1);
        AttReq.len = 2;
        AttReq.sig = 0;
        AttReq.cmd = 0;
        ValueBuf[0] = 0x01;
        ValueBuf[1] = 0x00;
        osal_memcpy(AttReq.value,ValueBuf,2);       
        GATT_WriteCharValue( 0, &AttReq, simpleBLETaskId );
      }
      break;
      
    default:
      break;
  }
}

/*********************************************************************
 * @fn      pairStateCB
 *
 * @brief   Pairing state callback.
 *
 * @return  none
 */
static void simpleBLECentralPairStateCB( uint16 connHandle, uint8 state, uint8 status )
{
  if ( state == GAPBOND_PAIRING_STATE_STARTED )
  {
    LCD_WRITE_STRING( "Pairing started", HAL_LCD_LINE_1 );
  }
  else if ( state == GAPBOND_PAIRING_STATE_COMPLETE )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Pairing success", HAL_LCD_LINE_1 );
    }
    else
    {
      LCD_WRITE_STRING_VALUE( "Pairing fail", status, 10, HAL_LCD_LINE_1 );
    }
  }
  else if ( state == GAPBOND_PAIRING_STATE_BONDED )
  {
    if ( status == SUCCESS )
    {
      LCD_WRITE_STRING( "Bonding success", HAL_LCD_LINE_1 );
    }
  }
}

/*********************************************************************
 * @fn      simpleBLECentralPasscodeCB
 *
 * @brief   Passcode callback.
 *
 * @return  none
 */
static void simpleBLECentralPasscodeCB( uint8 *deviceAddr, uint16 connectionHandle,
                                        uint8 uiInputs, uint8 uiOutputs )
{
#if (HAL_LCD == TRUE)

  uint32  passcode;
  uint8   str[7];

  // Create random passcode
  LL_Rand( ((uint8 *) &passcode), sizeof( uint32 ));
  passcode %= 1000000;
  
  // Display passcode to user
  if ( uiOutputs != 0 )
  {
    LCD_WRITE_STRING( "Passcode:",  HAL_LCD_LINE_1 );
    LCD_WRITE_STRING( (char *) _ltoa(passcode, str, 10),  HAL_LCD_LINE_2 );
  }
  
  // Send passcode response
  GAPBondMgr_PasscodeRsp( connectionHandle, SUCCESS, passcode );
#endif
}

/*********************************************************************
 * @fn      simpleBLECentralStartDiscovery
 *
 * @brief   Start service discovery.
 *
 * @return  none
 */
static void simpleBLECentralStartDiscovery( void )
{
  uint8 uuid[ATT_BT_UUID_SIZE] = { LO_UINT16(SIMPLEPROFILE_SERV_UUID),
                                   HI_UINT16(SIMPLEPROFILE_SERV_UUID) };
  
  BLE_DEV_INFO *pDev = &stDevInfo[simpleBLEScanIdx];
  
  // Initialize cached handles
  //simpleBLESvcStartHdl = simpleBLESvcEndHdl = simpleBLECharHdl = 0;
  //simpleBLEDiscState = BLE_DISC_STATE_SVC;

  pDev->simpleBLESvcStartHdl = pDev->simpleBLESvcEndHdl = 0;
  pDev->simpleBLEDiscState = BLE_DISC_STATE_SVC;
  
  // Discovery simple BLE service
  GATT_DiscPrimaryServiceByUUID( pDev->simpleBLEConnHandle,
                                 uuid,
                                 ATT_BT_UUID_SIZE,
                                 simpleBLETaskId );
}

/*********************************************************************
 * @fn      simpleBLEGATTDiscoveryEvent
 *
 * @brief   Process GATT discovery event
 *
 * @return  none
 */
static void simpleBLEGATTDiscoveryEvent( gattMsgEvent_t *pMsg )
{
  attReadByTypeReq_t req;
  
  BLE_DEV_INFO *pDev = simpleBLECentralGetDevHandle(pMsg->connHandle);
  
  if ( pDev->simpleBLEDiscState == BLE_DISC_STATE_SVC )
  {
    // Service found, store handles
    if ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP &&
         pMsg->msg.findByTypeValueRsp.numInfo > 0 )
    {
      pDev->simpleBLESvcStartHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].handle;
      pDev->simpleBLESvcEndHdl = pMsg->msg.findByTypeValueRsp.handlesInfo[0].grpEndHandle;
    }
    
    // If procedure complete
    if ( ( pMsg->method == ATT_FIND_BY_TYPE_VALUE_RSP  && 
           pMsg->hdr.status == bleProcedureComplete ) ||
         ( pMsg->method == ATT_ERROR_RSP ) )
    {
      if ( pDev->simpleBLESvcStartHdl != 0 )
      {
        // Discover characteristic
        pDev->simpleBLEDiscState = BLE_DISC_STATE_CHAR1;
        
        req.startHandle = pDev->simpleBLESvcStartHdl;
        req.endHandle = pDev->simpleBLESvcEndHdl;
        req.type.len = ATT_BT_UUID_SIZE;
        req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR1_UUID);
        req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR1_UUID);

        GATT_ReadUsingCharUUID( pDev->simpleBLEConnHandle, &req, simpleBLETaskId );
      }
    }
  }
  else if ( pDev->simpleBLEDiscState == BLE_DISC_STATE_CHAR1 )
  {
    //NPI_PrintValue("pMsg->method", pMsg->method, 10);
    //NPI_PrintValue("numPairs", pMsg->msg.readByTypeRsp.numPairs, 10);
    
    // Characteristic found, store handle
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP && 
         pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      pDev->simpleBLECharHdl[BLE_CHAR1] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],
                                       pMsg->msg.readByTypeRsp.dataList[1] );
      
      NPI_PrintString( "CHAR 1 Found\r\n");
      //simpleBLEProcedureInProgress = FALSE;
    }
    else
    {
      pDev->simpleBLEDiscState = BLE_DISC_STATE_CHAR6;
      req.startHandle = pDev->simpleBLESvcStartHdl;
      req.endHandle = pDev->simpleBLESvcEndHdl;   
      req.type.len = ATT_BT_UUID_SIZE;    
      req.type.uuid[0] = LO_UINT16(SIMPLEPROFILE_CHAR6_UUID);   
      req.type.uuid[1] = HI_UINT16(SIMPLEPROFILE_CHAR6_UUID);    
      GATT_ReadUsingCharUUID( pDev->simpleBLEConnHandle, &req, simpleBLETaskId );
    }
  }
  else if (pDev->simpleBLEDiscState == BLE_DISC_STATE_CHAR6)
  {
    if ( pMsg->method == ATT_READ_BY_TYPE_RSP &&     
      pMsg->msg.readByTypeRsp.numPairs > 0 )
    {
      pDev->simpleBLECharHdl[BLE_CHAR6] = BUILD_UINT16( pMsg->msg.readByTypeRsp.dataList[0],                                      
                                     pMsg->msg.readByTypeRsp.dataList[1] );
      
      NPI_PrintString( "CHAR 6 Found\r\n\r\n");
    }
    
    pDev->simpleBLEDiscState = BLE_DISC_STATE_IDLE; 
  }
}


/*********************************************************************
 * @fn      simpleBLEFindSvcUuid
 *
 * @brief   Find a given UUID in an advertiser's service UUID list.
 *
 * @return  TRUE if service UUID found
 */
static bool simpleBLEFindSvcUuid( uint16 uuid, uint8 *pData, uint8 dataLen )
{
  uint8 adLen;
  uint8 adType;
  uint8 *pEnd;
  
  pEnd = pData + dataLen - 1;
  
  // While end of data not reached
  while ( pData < pEnd )
  {
    // Get length of next AD item
    adLen = *pData++;
    if ( adLen > 0 )
    {
      adType = *pData;
      
      // If AD type is for 16-bit service UUID
      if ( adType == GAP_ADTYPE_16BIT_MORE || adType == GAP_ADTYPE_16BIT_COMPLETE )
      {
        pData++;
        adLen--;
        
        // For each UUID in list
        while ( adLen >= 2 && pData < pEnd )
        {
          // Check for match
          if ( pData[0] == LO_UINT16(uuid) && pData[1] == HI_UINT16(uuid) )
          {
            // Match found
            return TRUE;
          }
          
          // Go to next
          pData += 2;
          adLen -= 2;
        }
        
        // Handle possible erroneous extra byte in UUID list
        if ( adLen == 1 )
        {
          pData++;
        }
      }
      else
      {
        // Go to next item
        pData += adLen;
      }
    }
  }
  
  // Match not found
  return FALSE;
}

/*********************************************************************
 * @fn      simpleBLEAddDeviceInfo
 *
 * @brief   Add a device to the device discovery result list
 *
 * @return  none
 */
static void simpleBLEAddDeviceInfo( uint8 *pAddr, uint8 addrType )
{
  uint8 i;
  
  // If result count not at max
  if ( simpleBLEScanRes < DEFAULT_MAX_SCAN_RES )
  {
    // Check if device is already in scan results
    for ( i = 0; i < simpleBLEScanRes; i++ )
    {
      if ( osal_memcmp( pAddr, simpleBLEDevList[i].addr , B_ADDR_LEN ) )
      {
        return;
      }
    }
    
    // Add addr to scan result list
    osal_memcpy( simpleBLEDevList[simpleBLEScanRes].addr, pAddr, B_ADDR_LEN );
    simpleBLEDevList[simpleBLEScanRes].addrType = addrType;
    
    // Increment scan result count
    simpleBLEScanRes++;
  }
}

/*********************************************************************
 * @fn      bdAddr2Str
 *
 * @brief   Convert Bluetooth address to string
 *
 * @return  none
 */
char *bdAddr2Str( uint8 *pAddr )
{
  uint8       i;
  char        hex[] = "0123456789ABCDEF";
  static char str[B_ADDR_STR_LEN];
  char        *pStr = str;
  
  *pStr++ = '0';
  *pStr++ = 'x';
  
  // Start from end of addr
  pAddr += B_ADDR_LEN;
  
  for ( i = B_ADDR_LEN; i > 0; i-- )
  {
    *pStr++ = hex[*--pAddr >> 4];
    *pStr++ = hex[*pAddr & 0x0F];
  }
  
  *pStr = 0;
  
  return str;
}

static void performPeriodAutoConnect(void)
{
  uint8 addrType;
  uint8 *peerAddr;
  
  if(stDevInfo[simpleBLEScanIdx].simpleBLEState == BLE_STATE_IDLE)
  {
    if((simpleBLEScanRes > 0) && (simpleBLEScanIdx < simpleBLEScanRes))
    {
      addrType = simpleBLEDevList[simpleBLEScanIdx].addrType;
      peerAddr = simpleBLEDevList[simpleBLEScanIdx].addr;
      
      stDevInfo[simpleBLEScanIdx].simpleBLEState = BLE_STATE_CONNECTING;
      
      GAPCentralRole_EstablishLink( DEFAULT_LINK_HIGH_DUTY_CYCLE,
                                    DEFAULT_LINK_WHITE_LIST,
                                    addrType, peerAddr );
      
      NPI_PrintString(">>>Connecting ");
      NPI_PrintString((uint8 *)bdAddr2Str(peerAddr));
      NPI_PrintString("\r\n");
    }
  }
  else if((stDevInfo[simpleBLEScanIdx].simpleBLEState == BLE_STATE_CONNECTING) ||
          (stDevInfo[simpleBLEScanIdx].simpleBLEState == BLE_STATE_CONNECTED) ||
          (stDevInfo[simpleBLEScanIdx].simpleBLEState == BLE_STATE_DISCONNECTING))
  {
    stDevInfo[simpleBLEScanIdx].simpleBLEState = BLE_STATE_DISCONNECTING;
    
    GAPCentralRole_TerminateLink(stDevInfo[simpleBLEScanIdx].simpleBLEConnHandle);
  }
}

static BLE_DEV_INFO *simpleBLECentralGetDevHandle(uint16 connHandle)
{
  uint8 i;
  
  for(i = 0; i < DEFAULT_MAX_SCAN_RES; i++)
  {
    if(connHandle == stDevInfo[i].simpleBLEConnHandle)
    {
      return &stDevInfo[i];
    }
  }
  
  return NULL;
}

void simpleBLE_NpiSerialCallback(uint8 port, uint8 events)
{
  (void)port;
  
  static uint32 oldTime;
  static uint32 oldTimeDataLen = 0;
  uint32 newTime;
  uint8 memorySize = BLE_PENDING_DATA_MAX;
  
  if(events & (HAL_UART_RX_TIMEOUT | HAL_UART_RX_FULL))
  {
    uint8 numBytes = 0;
    
    numBytes = NPI_RxBufLen();
    
    if(numBytes == 0)
    {
      oldTimeDataLen = 0;
      return;
    }
    
    if(oldTimeDataLen == 0)
    {
      oldTime = osal_GetSystemClock();
      oldTimeDataLen = numBytes;
    }
    else
    {
      newTime = osal_GetSystemClock();
      
      if((newTime - oldTime) > 20)
      {
        uint8 *buffer = osal_mem_alloc(memorySize);
         
        if(buffer == NULL)
        {
          NPI_PrintString("FAIL!\r\n");
          return;
        }
        
        NPI_ReadTransport(buffer, numBytes);
        
        // AT Command
        if((buffer[numBytes - 2] == '\r') && (buffer[numBytes - 1] == '\n'))
        {
          if(simpleStrCmp(buffer, "AT+WRMAC", 8))
          {           
            uint8 idx = buffer[8] - '0';
            
            if(idx >= simpleBLEScanRes)
            {
              NPI_PrintString("device idx error.\r\n");
              return;
            }
            
            BLE_DEV_INFO *pDev = &stDevInfo[idx];
            
            if(pDev == NULL)
            {
              NPI_PrintString("No device found.\r\n");
              return;
            }

            if(pDev->simpleBLEDoWrite)
            {
              uint8 dataLen = numBytes - 9 - 1 - 2;
              
              if(dataLen > BLE_PENDING_DATA_MAX)
              {
                NPI_PrintValue("data len > ", BLE_PENDING_DATA_MAX, 10);
                return;
              }
              
              osal_memset(simpleBLEPendingDataBuffer, '\0', BLE_PENDING_DATA_MAX);
              osal_memcpy(simpleBLEPendingDataBuffer, buffer + 10, dataLen);
              
              simpleBLESendDataCycleNum = (dataLen / SIMPLEPROFILE_CHAR6_LEN) + 1;
              simpleBLELastSendDataLen = dataLen % SIMPLEPROFILE_CHAR6_LEN;
              
              // length of data <= 19 bytes
              if(simpleBLESendDataCycleNum == 1)
              {
                simpleBLECentarlSendData2Peripheral(pDev->simpleBLEDevIdx, BLE_CHAR6, simpleBLEPendingDataBuffer, simpleBLELastSendDataLen);
              }
              else // length of data > 19 bytes
              {
#ifdef BYTE_RATE_DEBUG
                simpleByteNum = dataLen;
                simpleByteRateTime[0] = osal_GetSystemClock();
#endif
                
                simpleBLECentarlSendData2Peripheral(pDev->simpleBLEDevIdx, BLE_CHAR6, simpleBLEPendingDataBuffer, SIMPLEPROFILE_CHAR6_LEN);
                
                SET_BLE_WRITE_CYCLE_FLAG();
              }
              
              pDev->simpleBLEDoWrite = FALSE;
            }
          }
        }
        
        oldTimeDataLen = 0;
        //oldTime = newTime;
        osal_mem_free(buffer);
      }
    }
  }
}

void simpleBLECentarlSendData2Peripheral(uint8 devIdx, uint8 charIdx, uint8 *pData, uint8 len)
{ 
  BLE_DEV_INFO *pDev = &(stDevInfo[devIdx]);
  
  if((pDev->simpleBLEState == BLE_STATE_CONNECTED) && 
     (pDev->simpleBLECharHdl[BLE_CHAR6] != 0) &&
     (pDev->simpleBLEConnHandle != GAP_CONNHANDLE_INIT))
  {
    uint8 status;
    
    pDev->simpleBLEDoWrite = FALSE;
    
    // do write
    attWriteReq_t req;
    
    req.handle = pDev->simpleBLECharHdl[BLE_CHAR6];
    req.len = len;
    osal_memcpy(req.value, pData, len);
    req.sig = 0;
    req.cmd = 0;
    
    status = GATT_WriteCharValue(pDev->simpleBLEConnHandle, &req, simpleBLETaskId);

    if(status != SUCCESS)
    {
      NPI_PrintValue("Write Error :", status, 10);
    }
  }
}

bool simpleGetDeviceWriteFlag(void)
{
  uint8 i;
  
  for(i = 0; i < simpleBLEScanRes; i++)
  {
    if(stDevInfo[i].simpleBLEDoWrite == FALSE)
    {
      return FALSE;
    }
  }
  
  return TRUE;
}

static bool simpleStrCmp(uint8 *pStr1, uint8 *pStr2, uint8 len)
{
  uint8 i = 0;
  
  while(i < len)
  {
    if(pStr1[i] != pStr2[i])
    {
      return FALSE;
    }
    i++;
  }
  
  return TRUE;
}

/*********************************************************************
*********************************************************************/
