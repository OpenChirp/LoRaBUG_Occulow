
/* XDCtools Header files */

#include <App/Commissioning.h>
#include <App/pb_decode.h>
#include <App/pb_encode.h>
#include <App/occulow.pb.h>
#include <xdc/std.h>
#include <xdc/runtime/System.h>

/* BIOS Header files */
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>
#include <ti/sysbios/knl/Semaphore.h>

/* TI-RTOS Header files */
// #include <ti/drivers/I2C.h>
#include <ti/drivers/PIN.h>
#include <ti/drivers/SPI.h>
#include <ti/drivers/UART.h>
#include <ti/drivers/ADC.h>
// #include <ti/drivers/Watchdog.h>

/* Board Header files */
#include "Board_LoRaBUG.h"

#include <string.h> // strlen in uartputs and LoRaWan code

#include "board.h"
#include "io.h"
#include "Services/ble_rf.h"

#include "LoRaMac.h"

#include "Services/grideyeService.h"
#include "Services/pcService.h"

#include <ti/drivers/I2C.h>
#include <ti/drivers/i2c/I2CCC26XX.h>
#include <driverlib/flash.h>
#include <driverlib/vims.h>

#define TASKSTACKSIZE   1500

Task_Struct task0Struct;
Char task0Stack[TASKSTACKSIZE];

/* Runtime Events */
#define EVENT_STATECHANGE Event_Id_00
static Event_Struct runtimeEventsStruct;
static Event_Handle runtimeEvents;

#define BLE_ADV_DUTY_CYCLE_MS 100
#define BLE_PAYLOAD_MAX_SIZE 30

static uint8_t blePayload[BLE_PAYLOAD_MAX_SIZE];

static TimerEvent_t buttonTimer;
static TimerEvent_t broadcastTimer;

#define JOINED_STATUS_FLASH_ADDR 0x1FFFF
#define JOINED_STATUS_VAL 0x42

/*------------------------------------------------------------------------*/
/*                      Start of LoRaWan Demo Code                        */
/*------------------------------------------------------------------------*/

/*!
 * Defines the application data transmission duty cycle. 15s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            30000

/*!
 * Defines a random delay for application data transmission duty cycle. 2s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
//#define LORAWAN_DEFAULT_DATARATE                    DR_0
#define LORAWAN_DEFAULT_DATARATE                    DR_4

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    false

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              0

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size BAND_915
 */
#define LORAWAN_APP_DATA_SIZE                       11

#define LORAWAN_DEV_EUI_SIZE 8
#define LORAWAN_APP_KEY_SIZE 16

#if(USE_BOARD_UNIQUE_ID_DEV_EUI == 1)
static uint8_t DevEui[LORAWAN_DEV_EUI_SIZE];
#else
static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
#endif

static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_BROADCAST,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

void user_delay_ms(uint32_t period)
{
    DELAY_MS(period);
}

/*********************************************************************
 * @fn      enableCache
 *
 * @brief   enable cache.
 *
 * @param   state - the VIMS state returned from disableCache.
 *
 * @return  none.
 */
static void enableCache ( uint8_t state )
{
  if ( state != VIMS_MODE_DISABLED )
  {
    // Enable the Cache.
    VIMSModeSet( VIMS_BASE, VIMS_MODE_ENABLED );
  }
}

/*********************************************************************
 * @fn      disableCache
 *
 * @brief   invalidate and disable cache.
 *
 * @param   none
 *
 * @return  VIMS state
 */
static uint8_t disableCache ( void )
{
  uint8_t state = VIMSModeGet( VIMS_BASE );

  // Check VIMS state
  if ( state != VIMS_MODE_DISABLED )
  {
    // Invalidate cache
    VIMSModeSet( VIMS_BASE, VIMS_MODE_DISABLED );

    // Wait for disabling to be complete
    while ( VIMSModeGet( VIMS_BASE ) != VIMS_MODE_DISABLED );

  }

  return state;
}

static bool hasJoined() {
    uint8_t val;
    memcpy(&val, (void *) JOINED_STATUS_FLASH_ADDR, 1);

    return (val == JOINED_STATUS_VAL);
}

static void setJoined(bool joined) {
    uint8_t val;
    uint8_t state = disableCache();
    if (joined) {
        val = JOINED_STATUS_VAL;
    } else {
        val = 0x00;
    }

    FlashProgram(&val, JOINED_STATUS_FLASH_ADDR, 1);
    enableCache(state);
}

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    size_t message_length;
    static CountMessage message = CountMessage_init_zero;
    pb_ostream_t stream;
    bool status;
    static pc_counter_t counter;

    //uartprintf ("# PrepareTxFrame\r\n");

    switch( port )
    {
    case LORAWAN_APP_PORT:
        //Prepare sensor readings to send over LoRa
        stream = pb_ostream_from_buffer(AppData, sizeof(AppData));

        pc_get_counts(&counter, true);
        message.count_in = counter.in_count;
        message.count_out = counter.out_count;
        message.batteryVoltage = BoardGetBatteryVoltage();
        message.batteryLevel = BoardGetBatteryLevel();
        //uartprintf ("Sending %d/%d\r\nVoltage: %d\r\nLevel: %d\r\n", message.count_in, message.count_out, message.batteryVoltage, message.batteryLevel);

        status = pb_encode(&stream, CountMessage_fields, &message);
        message_length = stream.bytes_written;

        AppDataSize = message_length;

        if(!status) {
            //uartprintf ("Encoding failed: %s\r\n", PB_GET_ERROR(&stream));
        }

        break;

    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    //uartprintf ("# SendFrame\r\n");

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    //uartprintf ("# OnTxNextPacketTimerEvent\r\n");
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );
    //uartprintf ("Status: %d\r\n", (int)status);

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }

    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    //uartprintf ("# McpsConfirm\r\n");
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                //uartprintf("# Got McpsConfirm: MCPS_UNCONFIRMED\r\n");
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                // Check TxPower
                // Check AckReceived
                // Check NbTrials
                //uartprintf("# Got McpsConfirm: MCPS_CONFIRMED\r\n");
                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }
    }
    NextTx = true;

    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    //uartprintf ("# McpsIndication\r\n");
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            //uartprintf ("# Got McpsIndication: MCPS_UNCONFIRMED\n");
            break;
        }
        case MCPS_CONFIRMED:
        {
            //uartprintf ("# Got McpsIndication: MCPS_CONFIRMED\n");
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );

                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            //uartprintf ("# MlmeConfirm: Join\r\n");
            //uartprintf ("# Mlme status: %d\r\n", (int)mlmeConfirm->Status);
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                //uartprintf ("# Got OK status\r\n");
                setJoined(true);
                DeviceState = DEVICE_STATE_SEND;
            }
            else
            {
                // Join was not successful. Try to join again
                //uartprintf ("# Not successful\r\n");
                DeviceState = DEVICE_STATE_JOIN;
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            //uartprintf ("# MlmeConfirm: Link Check\n");
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;

    Event_post(runtimeEvents, EVENT_STATECHANGE);
}

static void OnBroadcastTimerEvent(void) {
    TimerStop(&broadcastTimer);
    DeviceState = DEVICE_STATE_SEND;
}

static void btnIntCallback(PIN_Handle handle, PIN_Id pinId)
{
    TimerReset(&buttonTimer);
}

static void OnButtonTimerEvent( void )
{
    TimerStop( &buttonTimer );
    if (DeviceState == DEVICE_STATE_BROADCAST) {
        DeviceState = DEVICE_STATE_JOIN;
    } else {
        DeviceState = DEVICE_STATE_BROADCAST;
        if (hasJoined()) {
            TimerStart(&broadcastTimer);
        }
        //uartprintf("Dev EUI:\r\n");
        for (int i = 0; i < 8; i++) {
            //uartprintf("%02x", DevEui[i]);
        }
        //uartprintf("\r\n");
        Event_post(runtimeEvents, EVENT_STATECHANGE);
    }
}

static void loadDeviceInfo() {
#if(USE_BOARD_UNIQUE_ID_DEV_EUI == 1)
    // Dev EUI is device BLE address
    BoardGetUniqueId(DevEui);
#endif
}

void maintask(UArg arg0, UArg arg1)
{
    LoRaMacPrimitives_t LoRaMacPrimitives;
    LoRaMacCallback_t LoRaMacCallbacks;
    MibRequestConfirm_t mibReq;

    Event_construct(&runtimeEventsStruct, NULL);
    runtimeEvents = Event_handle(&runtimeEventsStruct);

    BoardInitMcu( );
    BoardInitPeriph( );
    DELAY_MS(5000);
    loadDeviceInfo();
    //uartprintf ("# Board initialized\r\n");

    DeviceState = DEVICE_STATE_INIT;

    while( 1 )
    {
        switch( DeviceState )
        {
            case DEVICE_STATE_INIT:
            {
                //uartprintf ("# DeviceState: DEVICE_STATE_INIT\r\n");
                LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
                LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
                LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
                LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
                LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

                TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );

                TimerInit(&buttonTimer, OnButtonTimerEvent);
                TimerSetValue(&buttonTimer, 1000);
                setBtnIntCallback(btnIntCallback);

                TimerInit(&broadcastTimer, OnBroadcastTimerEvent);
                TimerSetValue(&broadcastTimer, 10000);

                mibReq.Type = MIB_ADR;
                mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_PUBLIC_NETWORK;
                mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
                LoRaMacMibSetRequestConfirm( &mibReq );

                if (hasJoined()) {
                    DeviceState = DEVICE_STATE_JOIN;
                } else {
                    DeviceState = DEVICE_STATE_SLEEP;
                }
                break;
            }
            case DEVICE_STATE_BROADCAST:
            {
                //uartprintf ("# DeviceState: DEVICE_STATE_BROADCAST\r\n");

                // While broadcasting, send advertisements out every 50 ms
                // Button interrupt will change the state to join
                blePayload[0] = 0x09; // 9 bytes after this one
                blePayload[1] = 0x16; // 0x16 is service data
                for (int i = 0; i < 8; i++) {
                    blePayload[i+2] = DevEui[i];
                }
                send_advertisement(blePayload, 10);
                user_delay_ms(BLE_ADV_DUTY_CYCLE_MS);
                break;
            }
            case DEVICE_STATE_JOIN:
            {
                //uartprintf ("# DeviceState: DEVICE_STATE_JOIN\r\n");
#if( OVER_THE_AIR_ACTIVATION != 0 )
                MlmeReq_t mlmeReq;

                // Initialize LoRaMac device unique ID
//                BoardGetUniqueId( DevEui );

                mlmeReq.Type = MLME_JOIN;

                mlmeReq.Req.Join.DevEui = DevEui;
                mlmeReq.Req.Join.AppEui = AppEui;
                mlmeReq.Req.Join.AppKey = AppKey;
                mlmeReq.Req.Join.NbTrials = 3;

                if( NextTx == true )
                {
                    LoRaMacStatus_t status = LoRaMacMlmeRequest( &mlmeReq );
                    //uartprintf ("Result: %d\r\n", (int)status);
                }
                DeviceState = DEVICE_STATE_SLEEP;
#else
                // Choose a random device address if not already defined in Commissioning.h
                if( DevAddr == 0 )
                {
                    // Random seed initialization
                    srand1( BoardGetRandomSeed( ) );

                    // Choose a random device address
                    DevAddr = randr( 0, 0x01FFFFFF );
                }

                mibReq.Type = MIB_NET_ID;
                mibReq.Param.NetID = LORAWAN_NETWORK_ID;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_DEV_ADDR;
                mibReq.Param.DevAddr = DevAddr;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NWK_SKEY;
                mibReq.Param.NwkSKey = NwkSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_APP_SKEY;
                mibReq.Param.AppSKey = AppSKey;
                LoRaMacMibSetRequestConfirm( &mibReq );

                mibReq.Type = MIB_NETWORK_JOINED;
                mibReq.Param.IsNetworkJoined = true;
                LoRaMacMibSetRequestConfirm( &mibReq );

                DeviceState = DEVICE_STATE_SEND;
#endif
                break;
            }
            case DEVICE_STATE_SEND:
            {
                //uartprintf ("# DeviceState: DEVICE_STATE_SEND\r\n");
                if( NextTx == true )
                {
                    PrepareTxFrame( AppPort );

                    NextTx = SendFrame( );
                }
                if( ComplianceTest.Running == true )
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = 5000; // 5000 ms
                }
                else
                {
                    // Schedule next packet transmission
                    TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
                }
                DeviceState = DEVICE_STATE_CYCLE;
                break;
            }
            case DEVICE_STATE_CYCLE:
            {
                //uartprintf ("# DeviceState: DEVICE_STATE_CYCLE\r\n");
                DeviceState = DEVICE_STATE_SLEEP;

                // Schedule next packet transmission
                TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
                TimerStart( &TxNextPacketTimer );
                break;
            }
            case DEVICE_STATE_SLEEP:
            {
                //uartprintf ("# DeviceState: DEVICE_STATE_SLEEP\r\n");
                // Wake up through events
                Event_pend(runtimeEvents, Event_Id_NONE, EVENT_STATECHANGE, BIOS_WAIT_FOREVER);
                break;
            }
            default:
            {
                DeviceState = DEVICE_STATE_INIT;
                break;
            }
        }
    }

}

/*
 *  ======== main ========
 */
int main(void)
{
    Task_Params taskParams;

    /* Call board init functions */
    Board_initGeneral();
    Board_initI2C();
    Board_initSPI();
    Board_initUART();
    //Board_initWatchdog();
    ADC_init();

    /* Construct heartBeat Task  thread */
    Task_Params_init(&taskParams);
    taskParams.arg0 = 1000000 / Clock_tickPeriod;
    taskParams.stackSize = TASKSTACKSIZE;
    taskParams.stack = &task0Stack;
    Task_construct(&task0Struct, (Task_FuncPtr) maintask, &taskParams, NULL);

    pcService_createTask();

    /* Open and setup pins */
    setuppins();

    /* Open UART */
    //setupuart();

    /* Start BIOS */
    BIOS_start();

    return (0);
}

