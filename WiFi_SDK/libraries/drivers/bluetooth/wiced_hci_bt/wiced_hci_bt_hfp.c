/*
 * $ Copyright Broadcom Corporation $
 */

/*
 * $ Copyright Broadcom Corporation $
 */

/** @bluetooth_hfp.c
 *
 * This handles the HFP api functions when USE_WICED_HCI is defined
 *
 */

#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wwd_debug.h"
#include "wiced_bt_hfp_hf.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_sco.h"
#include "wiced_hci_bt_internal_common.h"

/******************************************************
  *                    Constants
  ******************************************************/
#define HFP_CONN_MAX_LIMIT     2

/******************************************************
  *                   Structures
  ******************************************************/
 typedef struct wiced_hci_bt_hfp_context {
    wiced_bt_hfp_hf_event_cb_t     p_event_cback;
    wiced_bt_hfp_hf_config_data_t  *config_data;
    wiced_bool_t                   active_call;
}wiced_hci_bt_hfp_context_t;

typedef struct
{
    wiced_bt_device_address_t        peer_addr;
    uint16_t                         handle;
    uint16_t                         sco_index;
    wiced_bool_t                     in_use;
} wiced_bt_hci_hfp_remote_dev_t;

wiced_bt_hci_hfp_remote_dev_t      wiced_bt_hci_hfp_remote_dev[HFP_CONN_MAX_LIMIT] =
{
    {
        .peer_addr     = {0,},
        .handle        = 0,
        .sco_index     = 0,
        .in_use        = WICED_FALSE
    },
    {
        .peer_addr     = {0,},
        .handle        = 0,
        .sco_index     = 0,
        .in_use        = WICED_FALSE
    }
};

/******************************************************
 *                   Enumerations
 ******************************************************/

/* Hands Free Profile Indicators  */
 enum wiced_hci_bt_ciev_indicators_t
{
    WICED_BT_HFP_HF_SERVICE_IND_NAME = 1,
    WICED_BT_HFP_HF_CALL_IND_NAME,
    WICED_BT_HFP_HF_CALLSETUP_IND_NAME,
    WICED_BT_HFP_HF_CALL_HELD_IND_NAME,
    WICED_BT_HFP_HF_SIGNAL_IND_NAME,
    WICED_BT_HFP_HF_ROAM_IND_NAME,
    WICED_BT_HFP_HF_BATTERY_IND_NAME
};

/* Service Availablity Indication */

enum wiced_hci_bt_indicator_service_t
{
    WICED_BT_HFP_NO_SERVICE,
    WICED_BT_HFP_SERVICE_PRESENCE
};

/* Standard Call Status Indicator*/
enum wiced_hci_bt_indicators_call_t
{
    WICED_BT_HFP_NO_CALL_IN_PROGRESS,
    WICED_BT_HFP_CALL_IN_PROGRESS
};

/* Call Setup status Indicator */
enum wiced_hci_bt_indicators_callsetup_t
{
    WICED_BT_HFP_NOT_IN_CALL_SETUP,
    WICED_BT_HFP_INCOMING_CALL_PROCESS_ONGOING,
    WICED_BT_HFP_OUTGOING_CALL_SETUP_ONGOING,
    WICED_BT_HFP_REMOTE_PARTY_ALERTED_IN_OUTGOING_CALL
};

/* Call Hold Status Indicator */
enum wiced_hci_bt_indicators_callheld_t
{
    WICED_BT_HFP_NO_CALLS_HELD,
    WICED_BT_HFP_HELD_CALLED_SWAPPED,
    WICED_BT_HFP_CALL_ON_HOLD
};

/* Signal Strength Indicator */
enum wiced_hci_bt_indicator_sig_strength_t
{
    WICED_BT_HFP_MAX_SIGNAL_STRENGTH = 5
};

/* Roaming status Indicator */
enum wiced_hci_bt_indicator_roaming_status_t
{
    WICED_BT_HFP_ROAMING_NOT_ACTIVE,
    WICED_BT_HFP_ROAMING_ACTIVE
};

/* Battery Charge Indicator */
enum wiced_hci_bt_battery_charge_ind_t
{
     WICED_BT_HFP_MAX_BATTERY_CHARGE_VALUE = 5
};

/******************************************************
  *               Static Function Declarations
  ******************************************************/
static void wiced_hci_hfp_cb( uint16_t command, uint8_t* payload, uint32_t len );
void hci_control_hf_at_command (uint16_t handle, uint8_t command, int num_value, uint8_t *p_data);
static int get_hfp_context_index_by_addr(wiced_bt_device_address_t remote_addr);
static int get_hfp_context_index_by_handle(uint16_t    handle);
static int set_hfp_context_index_peer(wiced_bt_device_address_t remote_addr);

/******************************************************
  *               External Function Declarations
  ******************************************************/
extern wiced_hci_bt_dm_context_t wh_bt_dm_context;

/******************************************************
  *               Variable Definitions
  ******************************************************/
wiced_hci_bt_hfp_context_t wh_bt_hfp_context;

/******************************************************
  *               Function Definitions
  ******************************************************/

/*******************************************************************************
  **
  ** Function         wiced_bt_hfp_sink_utils_bdcpy
  **
  ** Description      Copy bd addr b to a.
  **
  ** Returns          void
  **
  *******************************************************************************/
    void wiced_bt_hfp_sink_utils_bdcpy(wiced_bt_device_address_t a,
        const wiced_bt_device_address_t b)
    {
        int i;
        for(i = BD_ADDR_LEN; i!=0; i--)
        {
            *a++ = *b++;
        }
    }

/*******************************************************************************
  **
  ** Function         wiced_bt_hfp_sink_utils_bdcmp
  **
  ** Description          Compare bd addr b to a.
  **
  ** Returns              Zero if b==a, nonzero otherwise (like memcmp).
  **
  *******************************************************************************/
  int wiced_bt_hfp_sink_utils_bdcmp(const wiced_bt_device_address_t a,
        const wiced_bt_device_address_t b)
    {
        int i;
        for(i = BD_ADDR_LEN; i!=0; i--)
        {
            if( *a++ != *b++ )
            {
                return -1;
            }
        }
        return 0;
    }

wiced_result_t wiced_bt_hfp_hf_init( wiced_bt_hfp_hf_config_data_t *p_config_data,
      wiced_bt_hfp_hf_event_cb_t event_cb )
{
    wiced_result_t result = WICED_SUCCESS;

    wh_bt_hfp_context.active_call = WICED_FALSE;

    /* copy the config data */
    wh_bt_hfp_context.config_data = p_config_data;

    /* Copy the callback information */
    wh_bt_hfp_context.p_event_cback = event_cb;

    /* set the event callback */
    wiced_hci_set_event_callback( HF, wiced_hci_hfp_cb );

    return result;
}

wiced_result_t wiced_bt_hfp_hf_deinit( void )
{
    memset(&wh_bt_hfp_context, 0, sizeof(wh_bt_hfp_context));

    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_hfp_hf_connect( wiced_bt_device_address_t bd_address )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t      length  = 0;
    wiced_result_t result = WICED_SUCCESS;

    length += ( sizeof(wiced_bt_device_address_t) );
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    BDADDR_TO_STREAM(p, bd_address);
    wiced_hci_send( HCI_CONTROL_HF_COMMAND_CONNECT , data, length );

    free(data);
    return result;
}

wiced_result_t wiced_bt_hfp_hf_disconnect( wiced_bt_device_address_t bd_address )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t      length  = 0;
    int             context_index;

    length += ( sizeof(uint16_t) );
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    context_index = get_hfp_context_index_by_addr(bd_address);
    if(context_index == HFP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADVALUE;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_hfp_remote_dev[context_index].handle);
    wiced_hci_send( HCI_CONTROL_HF_COMMAND_DISCONNECT , data, length );

    free(data);
    return result;
}

wiced_result_t wiced_bt_hfp_hf_perform_call_action( wiced_bt_device_address_t bd_address,
    wiced_bt_hfp_hf_call_action_t action, char* number )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*       data   = NULL;
    uint8_t*       ptr    = NULL;
    uint16_t       length = 0;
    uint32_t       opcode = 0;
    uint8_t        num = 0;
    int            context_index;

    WICED_HCI_DEBUG_LOG(("[%s] action = %d\n",__func__,action));

    switch(action)
    {
        case WICED_BT_HFP_HF_CALL_ACTION_ANSWER:
        {
            opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_A;
        }
        break;

        case WICED_BT_HFP_HF_CALL_ACTION_HANGUP:
        {
            opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_CHUP;
        }
        break;

        case WICED_BT_HFP_HF_CALL_ACTION_DIAL:
        {
            opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_BLDN;
        }
        break;

        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_0:
        {
            opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_CHLD;
            num = 0;
        }
        break;

        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_1:
        {
            opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_CHLD;
            num = 1;
        }
        break;

        case WICED_BT_HFP_HF_CALL_ACTION_HOLD_2:
        {
            opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_CHLD;
            num = 2;
        }
        break;

        default:
        /* do nothing */
        break;
    }

    length += ( sizeof(uint16_t) + sizeof(uint8_t) );
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    ptr = data;

    context_index = get_hfp_context_index_by_addr(bd_address);
    WICED_HCI_DEBUG_LOG(("context_index: %d\n",context_index));
    if(context_index == HFP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADVALUE;
    }
    UINT16_TO_STREAM(ptr, wiced_bt_hci_hfp_remote_dev[context_index].handle);
    UINT8_TO_STREAM( ptr, num );

    wiced_hci_send( opcode, data, length );

    free(data);

    return result;
}

wiced_result_t wiced_bt_hfp_hf_notify_volume( wiced_bt_device_address_t bd_address,
    wiced_bt_hfp_hf_volume_type_t volume_type, uint8_t volume_level )
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*       data   = NULL;
    uint8_t*       ptr    = NULL;
    uint16_t       length = 0;
    uint32_t       opcode;
    int            context_index;

    WICED_HCI_DEBUG_LOG(("[%s]\n",__func__));
    length += ( sizeof(uint16_t) + sizeof(uint8_t) );
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    ptr = data;

    if( volume_type == WICED_BT_HFP_HF_SPEAKER )
    {
        opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_SPK;
    }
    else
    {
        opcode = HCI_CONTROL_HF_AT_COMMAND_BASE + HCI_CONTROL_HF_AT_COMMAND_MIC;
    }

    context_index = get_hfp_context_index_by_addr(bd_address);
    if(context_index == HFP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADVALUE;
    }
    UINT16_TO_STREAM( ptr, wiced_bt_hci_hfp_remote_dev[context_index].handle );
    UINT8_TO_STREAM( ptr, volume_level );

    wiced_hci_send( opcode, data, length );

    free(data);
    return result;
}

/* stub fucntion */
wiced_result_t wiced_bt_hfp_hf_send_at_cmd( wiced_bt_device_address_t bd_address, char* at_cmd )
{
    wiced_result_t result = WICED_SUCCESS;

    return result;
}

static void wiced_hci_hfp_cb( uint16_t command, uint8_t *payload, uint32_t len )
{
    wiced_bt_hfp_hf_event_data_t event_data;
    uint8_t* p = payload;
    uint8_t status;
    uint16_t handle;
    uint8_t  hs_cmd;
    int      num_value;
    int      context_index;

    wiced_bt_management_evt_data_t p_event_data;

    WICED_HCI_DEBUG_LOG( ("[%s]\n",__func__) );

    switch( command )
    {
        case HCI_CONTROL_HF_EVENT_OPEN:
            {
                STREAM_TO_UINT16( handle, p );
                STREAM_TO_BDADDR( event_data.remote_address, p );
                STREAM_TO_UINT8( status, p );

                WPRINT_APP_INFO( ("HCI_CONTROL_HF_EVENT_OPEN:Status:%d handle:%x\n",status,handle) );
                context_index = set_hfp_context_index_peer(event_data.remote_address);
                if(context_index == HFP_CONN_MAX_LIMIT)
                {
                    WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                    return;
                }
                wiced_bt_hfp_sink_utils_bdcpy(wiced_bt_hci_hfp_remote_dev[context_index].peer_addr, event_data.remote_address);
                wiced_bt_hci_hfp_remote_dev[context_index].handle = handle;
                event_data.conn_state = WICED_BT_HFP_HF_STATE_CONNECTED;
                wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CONNECTION_STATE_EVT, &event_data );
            }
            break;

        case HCI_CONTROL_HF_EVENT_CLOSE:
            {
                WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_EVENT_CLOSE\n"));
                STREAM_TO_UINT16(handle, p);

                context_index = get_hfp_context_index_by_handle(handle);
                if(context_index == HFP_CONN_MAX_LIMIT)
                {
                    WICED_HCI_DEBUG_LOG(("%s: HF Connection Failed!!\n", __FUNCTION__));
                    memset(event_data.remote_address, 0, sizeof(wiced_bt_device_address_t));
                }
                else
                {
                    wiced_bt_hci_hfp_remote_dev[context_index].in_use = WICED_FALSE;
                    memcpy(event_data.remote_address,wiced_bt_hci_hfp_remote_dev[context_index].peer_addr,sizeof(wiced_bt_device_address_t));
                }
                event_data.conn_state = WICED_BT_HFP_HF_STATE_DISCONNECTED;
                wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CONNECTION_STATE_EVT, &event_data );

                if(wh_bt_hfp_context.active_call == WICED_TRUE)
                    wh_bt_hfp_context.active_call = WICED_FALSE;
            }
            break;

        case HCI_CONTROL_HF_EVENT_CONNECTED:
            {
                wiced_bt_hfp_hf_event_data_t event_data_for_feature_flag;

                STREAM_TO_UINT16( handle, p );
                STREAM_TO_UINT32( event_data_for_feature_flag.ag_feature_flags, p );

                WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_EVENT_CONNECTED:%s\n",__func__));

                context_index = get_hfp_context_index_by_handle(handle);
                if(context_index == HFP_CONN_MAX_LIMIT)
                {
                    WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                    return;
                }
                event_data.conn_state = WICED_BT_HFP_HF_STATE_SLC_CONNECTED;
                memcpy(event_data.remote_address,wiced_bt_hci_hfp_remote_dev[context_index].peer_addr,sizeof(wiced_bt_device_address_t));
                wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CONNECTION_STATE_EVT, &event_data );
                wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_AG_FEATURE_SUPPORT_EVT, &event_data_for_feature_flag );

                if(wh_bt_hfp_context.active_call == WICED_TRUE)
                    wh_bt_hfp_context.active_call = WICED_FALSE;
            }
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_OPEN:
            {
                WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_EVENT_AUDIO_OPEN\n"));
            }
                break;

        case HCI_CONTROL_HF_EVENT_AUDIO_CLOSE:
            {
                WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_EVENT_AUDIO_CLOSE\n"));

                STREAM_TO_UINT16(handle, p);

                context_index = get_hfp_context_index_by_handle(handle);
                if(context_index == HFP_CONN_MAX_LIMIT)
                {
                    WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!!\n", __FUNCTION__));
                }
                else
                {
                    p_event_data.sco_disconnected.sco_index = wiced_bt_hci_hfp_remote_dev[context_index].sco_index;
                }
                wh_bt_dm_context.dm_mgmt_cb( BTM_SCO_DISCONNECTED_EVT, &p_event_data );

                /* TODO: This needs to be called from an application API. */
                /* Turn off the PCM/I2S clock after the sco disconnects */
                wiced_hci_send( HCI_CONTROL_HF_COMMAND_TURN_OFF_PCM_CLK, NULL, 0 );
            }
            break;

        case HCI_CONTROL_HF_EVENT_AUDIO_CONN_REQ:
            {
                WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_EVENT_AUDIO_CONN_REQ\n"));

                STREAM_TO_BDADDR( p_event_data.sco_connection_request.bd_addr, p );
                STREAM_TO_UINT16( p_event_data.sco_connection_request.sco_index, p );

                context_index = get_hfp_context_index_by_addr(p_event_data.sco_connection_request.bd_addr);
                if(context_index == HFP_CONN_MAX_LIMIT)
                {
                    WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                    break;
                }
                wiced_bt_hci_hfp_remote_dev[context_index].sco_index = p_event_data.sco_connection_request.sco_index;

                WICED_HCI_DEBUG_LOG(("sco index:%d\n",p_event_data.sco_connection_request.sco_index));

                wh_bt_dm_context.dm_mgmt_cb( BTM_SCO_CONNECTION_REQUEST_EVT, &p_event_data );
            }
            break;

        default:
            {
                hs_cmd = command- HCI_CONTROL_HF_AT_EVENT_BASE;
                STREAM_TO_UINT16( handle, p );
                STREAM_TO_UINT16( num_value, p );
                hci_control_hf_at_command ( handle,hs_cmd, num_value, p );
            }
            break;
    }
}

void hci_control_hf_at_command (uint16_t handle, uint8_t command, int num_value, uint8_t *p_data)
{
    char                         at_cmd[255] = {0};
    uint8_t                      indicator_num;
    uint8_t                      indicator_val;
    wiced_bt_hfp_hf_event_data_t event_data;
    int                          context_index;

    context_index = get_hfp_context_index_by_handle(handle);
    if(context_index == HFP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        return;
    }
    memcpy(event_data.remote_address,wiced_bt_hci_hfp_remote_dev[context_index].peer_addr,sizeof(wiced_bt_device_address_t));

    if (p_data)
    {
        strncpy(at_cmd, (char *)p_data, (sizeof(at_cmd) - 1));
        at_cmd[(sizeof(at_cmd) - 1)] = '\0';
    }

    switch( command )
    {
        case HCI_CONTROL_HF_AT_EVENT_CIEV:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_CIEV:the at command:%s\n",at_cmd));
            /* at_cmd format: [space]indicator_num,indiactor_value. eg:[ 2,1]
                      * hence taking 0th and 2nd element only */
            indicator_num = at_cmd[0] - '0';
            indicator_val = at_cmd[2] - '0';

            switch( indicator_num )
            {
                case WICED_BT_HFP_HF_CALLSETUP_IND_NAME:
                    if( indicator_val == WICED_BT_HFP_INCOMING_CALL_PROCESS_ONGOING )
                    {
                        WICED_HCI_DEBUG_LOG(("incoming call\n"));
                        event_data.call_data.setup_state = indicator_val;
                        event_data.call_data.active_call_present =
                                (wh_bt_hfp_context.active_call == WICED_TRUE)?WICED_BT_HFP_CALL_IN_PROGRESS:WICED_BT_HFP_NO_CALL_IN_PROGRESS;
                        wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CALL_SETUP_EVT, &event_data );
                    }
                    else if(indicator_val == WICED_BT_HFP_NOT_IN_CALL_SETUP)
                    {
                        if( wh_bt_hfp_context.active_call == WICED_TRUE )
                        {
                            WICED_HCI_DEBUG_LOG(("active call present\n"));
                            event_data.call_data.active_call_present = WICED_BT_HFP_CALL_IN_PROGRESS;
                        }
                        else
                        {
                            WICED_HCI_DEBUG_LOG(("active call not present\n"));
                            event_data.call_data.active_call_present = WICED_BT_HFP_NO_CALL_IN_PROGRESS;
                        }
                        event_data.call_data.setup_state = indicator_val;
                        wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CALL_SETUP_EVT, &event_data );
                    }
                    else if(indicator_val == WICED_BT_HFP_OUTGOING_CALL_SETUP_ONGOING)
                    {
                        WICED_HCI_DEBUG_LOG(("outgoing call setup going on\n"));
                        event_data.call_data.active_call_present =
                                (wh_bt_hfp_context.active_call == WICED_TRUE)?WICED_BT_HFP_CALL_IN_PROGRESS:WICED_BT_HFP_NO_CALL_IN_PROGRESS;
                        event_data.call_data.setup_state = indicator_val;
                        wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CALL_SETUP_EVT, &event_data );
                    }
                    break;

                case WICED_BT_HFP_HF_SIGNAL_IND_NAME:
                    WICED_HCI_DEBUG_LOG(("WICED_BT_HFP_HF_SIGNAL_IND_NAME, value=%d\n",indicator_val));
                    break;

                case WICED_BT_HFP_HF_CALL_IND_NAME:
                    WICED_HCI_DEBUG_LOG(("WICED_BT_HFP_HF_CALL_IND_NAME, value=%d\n",indicator_val));
                    if(indicator_val == WICED_BT_HFP_CALL_IN_PROGRESS )
                    {
                        WICED_HCI_DEBUG_LOG(("call in progress\n"));
                        wh_bt_hfp_context.active_call = WICED_TRUE;
                        event_data.call_data.setup_state = indicator_val;
                    }
                    else if(indicator_val == WICED_BT_HFP_NO_CALL_IN_PROGRESS)
                    {
                        WICED_HCI_DEBUG_LOG(("no call in progress, call disconnecting\n"));
                        event_data.call_data.setup_state = indicator_val;
                        wh_bt_hfp_context.active_call = WICED_FALSE;
                        wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CALL_SETUP_EVT, &event_data );
                    }
                    break;

                case WICED_BT_HFP_HF_SERVICE_IND_NAME:
                    WICED_HCI_DEBUG_LOG(("WICED_BT_HFP_HF_SERVICE_IND_NAME, value=%d\n",indicator_val));
                    break;

                case WICED_BT_HFP_HF_CALL_HELD_IND_NAME:
                    WICED_HCI_DEBUG_LOG(("WICED_BT_HFP_HF_CALL_HELD_IND_NAME, value=%d\n",indicator_val));
                    break;

                case WICED_BT_HFP_HF_ROAM_IND_NAME:
                    WICED_HCI_DEBUG_LOG(("WICED_BT_HFP_HF_ROAM_IND_NAME, value=%d\n",indicator_val));
                    break;

                case WICED_BT_HFP_HF_BATTERY_IND_NAME:
                    WICED_HCI_DEBUG_LOG(("WICED_BT_HFP_HF_BATTERY_IND_NAME, value=%d\n",indicator_val));
                    break;

                default:
                    WICED_HCI_DEBUG_LOG(("this is the default case \n"));
                    break;
            }

            break;

        case HCI_CONTROL_HF_AT_EVENT_OK:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_OK\n"));
            break;

        case HCI_CONTROL_HF_AT_EVENT_RING:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_RING\n"));
            wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_RING_EVT, &event_data );

            break;

        case HCI_CONTROL_HF_AT_EVENT_CHLD:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_CHLD\n"));
            break;

        case HCI_CONTROL_HF_AT_EVENT_CIND:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_CIND\n"));
            break;

        case HCI_CONTROL_HF_AT_EVENT_CLIP:
            strncpy( event_data.clip.caller_num, at_cmd, sizeof(event_data.clip.caller_num) - 1 );
            event_data.clip.caller_num[sizeof(event_data.clip.caller_num) - 1] = '\0';
            event_data.clip.type = num_value;

            wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_CLIP_IND_EVT, &event_data );
            break;

        case HCI_CONTROL_HF_AT_EVENT_BINP:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_BINP\n"));
            break;

        case HCI_CONTROL_HF_AT_EVENT_CCWA:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_CCWA\n"));
            break;

        case HCI_CONTROL_HF_AT_EVENT_VGS:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_VGS %d\n",num_value));
            event_data.volume.level = num_value;
            event_data.volume.type = 0;

            wh_bt_hfp_context.p_event_cback( WICED_BT_HFP_HF_VOLUME_CHANGE_EVT, &event_data );
            break;

        case HCI_CONTROL_HF_AT_EVENT_MAX:
            WICED_HCI_DEBUG_LOG(("HCI_CONTROL_HF_AT_EVENT_UNAT or HCI_CONTROL_HF_AT_EVENT_MAX \n"));
            break;

        default:
            WICED_HCI_DEBUG_LOG(("[%s]default at_command:%x\n",__func__,command));
            break;
    }
}

/* sco functions */
wiced_bt_dev_status_t wiced_bt_sco_set_data_callback ( wiced_bt_sco_data_cback_t *p_cback )
{
    wiced_result_t result = WICED_SUCCESS;

    return result;
}

wiced_bt_dev_status_t wiced_bt_sco_set_buffer_pool( uint16_t buffer_size,
                            uint16_t buffer_count )
{
    wiced_result_t result = WICED_SUCCESS;

    return result;
}

wiced_bt_dev_status_t wiced_bt_sco_accept_connection (uint16_t sco_index, uint8_t hci_status,
                                     wiced_bt_sco_esco_codec_setting_id_t esco_set_id)
{
    wiced_result_t result = WICED_SUCCESS;
    uint8_t*       data   = NULL;
    uint8_t*       ptr    = NULL;
    uint16_t       length = 0;

    WICED_HCI_DEBUG_LOG(("[%s] [%d]\n",__func__,hci_status));

    if( hci_status == WICED_BT_SCO_CONNECTION_ACCEPT )
    {
        hci_status = ~hci_status;
    }

    length += ( sizeof(uint16_t) + 1 );
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    ptr = data;

    UINT16_TO_STREAM( ptr, sco_index );
    UINT8_TO_STREAM( ptr, hci_status);

    wiced_hci_send( HCI_CONTROL_HF_COMMAND_AUDIO_ACCEPT_CONN, data, length );
    free(data);

    return result;

}

/* stub functions */
wiced_bt_dev_status_t wiced_bt_sco_create_as_acceptor (uint16_t *p_sco_index)
{
    wiced_result_t result = WICED_SUCCESS;

    return result;
}

wiced_bt_dev_status_t wiced_bt_sco_remove (uint16_t sco_index)
{
    wiced_result_t result = WICED_SUCCESS;

    return result;
}

static int get_hfp_context_index_by_addr(wiced_bt_device_address_t remote_addr)
{
    int i=0;

    while(i < HFP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_hfp_remote_dev[i].in_use == WICED_TRUE) &&
            (!wiced_bt_hfp_sink_utils_bdcmp(wiced_bt_hci_hfp_remote_dev[i].peer_addr, remote_addr)))
        {
            WICED_HCI_DEBUG_LOG(( "get_hfp_context_index_by_addr i:%d\n",i ));
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_hfp_context_index_by_addr Invalid!!\n" ));
    return HFP_CONN_MAX_LIMIT;
}

static int get_hfp_context_index_by_handle(uint16_t    handle)
{
    int i=0;

    while(i < HFP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_hfp_remote_dev[i].in_use == WICED_TRUE) &&
            (wiced_bt_hci_hfp_remote_dev[i].handle == handle))
        {
            WICED_HCI_DEBUG_LOG(( "get_hfp_context_index_by_handle Already present i:%d\n",i ));
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_hfp_context_index_by_handle Invalid!!\n" ));
    return HFP_CONN_MAX_LIMIT;
}

static int set_hfp_context_index_peer(wiced_bt_device_address_t remote_addr)
{
    int i=0;
    while(i < HFP_CONN_MAX_LIMIT)
    {
        if(wiced_bt_hci_hfp_remote_dev[i].in_use == WICED_TRUE)
        {
            if(!wiced_bt_hfp_sink_utils_bdcmp(wiced_bt_hci_hfp_remote_dev[i].peer_addr, remote_addr))
             {
                 WICED_HCI_DEBUG_LOG(( "set_context_index Already present i:%d\n",i ));
                 return i;
             }
             else
             {
                 i++;
                 continue;
             }
        }
        else
        {
            wiced_bt_hci_hfp_remote_dev[i].in_use = WICED_TRUE;
            WICED_HCI_DEBUG_LOG( ( "set_context_index New entry i:%d\n",i ));
            return i;
        }
    }
    WICED_HCI_DEBUG_LOG(("set_hfp_context_index_peer Invalid!!\n" ));
    return HFP_CONN_MAX_LIMIT;
}

