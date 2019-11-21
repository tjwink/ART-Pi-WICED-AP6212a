/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * @file wiced_hci_bt_avrcp.c
 *
 * Bluetooth AVRC Remote Control Application WICED HCI Interface
 *
 */
#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_remote_control.h"
#include "wwd_debug.h"
#include "wiced_hci_bt_internal_common.h"

/******************************************************
 *                    Constants
 ******************************************************/
#define AVRCP_CONN_MAX_LIMIT     2

/******************************************************
 *                   Structures
 ******************************************************/
typedef struct
{
    UINT32                                           local_features;
    wiced_bt_remote_control_connection_state_cback_t connection_cb;
    wiced_bt_remote_control_cmd_cback_t              cmd_cb;
    wiced_bt_remote_control_rsp_cback_t              rsp_cb;
} rcc_hci_cb_t;

typedef struct
{
    UINT32                                           peer_features;
    wiced_bt_device_address_t                        peer_bd_addr;
    uint16_t                                         handle;
    wiced_bool_t                                     in_use;
} wiced_bt_hci_avrc_remote_dev_t;

/*****************************************************************************
** Global data
*****************************************************************************/
static rcc_hci_cb_t               rcc_hci_cb;
wiced_bt_hci_avrc_remote_dev_t    wiced_bt_hci_avrcp_remote_dev[AVRCP_CONN_MAX_LIMIT] =
{
    {
        .peer_features = 0,
        .peer_bd_addr  = {0,},
        .handle        = 0,
        .in_use        = WICED_FALSE
    },
    {
        .peer_features = 0,
        .peer_bd_addr  = {0,},
        .handle        = 0,
        .in_use        = WICED_FALSE
    }
};

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void wiced_hci_avrc_controller_cb(uint16_t event, uint8_t* payload, uint32_t len);
static int get_rc_context_index_by_addr(wiced_bt_device_address_t remote_addr);
static int get_rc_context_index_by_handle(uint16_t    handle);
static int set_rc_context_index_peer(wiced_bt_device_address_t remote_addr);

/******************************************************
 *               Function Definitions
 ******************************************************/
/*******************************************************************************
**
** Function         wiced_bt_avrcp_sink_utils_bdcpy
**
** Description      Copy bd addr b to a.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_avrcp_sink_utils_bdcpy(wiced_bt_device_address_t a,
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
** Function         wiced_bt_avrcp_sink_utils_bdcmp
**
** Description          Compare bd addr b to a.
**
** Returns              Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
int wiced_bt_avrcp_sink_utils_bdcmp(const wiced_bt_device_address_t a,
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

/****************************************************************************/
/**
 * Function         wiced_bt_remote_control_init
 *
 *                  Initialize the AVRC controller and start listening for incoming connections
 *
 * @param[in]       local_features      : Local supported features mask
 *                                        Combination of wiced_bt_remote_control_features_t
 * @param[in]       p_connection_cback  : Callback for connection state
 * @param[in]       p_rsp_cb            : Callback from peer device in response to AVRCP commands
 * @param[in]       p_cmd_cb            : Callback when peer device sends AVRCP commands
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_init(uint32_t local_features,
                    wiced_bt_remote_control_connection_state_cback_t p_connection_cb,
                    wiced_bt_remote_control_cmd_cback_t p_cmd_cb,
                    wiced_bt_remote_control_rsp_cback_t p_rsp_cb)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;

    WICED_HCI_DEBUG_LOG((" %s: [%d] \n", __func__, (int)local_features));
    memset ( &rcc_hci_cb, 0, sizeof(rcc_hci_cb) );

    rcc_hci_cb.local_features = local_features;
    rcc_hci_cb.connection_cb = p_connection_cb;
    rcc_hci_cb.cmd_cb = p_cmd_cb;
    rcc_hci_cb.rsp_cb = p_rsp_cb;

    wiced_hci_set_event_callback(AVRC_CONTROLLER, wiced_hci_avrc_controller_cb);
    wiced_hci_set_event_callback(AVRC_TARGET, wiced_hci_avrc_controller_cb);

    length += sizeof(uint32_t);

    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    UINT32_TO_STREAM(p, local_features);

    /*Comenting below code since It is decided that 2070X Embedded application itself handles this functionality.
         So this function can made stub function later. */
  //  wiced_hci_send(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_INIT, data, length);

    /* free the data */
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_remote_control_deinit(void)
{
    memset(&rcc_hci_cb, 0, sizeof(rcc_hci_cb));
    memset(&wiced_bt_hci_avrcp_remote_dev, 0 , sizeof(wiced_bt_hci_avrcp_remote_dev));

    return WICED_SUCCESS;
}

/**
 * Function         wiced_bt_remote_control_connect
 *
 *                  Initiate connection to the peer AVRC target device.
 *                  After connection establishment, stop listening for incoming connections
 *
 * @param[in]       remote_addr : Bluetooth address of peer device
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_connect( wiced_bt_device_address_t remote_addr)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t      length  = 0;

    length += sizeof(wiced_bt_device_address_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    BDADDR_TO_STREAM(p, remote_addr);
    wiced_hci_send(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_CONNECT, data, length);
    free(data);
    return WICED_SUCCESS;
}

/**
 * Function         wiced_bt_remote_control_disconnect
 *
 *                  Disconnect from the peer AVRC target device
 *                  After disconnection , start listening for incoming connections
 *
 * @param[in]       remote_addr : Bluetooth address of connected peer device
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_disconnect( wiced_bt_device_address_t remote_addr)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length += sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_rc_context_index_by_addr(remote_addr);
    if(context_index == AVRCP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_avrcp_remote_dev[context_index].handle);

    wiced_hci_send(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_DISCONNECT, data, length);
    free(data);
    return WICED_SUCCESS;
}

static uint16_t wiced_bt_remote_control_translate_to_hci_command( uint8_t event_id )
{
    switch( event_id )
    {
        case AVRC_ID_PLAY:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PLAY;
        case AVRC_ID_PAUSE:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PAUSE;
        case AVRC_ID_STOP:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_STOP;
        case AVRC_ID_FORWARD:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_NEXT_TRACK;
        case AVRC_ID_BACKWARD:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_PREVIOUS_TRACK;
        case AVRC_ID_VOL_UP:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_UP;
        case AVRC_ID_VOL_DOWN:
            return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_DOWN;
        //TODO
        /* Currently application is not handling AVRC_ID_FAST_FOR/AVRC_ID_REWIND but to handle them needs to rethink implementation of
        wiced_bt_remote_control_send_pass_through_cmd() because we are ignoring AVRC_STATE_RELEASE case.

        case AVRC_ID_FAST_FOR:
                return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_FAST_FORWARD/HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_FAST_FORWARD
        case AVRC_ID_REWIND:
               return HCI_CONTROL_AVRC_CONTROLLER_COMMAND_BEGIN_REWIND/HCI_CONTROL_AVRC_CONTROLLER_COMMAND_END_REWIND
        */
    }
    return 0x00;
}

/**
 * Function         wiced_bt_remote_control_send_pass_through_cmd
 *
 *                  Send PASS THROUGH command
 *
 * @param[in]       remote_addr     : Bluetooth address of connected peer device
 * @param[in]       cmd             : Pass through command id (see #AVRC_ID_XX)
 * @param[in]       state           : State of the pass through command (see #AVRC_STATE_XX)
 * @param[in]       data_field_len  : Data field length
 * @param[in]       data_field      : Data field
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_send_pass_through_cmd(
                    wiced_bt_device_address_t remote_addr, uint8_t cmd, uint8_t state,
                    uint8_t data_len, uint8_t *data )
{
    uint8_t*        data_hci  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    uint16_t        command = 0;
    int             context_index;

/**
        Application involkes this function twice once for AVRC_STATE_PRESS and later for AVRC_STATE_RELEASE. eg:
                result = wiced_bt_remote_control_send_pass_through_cmd(a2dp_context.peer_address, cmd, AVRC_STATE_PRESS, 0, NULL);
                wiced_rtos_delay_milliseconds( 50 );
                result = wiced_bt_remote_control_send_pass_through_cmd(a2dp_context.peer_address, cmd, AVRC_STATE_RELEASE, 0, NULL);
         But 2070x needs pass_through only once since it internally handles RELEASE KEY. So inorder not to change existing application code, we have decided to ignore
         function called for AVRC_STATE_RELEASE.
*/

    WICED_HCI_DEBUG_LOG((" %s: cmd:%x, state:%x \n", __func__, cmd, state));

    if (state == AVRC_STATE_RELEASE)
        return WICED_SUCCESS;

    length += sizeof(uint16_t);
    /* allocate memory for the data */
    data_hci = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data_hci;
    context_index = get_rc_context_index_by_addr(remote_addr);
    if(context_index == AVRCP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data_hci);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_avrcp_remote_dev[context_index].handle);
    command = wiced_bt_remote_control_translate_to_hci_command(cmd);

    wiced_hci_send(command, data_hci, length);
    free(data_hci);
    return WICED_SUCCESS;
}

/*****************************************************************************
 *  VOLUME FUNCTIONS
 ****************************************************************************/
/**
 * Function         wiced_bt_remote_control_set_volume_cmd
 *
 *                  Set volume for peer device
 *
 * @param[in]       remote_addr : Bluetooth address of connected peer device
 * @param[in]       volume      : Volume
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_set_volume_cmd( wiced_bt_device_address_t remote_addr, uint8_t volume )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    uint8_t         volume_level;
    int             context_index;

    volume_level = (uint8_t)((double)((double)volume * 100.0)/(double)127.0);

    length += sizeof(uint16_t) + sizeof(uint8_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_rc_context_index_by_addr(remote_addr);
    if(context_index == AVRCP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_avrcp_remote_dev[context_index].handle);
    UINT8_TO_STREAM(p, volume_level);
    wiced_hci_send(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_VOLUME_LEVEL, data, length);
    free(data);
    return WICED_SUCCESS;
}

/**
 * Function         wiced_bt_remote_control_get_element_attr_cmd
 *
 *                  Requests the target device to provide the attributes
 *                  of the element specified in the parameter
 *
 * @param[in]       remote_addr : Bluetooth address of connected peer device
 * @param[in]       element_id  : Element id
 * @param[in]       num_attr    : Number of attributes
 * @param[in]       p_attrs     : Media attribute ids (see #AVRC_MEDIA_ATTR_ID)
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_get_element_attr_cmd(
                    wiced_bt_device_address_t remote_addr, wiced_bt_avrc_uid_t element_id,
                    uint8_t num_attr, uint32_t *p_attrs)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    uint8_t         index;
    int             context_index;

    length += sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t)* num_attr;
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_rc_context_index_by_addr(remote_addr);
    if(context_index == AVRCP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_avrcp_remote_dev[context_index].handle);
    UINT8_TO_STREAM(p, num_attr);
    for(index=0; index<num_attr; index++)
    {
        *p = p_attrs[index] & 0xff;
        p++;
    }
    wiced_hci_send(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_GET_TRACK_INFO, data, length);
    free(data);
    return WICED_SUCCESS;
}


/**
 * Function         wiced_bt_remote_control_get_play_status_cmd
 *
 *                  Get the status of the currently playing media at the TG
 *
 * @param[in]       remote_addr : Bluetooth address of connected peer device
 *
 * @return          wiced_result_t
 *
 */
wiced_result_t wiced_bt_remote_control_get_play_status_cmd( wiced_bt_device_address_t remote_addr )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length += sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_rc_context_index_by_addr(remote_addr);
    if(context_index == AVRCP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_avrcp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AVRC_CONTROLLER_COMMAND_GET_PLAY_STATUS, data, length);
    free(data);
    return WICED_SUCCESS;
}

static int get_rc_context_index_by_addr(wiced_bt_device_address_t remote_addr)
{
    int i=0;

    while(i < AVRCP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_avrcp_remote_dev[i].in_use == WICED_TRUE) &&
            (!wiced_bt_avrcp_sink_utils_bdcmp(wiced_bt_hci_avrcp_remote_dev[i].peer_bd_addr, remote_addr)))
        {
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_rc_context_index_by_addr Invalid!!\n" ));
    return AVRCP_CONN_MAX_LIMIT;
}

static int get_rc_context_index_by_handle(uint16_t    handle)
{
    int i=0;

    while(i < AVRCP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_avrcp_remote_dev[i].in_use == WICED_TRUE) &&
            (wiced_bt_hci_avrcp_remote_dev[i].handle == handle))
        {
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_rc_context_index_by_handle Invalid!!\n" ));
    return AVRCP_CONN_MAX_LIMIT;
}

static int set_rc_context_index_peer(wiced_bt_device_address_t remote_addr)
{
    int i=0;
    while(i < AVRCP_CONN_MAX_LIMIT)
    {
        if(wiced_bt_hci_avrcp_remote_dev[i].in_use == WICED_TRUE)
        {
            if(!wiced_bt_avrcp_sink_utils_bdcmp(wiced_bt_hci_avrcp_remote_dev[i].peer_bd_addr, remote_addr))
             {
                 WICED_HCI_DEBUG_LOG(( "set_rc_context_index_peer Already present i:%d\n",i ));
                 return i;
             }
             else
             {
                 i=i+1;
                 continue;
             }
        }
        else
        {
            wiced_bt_hci_avrcp_remote_dev[i].in_use = WICED_TRUE;
            WICED_HCI_DEBUG_LOG( ( "set_rc_context_index_peer New entry i:%d\n",i ));
            return i;
        }
    }
    WICED_HCI_DEBUG_LOG(("set_rc_context_index_peer Invalid!!\n" ));
    return AVRCP_CONN_MAX_LIMIT;
}

static void wiced_hci_avrc_controller_cb(uint16_t event, uint8_t* payload, uint32_t len)
{
    uint8_t*                     p = payload;
    wiced_bt_avrc_response_t     avrc_rsp;
    uint16_t                     handle;
    int                          context_index;

    WICED_HCI_DEBUG_LOG(("%s event:%x\n", __func__, event));

    switch(event)
    {
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_COMMAND_STATUS:
        {
            WICED_HCI_DEBUG_LOG(("%s: command status:%d\n", __FUNCTION__, *p));
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED:
        {
            uint8_t                      status = 0;
            wiced_bt_device_address_t    bd_addr_temp;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_CONNECTED\n", __FUNCTION__));
            STREAM_TO_BDADDR(bd_addr_temp, p);
            context_index = set_rc_context_index_peer(bd_addr_temp);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_avrcp_sink_utils_bdcpy(wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, bd_addr_temp);
            STREAM_TO_UINT8(status, p);
            WICED_HCI_DEBUG_LOG(("%s status:%d\n",__FUNCTION__, status));
            UNUSED_PARAMETER(status);
            STREAM_TO_UINT16(wiced_bt_hci_avrcp_remote_dev[context_index].handle, p);
            STREAM_TO_UINT8(wiced_bt_hci_avrcp_remote_dev[context_index].peer_features, p);

            (rcc_hci_cb.connection_cb)(wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, WICED_SUCCESS, REMOTE_CONTROL_CONNECTED,
                (uint32_t)wiced_bt_hci_avrcp_remote_dev[context_index].peer_features);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_DISCONNECTED\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                wiced_bt_device_address_t    bd_addr_temp;
                memset( bd_addr_temp, 0, sizeof(wiced_bt_device_address_t) );
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                (rcc_hci_cb.connection_cb)(bd_addr_temp, WICED_SUCCESS, REMOTE_CONTROL_DISCONNECTED,0);
            }
            else
            {
                (rcc_hci_cb.connection_cb)(wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, WICED_SUCCESS, REMOTE_CONTROL_DISCONNECTED,
                (uint32_t)wiced_bt_hci_avrcp_remote_dev[context_index].peer_features);
                wiced_bt_hci_avrcp_remote_dev[context_index].in_use = WICED_FALSE;
                memset(wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, 0, sizeof(wiced_bt_device_address_t));
                wiced_bt_hci_avrcp_remote_dev[context_index].handle = 0;
            }
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS_INFO:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS_INFO\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }

            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            avrc_rsp.pdu = AVRC_PDU_GET_PLAY_STATUS;
            STREAM_TO_UINT8(avrc_rsp.get_play_status.play_status, p);
            STREAM_TO_UINT32(avrc_rsp.get_play_status.song_len,p);
            STREAM_TO_UINT32(avrc_rsp.get_play_status.song_pos,p)

            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO:
        {
            uint16_t     attr_name_strlen;
            uint8_t      attr_id;
            uint8_t      avrc_response_status;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_CURRENT_TRACK_INFO\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            STREAM_TO_UINT8(avrc_response_status, p);
            UNUSED_PARAMETER(avrc_response_status);
            STREAM_TO_UINT8(attr_id, p);
            STREAM_TO_UINT16(attr_name_strlen, p);
            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            avrc_rsp.pdu = AVRC_PDU_GET_ELEMENT_ATTR;
            avrc_rsp.get_elem_attrs.p_attrs = (wiced_bt_avrc_attr_entry_t*)calloc(1, sizeof(wiced_bt_avrc_attr_entry_t));
            avrc_rsp.get_elem_attrs.p_attrs->attr_id = (uint32_t) attr_id;
            avrc_rsp.get_elem_attrs.p_attrs->name.str_len = attr_name_strlen;
            avrc_rsp.get_elem_attrs.p_attrs->name.p_str = (uint8_t*) calloc(attr_name_strlen, sizeof(uint8_t));
            memcpy(avrc_rsp.get_elem_attrs.p_attrs->name.p_str, p, attr_name_strlen);

            //Todo: Currently 2070x sends a single attribute ID and attribute string at a time. Need to check the app code?
            avrc_rsp.get_elem_attrs.num_attr = 1;
            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_STATUS\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            STREAM_TO_UINT8(avrc_rsp.get_play_status.play_status, p);
            avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
            avrc_rsp.reg_notif.event_id = AVRC_EVT_PLAY_STATUS_CHANGE;

            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_CHANGE:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_CHANGE\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
            avrc_rsp.reg_notif.event_id = AVRC_EVT_TRACK_CHANGE;
            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_END:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_END\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
            avrc_rsp.reg_notif.event_id = AVRC_EVT_TRACK_REACHED_END;
            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_START:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AVRC_CONTROLLER_EVENT_TRACK_START\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
            avrc_rsp.reg_notif.event_id = AVRC_EVT_TRACK_REACHED_START;
            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_CONTROLLER_EVENT_PLAY_POSITION:
        {
            STREAM_TO_UINT16(handle, p);
            context_index = get_rc_context_index_by_handle(handle);
            if(context_index == AVRCP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            //TODO: play position is not being used by application, if needed we need to extract 4bytes and send to App.
            memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
            avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
            avrc_rsp.reg_notif.event_id = AVRC_EVT_PLAY_POS_CHANGED;
            rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        case HCI_CONTROL_AVRC_TARGET_EVENT_VOLUME_LEVEL:
        {
             uint8_t volume_level;
             uint8_t volume;
             wiced_bt_avrc_command_t avrc_cmd;

             STREAM_TO_UINT16(handle, p);
             WICED_HCI_DEBUG_LOG(("%s Absolute volume changed on Remote-device handle: %d\n", __FUNCTION__, handle));
             context_index = get_rc_context_index_by_handle(handle);
             if(context_index == AVRCP_CONN_MAX_LIMIT)
             {
                 WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                 break;
             }
             STREAM_TO_UINT8(volume_level, p);
             avrc_cmd.pdu = AVRC_PDU_SET_ABSOLUTE_VOLUME;
             volume = (uint8_t)((double)((double)volume_level * 127.0)/(double)100.0);
             avrc_cmd.volume.volume = volume;
             rcc_hci_cb.cmd_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_cmd);

             memset(&avrc_rsp,0,sizeof(wiced_bt_avrc_response_t));
             avrc_rsp.pdu = AVRC_PDU_REGISTER_NOTIFICATION;
             avrc_rsp.reg_notif.event_id = AVRC_EVT_VOLUME_CHANGE;
             avrc_rsp.reg_notif.param.volume = volume;
             rcc_hci_cb.rsp_cb( wiced_bt_hci_avrcp_remote_dev[context_index].peer_bd_addr, &avrc_rsp);
        }
        break;
        default:
            WICED_HCI_DEBUG_LOG(("%s EVENT not handled\n", __FUNCTION__));
        break;
    }
}
