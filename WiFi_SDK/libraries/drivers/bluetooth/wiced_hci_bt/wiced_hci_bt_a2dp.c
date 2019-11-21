/*
 * $ Copyright Broadcom Corporation $
 */

/**
 * @file wiced_hci_bt_a2dp.c
 *
 * This file contains implementation of the audio sink interface.
 *
 */
#include <string.h>
#include <stdlib.h>
#include "wiced_hci.h"
#include "wiced_bt_stack.h"
#include "wiced_bt_dev.h"
#include "wiced_bt_a2dp_sink.h"
#include "wwd_debug.h"
#include "wiced_hci_bt_internal_common.h"

/******************************************************
 *                    Constants
 ******************************************************/

#define AV_SOURCE_ROLE              0x00
#define AV_SINK_ROLE                0x01

#define A2DP_CONN_MAX_LIMIT     2

/******************************************************
 *                   Structures
 ******************************************************/
/* Type for A2DP sink control block */
typedef struct
{
    wiced_bt_a2dp_codec_info_t       codec_info;
    wiced_bt_a2dp_sink_control_cb_t  control_cb;     /* Application registered control callback function */
} wiced_bt_hci_a2dp_sink_cb_t;

typedef struct
{
    wiced_bt_device_address_t        peer_addr;
    uint16_t                         handle;
    wiced_bool_t                     in_use;
    uint8_t                          av_stream_label; /* This is updated on AV_START ind and later used in AV_START RSP while responding to 20706 */
} wiced_bt_hci_a2dp_remote_dev_t;

/*****************************************************************************
** Global data
*****************************************************************************/
/* A2DP sink control block */
wiced_bt_hci_a2dp_sink_cb_t         wiced_bt_hci_a2dp_sink_cb;
wiced_bt_hci_a2dp_remote_dev_t      wiced_bt_hci_a2dp_remote_dev[A2DP_CONN_MAX_LIMIT] =
{
    {
        .peer_addr       = {0,},
        .handle          = 0,
        .in_use          = WICED_FALSE,
        .av_stream_label = 0
    },
    {
        .peer_addr       = {0,},
        .handle          = 0,
        .in_use          = WICED_FALSE,
        .av_stream_label = 0
    }
};

wiced_bt_a2dp_sink_data_cb_t        wiced_bt_hci_a2dp_sink_data_cb; //Data Callback

/******************************************************
 *               Static Function Declarations
 ******************************************************/
static void wiced_hci_audio_cb(uint16_t event, uint8_t* payload, uint32_t len);
static int get_a2d_context_index_by_addr(wiced_bt_device_address_t remote_addr);
static int get_a2d_context_index_by_handle(uint16_t    handle);
static int set_a2d_context_index_peer(wiced_bt_device_address_t remote_addr);

/******************************************************
 *               Function Definitions
 ******************************************************/

/*******************************************************************************
**
** Function         wiced_bt_a2dp_sink_utils_bdcpy
**
** Description      Copy bd addr b to a.
**
** Returns          void
**
*******************************************************************************/
void wiced_bt_a2dp_sink_utils_bdcpy(wiced_bt_device_address_t a,
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
** Function         wiced_bt_a2dp_sink_utils_bdcmp
**
** Description          Compare bd addr b to a.
**
** Returns              Zero if b==a, nonzero otherwise (like memcmp).
**
*******************************************************************************/
int wiced_bt_a2dp_sink_utils_bdcmp(const wiced_bt_device_address_t a,
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

wiced_result_t wiced_bt_a2dp_sink_init(wiced_bt_a2dp_config_data_t *p_config_data,
    wiced_bt_a2dp_sink_control_cb_t control_cb,
    wiced_bt_a2dp_sink_data_cb_t data_cb)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t      length  = 0;
    uint16_t      length_of_codec_info;

    if(p_config_data == NULL || control_cb == NULL)
    {
        WPRINT_APP_ERROR(("%s: Invalid input parameters", __FUNCTION__));
        return WICED_BADARG;
    }

    /* Copy the callback information */
    wiced_bt_hci_a2dp_sink_cb.control_cb = control_cb;

    wiced_hci_set_event_callback(AUDIO_SINK, wiced_hci_audio_cb);

    if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_SINK_CODEC_SBC)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_sbc_cie_t);
    else if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_SINK_CODEC_M12)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_m12_cie_t);
    else if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_SINK_CODEC_M24)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_m24_cie_t);
    else if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_SINK_CODEC_M24)
        length_of_codec_info = sizeof(uint8_t)  + sizeof(wiced_bt_a2d_vendor_cie_t);
    else
        return WICED_BADARG;

    length += ( sizeof(uint8_t) + sizeof(uint8_t) + sizeof(uint16_t) + length_of_codec_info);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    UINT8_TO_STREAM(p, AV_SINK_ROLE);
    UINT8_TO_STREAM(p, p_config_data->feature_mask & 0xff);
    UINT16_TO_STREAM(p, length_of_codec_info);

    if(p_config_data->codec_capabilities.info->codec_id == WICED_BT_A2DP_SINK_CODEC_SBC)
    {
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->codec_id);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.samp_freq);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.ch_mode);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.block_len);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.num_subbands);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.alloc_mthd);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.max_bitpool);
        UINT8_TO_STREAM(p, p_config_data->codec_capabilities.info->cie.sbc.min_bitpool);
    }
    wiced_bt_hci_a2dp_sink_data_cb = data_cb;
    //TODO: Handling of Non SBC codec details.

 /*Comenting below code since It is decided that 2070X Embedded application itself handles this functionality.
      So this function can made stub function later. */
//    wiced_hci_send(HCI_CONTROL_AUDIO_COMMAND_INIT, data, length);

    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_deinit(void)
{
    memset(&wiced_bt_hci_a2dp_sink_cb, 0 ,sizeof(wiced_bt_hci_a2dp_sink_cb));
    memset(&wiced_bt_hci_a2dp_remote_dev, 0, sizeof(wiced_bt_hci_a2dp_remote_dev));
    memset(&wiced_bt_hci_a2dp_sink_data_cb, 0, sizeof(wiced_bt_hci_a2dp_sink_data_cb));

    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_connect(wiced_bt_device_address_t bd_address)
{
    uint8_t*                data  = NULL;
    uint8_t*                p     = NULL;
    uint16_t                length  = 0;


    length += ( sizeof(wiced_bt_device_address_t) + sizeof(uint8_t));
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    BDADDR_TO_STREAM(p, bd_address);
    UINT8_TO_STREAM(p, WICED_BT_A2DP_ROUTE_I2S);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_CONNECT, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_disconnect(wiced_bt_device_address_t bd_address)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_addr(bd_address);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_DISCONNECT, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_start(wiced_bt_device_address_t bd_address)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_addr(bd_address);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_START, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_start_rsp( wiced_bt_device_address_t bd_address, uint8_t response)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint8_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;

    context_index = get_a2d_context_index_by_addr(bd_address);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    WICED_HCI_DEBUG_LOG(("%s  handle:%x label:%x response:%x\n", __FUNCTION__, wiced_bt_hci_a2dp_remote_dev[context_index].handle,
                      wiced_bt_hci_a2dp_remote_dev[context_index].av_stream_label, response));

    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    UINT8_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].av_stream_label);
    UINT8_TO_STREAM(p, response);

    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_START_RSP, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_suspend(wiced_bt_device_address_t bd_address)
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_addr(bd_address);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_STOP, data, length);
    free(data);
    return WICED_SUCCESS;
}

wiced_result_t wiced_bt_a2dp_sink_change_route( wiced_bt_device_address_t bd_address, uint8_t route )
{
    uint8_t*        data  = NULL;
    uint8_t*        p     = NULL;
    uint16_t        length  = 0;
    int             context_index;

    length = sizeof(uint16_t) + sizeof(uint8_t);
    /* allocate memory for the data */
    data = (uint8_t*)calloc(length, sizeof(uint8_t));
    p = data;
    context_index = get_a2d_context_index_by_addr(bd_address);
    if(context_index == A2DP_CONN_MAX_LIMIT)
    {
        WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
        free(data);
        return WICED_BADARG;
    }
    UINT16_TO_STREAM(p, wiced_bt_hci_a2dp_remote_dev[context_index].handle);
    UINT8_TO_STREAM(p, route);
    wiced_hci_send(HCI_CONTROL_AUDIO_SINK_COMMAND_CHANGE_ROUTE, data, length);
    free(data);
    return WICED_SUCCESS;
}

static int get_a2d_context_index_by_addr(wiced_bt_device_address_t remote_addr)
{
    int i=0;

    while(i < A2DP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_a2dp_remote_dev[i].in_use == WICED_TRUE) &&
            (!wiced_bt_a2dp_sink_utils_bdcmp(wiced_bt_hci_a2dp_remote_dev[i].peer_addr, remote_addr)))
        {
            WICED_HCI_DEBUG_LOG(( "get_a2d_context_index_by_addr Already present i:%d\n",i ));
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_a2d_context_index_by_addr Invalid!!\n" ));
    return A2DP_CONN_MAX_LIMIT;
}

static int get_a2d_context_index_by_handle(uint16_t    handle)
{
    int i=0;

    while(i < A2DP_CONN_MAX_LIMIT)
    {
        if( (wiced_bt_hci_a2dp_remote_dev[i].in_use == WICED_TRUE) &&
            (wiced_bt_hci_a2dp_remote_dev[i].handle == handle))
        {
            WICED_HCI_DEBUG_LOG(( "get_a2d_context_index_by_handle Already present i:%d\n",i ));
            return i;
        }
        i++;
    }
    WICED_HCI_DEBUG_LOG(("get_a2d_context_index_by_handle Invalid!!\n" ));
    return A2DP_CONN_MAX_LIMIT;
}

static int set_a2d_context_index_peer(wiced_bt_device_address_t remote_addr)
{
    int i=0;
    while(i < A2DP_CONN_MAX_LIMIT)
    {
        if(wiced_bt_hci_a2dp_remote_dev[i].in_use == WICED_TRUE)
        {
            if(!wiced_bt_a2dp_sink_utils_bdcmp(wiced_bt_hci_a2dp_remote_dev[i].peer_addr, remote_addr))
             {
                 WICED_HCI_DEBUG_LOG(( "set_context_index Already present i:%d\n",i ));
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
            wiced_bt_hci_a2dp_remote_dev[i].in_use = WICED_TRUE;
            WICED_HCI_DEBUG_LOG( ( "set_context_index New entry i:%d\n",i ));
            return i;
        }
    }
    WICED_HCI_DEBUG_LOG(("set_a2d_context_index_peer Invalid!!\n" ));
    return A2DP_CONN_MAX_LIMIT;
}

static void wiced_hci_audio_cb(uint16_t event, uint8_t* payload, uint32_t len)
{
    uint8_t*                     p = payload;
    int                         context_index;

    //WICED_HCI_DEBUG_LOG(("%s event:%x\n", __func__, event));

    switch(event)
    {
        case HCI_CONTROL_AUDIO_SINK_EVENT_COMMAND_STATUS:
        {
            WICED_HCI_DEBUG_LOG(("%s: command status:%d\n", __FUNCTION__, *p));
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTED:
        {
            wiced_bt_a2dp_sink_status_t     connect;
            wiced_bt_device_address_t       bd_addr_temp;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_CONNECTED\n", __FUNCTION__));
            STREAM_TO_BDADDR(bd_addr_temp, p);
            context_index = set_a2d_context_index_peer(bd_addr_temp);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr, bd_addr_temp);
            STREAM_TO_UINT16(wiced_bt_hci_a2dp_remote_dev[context_index].handle, p);
            wiced_bt_a2dp_sink_utils_bdcpy(connect.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);
            connect.result = WICED_SUCCESS;
            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CONNECT_EVT, (wiced_bt_a2dp_sink_event_data_t*)&connect);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_CONNECTION_FAILED:
        {
            wiced_bt_a2dp_sink_status_t connect;

            WICED_HCI_DEBUG_LOG(("%s: Connection FAILED!!\n", __FUNCTION__));
            connect.result = WICED_BT_ERROR;
            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CONNECT_EVT, (wiced_bt_a2dp_sink_event_data_t*)&connect);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_DISCONNECTED:
        {
            wiced_bt_a2dp_sink_status_t disconnect;
            uint16_t handle;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_DISCONNECTED\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);

            context_index = get_a2d_context_index_by_handle(handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }

            wiced_bt_a2dp_sink_utils_bdcpy(disconnect.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);
            disconnect.result = WICED_SUCCESS;

            wiced_bt_hci_a2dp_remote_dev[context_index].in_use = WICED_FALSE;
            memset(wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr, 0, sizeof(wiced_bt_device_address_t));
            wiced_bt_hci_a2dp_remote_dev[context_index].handle = 0;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_DISCONNECT_EVT,
                (wiced_bt_a2dp_sink_event_data_t*) &disconnect);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_STARTED:
        {
            wiced_bt_a2dp_sink_status_t start;
            uint16_t                    handle;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_STARTED\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);

            context_index = get_a2d_context_index_by_handle(handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(start.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);

            start.result = WICED_SUCCESS;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_CFM_EVT,
                (wiced_bt_a2dp_sink_event_data_t *) &start);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_START_IND:
        {
            wiced_bt_a2dp_sink_status_t  start;
            uint16_t handle;
            uint8_t  label;

            STREAM_TO_UINT16(handle, p);
            STREAM_TO_UINT8(label, p);
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_START_IND handle:%x label:%x\n", __FUNCTION__, handle, label));

            context_index = get_a2d_context_index_by_handle(handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(start.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);
            wiced_bt_hci_a2dp_remote_dev[context_index].av_stream_label = label;
            start.result = WICED_SUCCESS;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_START_IND_EVT,
                (wiced_bt_a2dp_sink_event_data_t *) &start);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_STOPPED:
        {
            wiced_bt_a2dp_sink_status_t suspend;
            uint16_t handle;

            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_STOPPED\n", __FUNCTION__));
            STREAM_TO_UINT16(handle, p);

            context_index = get_a2d_context_index_by_handle(handle);
            if(context_index == A2DP_CONN_MAX_LIMIT)
            {
                WICED_HCI_DEBUG_LOG(("%s: Invalid Entry!!\n", __FUNCTION__));
                break;
            }
            wiced_bt_a2dp_sink_utils_bdcpy(suspend.bd_addr, wiced_bt_hci_a2dp_remote_dev[context_index].peer_addr);
            suspend.result = WICED_SUCCESS;

            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_SUSPEND_EVT,
                (wiced_bt_a2dp_sink_event_data_t *) &suspend);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_CODEC_CONFIGURED:
        {
            WICED_HCI_DEBUG_LOG(("%s HCI_CONTROL_AUDIO_EVENT_CODEC_CONFIGURED\n", __FUNCTION__));
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.codec_id, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.samp_freq, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.ch_mode, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.block_len, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.num_subbands, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.alloc_mthd, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.max_bitpool, p);
            STREAM_TO_UINT8(wiced_bt_hci_a2dp_sink_cb.codec_info.cie.sbc.min_bitpool, p);
            (*wiced_bt_hci_a2dp_sink_cb.control_cb)(WICED_BT_A2DP_SINK_CODEC_CONFIG_EVT,
                (wiced_bt_a2dp_sink_event_data_t*) &wiced_bt_hci_a2dp_sink_cb.codec_info);
        }
        break;
        case HCI_CONTROL_AUDIO_SINK_EVENT_AUDIO_DATA:
        {
            wiced_bt_a2dp_sink_audio_data_t audio_data;
            uint16_t media_info;
            uint8_t media_header_len;
            BT_HDR  *p_pkt;
            uint8_t* p_avdtp_header = p;
            BE_STREAM_TO_UINT16(media_info, p);
            BE_STREAM_TO_UINT16(audio_data.seq_num, p);
            BE_STREAM_TO_UINT32(audio_data.timestamp, p);
            // AVDTP media header length = 12 + (csrc count << 2 )
            media_header_len = 12 + ( ( (media_info & 0x0F00) >> 8 )  << 2 );
            p_pkt = (BT_HDR*)(p_avdtp_header + media_header_len -sizeof(BT_HDR));
            p_pkt->event = 0; // not relevant
            p_pkt->layer_specific = 0; // not relevant
            p_pkt->len = len - media_header_len;
            p_pkt->offset = 0; //offset from where the media payload starts.
            audio_data.p_pkt     = p_pkt;
            audio_data.m_pt      = (media_info & 0x00FF); //Marker and Payload Type
            wiced_bt_hci_a2dp_sink_data_cb(wiced_bt_hci_a2dp_sink_cb.codec_info.codec_id,&audio_data);
        }
        break;
        default:
            WICED_HCI_DEBUG_LOG(("%s EVENT not handled\n", __FUNCTION__));
            break;
    }
}
