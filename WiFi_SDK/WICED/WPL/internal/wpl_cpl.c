/*
 * Copyright 2018, Cypress Semiconductor Corporation or a subsidiary of 
 * Cypress Semiconductor Corporation. All Rights Reserved.
 * 
 * This software, associated documentation and materials ("Software"),
 * is owned by Cypress Semiconductor Corporation
 * or one of its subsidiaries ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products. Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 */

/** @file
 *
 */

/**
 * Define CPL functions
 */
#include "wiced.h"
#include "wpl_cpl.h"
#include "wiced_low_power.h"
#include "wpl_core.h"
#include "wpl_power_events.h"
#include "platform_wpl.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *               Variables Definitions
 ******************************************************/
cpl_data_t cpl_data;
cpl_log_buffer_t  *cpl_log;
static wiced_mutex_t cpl_update_lock;
uint8_t cpl_init_status;
uint8_t cpl_start_log;

/* CPL state machine maintainer */
static cpl_state_ctrl_t *cpl_state;

//aon variables
static uint32_t WPL_DEEP_SLEEP_SAVED_VAR( cpl_deep_sleep_save_data );
uint32_t WPL_DEEP_SLEEP_SAVED_VAR( cpl_last_log_time );


/******************************************************
 *               Function Declarations
 ******************************************************/
wiced_result_t cpl_callback_function( uint8_t cmd_id, uint32_t* len, uint8_t* in_data, uint8_t** out_data );
static uint8_t cpl_cmd_get_events_list( uint8_t proc_id, uint8_t **out_data );
static uint8_t cpl_cmd_get_event_desc_list( uint8_t proc_id, uint8_t event_id, uint8_t **out_data );
static uint8_t cpl_lut_get_proc_data( uint8_t *proc_id );
static uint8_t cpl_lut_get_events_list( uint8_t proc_id, uint8_t *out_data );
static uint8_t cpl_lut_get_event_desc_list( uint8_t proc_id, uint8_t event_id, uint8_t *out_data );
static wiced_result_t cpl_cmd_log_enable( uint8_t* in_data, wiced_bool_t enable );
static void cpl_log_enable_event( uint8_t proc_id, uint8_t event_id, wiced_bool_t enable );
static void cpl_log_enable_all_events( wiced_bool_t enable );
static wiced_result_t cpl_cmd_log_request( uint8_t* in_data, uint32_t* count );
static void cpl_log_buffer_init( void );
static void cpl_state_init( void );
static void cpl_reset_timestamp( uint32_t adjustment );
static void cpl_reset_event_duration( void );
static uint8_t cpl_proc_index_from_lut( uint8_t proc_id );
static uint8_t cpl_event_index_from_lut( uint8_t proc_id, uint8_t event_id );
static void cpl_refresh_log_buffer( void );
#ifdef WPRINT_ENABLE_APP_DEBUG
static void cpl_state_print( void );
#endif
static uint8_t  cpl_search_for_duplicate( uint8_t *data, uint32_t data_len, uint8_t event_id );
static void cpl_deep_sleep_log_state_store( uint8_t pindex, uint8_t eindex );
static wiced_bool_t cpl_deep_sleep_log_state_restore( uint8_t pindex, uint8_t eindex );
static void cpl_restore_log_states( void );
static void cpl_print_log_buffer( uint32_t line );
static void cpl_memory_init( void );
static uint8_t cpl_lut_get_max_proc_id( void );
static uint8_t cpl_lut_get_max_events_list( uint8_t proc_id );
static uint8_t cpl_lut_get_max_event_desc_list( uint8_t proc_id, uint8_t event_id );

/******************************************************
 *               Function Definitions
 ******************************************************/

void wpl_cpl_init( void )
{
    wiced_rtos_init_mutex( &cpl_update_lock );

    cpl_memory_init( );

    cpl_state_init( );

    cpl_log_buffer_init( );

    cpl_data.proc_count     =  cpl_state->lut_proc_id_cnt;
    cpl_data.proc_id        = cpl_state->proc_list;
    cpl_data.cpl_callback     = cpl_callback_function;
    cpl_data.log_size         = cpl_log->log_count;

#ifdef WPRINT_ENABLE_APP_DEBUG
    cpl_state_print( );
#endif

    wpl_register_cpl( &cpl_data );

    if ( WICED_DEEP_SLEEP_IS_WARMBOOT( ) )
    {
        cpl_reset_timestamp( cpl_last_log_time );
        cpl_restore_log_states( );
        wpl_set_last_power_state_before_warm_boot( );
        /* If logging was going on before deep sleep, enable platform for logging in warmboot */
        if(wpl_logging_status())
            platform_wpl_enable_logging(WICED_TRUE);
    }

    cpl_init_status = 1;

    wpl_set_first_power_state( );

    cpl_start_log = 1;
}

static void cpl_memory_init( void )
{
    cpl_state = ( cpl_state_ctrl_t * )malloc( sizeof( cpl_state_ctrl_t ) );
    if ( cpl_state == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return;
    }
}

static void cpl_reset_timestamp( uint32_t adjustment )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries =  cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
            eentries = cpl_state->proc_data[pindex].max_event_id;
            for ( eindex = 0; eindex < eentries; eindex++ )
            {
                cpl_state->proc_data[pindex].event_data[eindex].previous_time_stamp = adjustment;
            }
    }
}

static void cpl_reset_event_duration( void )
{
    uint8_t table_index;
    uint8_t table_entries = ( uint8_t ) cpl_log->log_count;

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        cpl_log->cpl_log_packet[table_index].event_duration = 0;
    }
}

static void cpl_state_init( void )
{
    uint8_t pindex, eindex, dindex;
    uint8_t pentries, eentries, dentries;

    //Fill the cpl_state structure
    cpl_state->max_proc_id = EVENT_PROC_ID_MAX;
    cpl_state->lut_proc_id_cnt = cpl_lut_get_max_proc_id( );
    cpl_state->proc_data = ( cpl_proc_id_struct_t * )malloc( cpl_state->max_proc_id * sizeof( cpl_proc_id_struct_t ) );
    if ( cpl_state->proc_data == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return;
    }

    cpl_state->proc_list = ( uint8_t * )malloc( cpl_state->lut_proc_id_cnt );
    if ( cpl_state->proc_list == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return;
    }

    cpl_lut_get_proc_data( cpl_state->proc_list );

    WPRINT_APP_DEBUG( ( "cpl_state->lut_proc_id_cnt:%u\n", cpl_state->lut_proc_id_cnt ) );
    pentries = cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {

        cpl_state->proc_data[pindex].proc_id = pindex;
        WPRINT_APP_DEBUG( ( "pindex, cpl_state->proc_data[%u].proc_id:0x%u\n", pindex, cpl_state->proc_data[pindex].proc_id ) );


        //Get Events list for each processor id
        cpl_state->proc_data[pindex].max_event_id = EVENT_ID_MAX;
        cpl_state->proc_data[pindex].lut_event_id_cnt = cpl_lut_get_max_events_list( cpl_state->proc_data[pindex].proc_id );
        cpl_state->proc_data[pindex].event_data = ( cpl_event_id_struct_t * )malloc( cpl_state->proc_data[pindex].max_event_id * sizeof( cpl_event_id_struct_t ) );
        if ( cpl_state->proc_data[pindex].event_data == NULL )
        {
            WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
            return;
        }

        cpl_state->proc_data[pindex].event_list = ( uint8_t * )malloc( cpl_state->proc_data[pindex].lut_event_id_cnt );
        if ( cpl_state->proc_data[pindex].event_list == NULL )
        {
            WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
            return;
        }

        cpl_lut_get_events_list( cpl_state->proc_data[pindex].proc_id, cpl_state->proc_data[pindex].event_list );

        WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].lut_event_id_cnt:%u\n", pindex, cpl_state->proc_data[pindex].lut_event_id_cnt ) );
        eentries = cpl_state->proc_data[pindex].max_event_id;

        for ( eindex = 0; eindex < eentries; eindex++ )
        {
            cpl_state->proc_data[pindex].event_data[eindex].event_id = eindex;
            WPRINT_APP_DEBUG( ( "pindex, cpl_state->proc_data[%u].event_data[%u].event_id:0x%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].event_id ) );


            //Get Event descriptor list for each processor id
            cpl_state->proc_data[pindex].event_data[eindex].lut_event_desc_cnt = cpl_lut_get_max_event_desc_list( cpl_state->proc_data[pindex].proc_id, cpl_state->proc_data[pindex].event_data[eindex].event_id );
            cpl_state->proc_data[pindex].event_data[eindex].max_event_desc = cpl_state->proc_data[pindex].event_data[eindex].lut_event_desc_cnt;
            cpl_state->proc_data[pindex].event_data[eindex].event_desc_list = ( uint8_t * )malloc( cpl_state->proc_data[pindex].event_data[eindex].max_event_desc );
            if ( cpl_state->proc_data[pindex].event_data[eindex].event_desc_list == NULL )
            {
                WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
                return;
            }

            cpl_lut_get_event_desc_list( cpl_state->proc_data[pindex].proc_id, cpl_state->proc_data[pindex].event_data[eindex].event_id, cpl_state->proc_data[pindex].event_data[eindex].event_desc_list );
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].lut_event_desc_cnt:%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].lut_event_desc_cnt ) );
            dentries = cpl_state->proc_data[pindex].event_data[eindex].max_event_desc;

            cpl_state->proc_data[pindex].event_data[eindex].previous_time_stamp = 0;
            cpl_state->proc_data[pindex].event_data[eindex].current_event_desc = 0;
            cpl_state->proc_data[pindex].event_data[eindex].log_enabled = 0;

            for ( dindex = 0; dindex < dentries; dindex++ )
                WPRINT_APP_DEBUG( ( "pindex, pindex, eindex, cpl_state->proc_data[%u].event_data[%u].event_desc_list[%u]:0x%u\n", pindex, eindex, dindex, cpl_state->proc_data[pindex].event_data[eindex].event_desc_list[dindex] ) );
        }
    }
}
#ifdef WPRINT_ENABLE_APP_DEBUG
static void cpl_state_print( void )
{
    uint8_t pindex, eindex, dindex;
    uint8_t pentries, eentries, dentries;


    WPRINT_APP_DEBUG( ( "cpl_state->lut_proc_id_cnt:%u\n", cpl_state->lut_proc_id_cnt ) );
    pentries = cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        WPRINT_APP_DEBUG( ( "cpl_state->proc_list[%u]:0x%u\n", pindex, cpl_state->proc_list[pindex] ) );

        WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].proc_id:%u\n", pindex, cpl_state->proc_data[pindex].proc_id ) );
        WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].lut_event_id_cnt:%u\n", pindex, cpl_state->proc_data[pindex].lut_event_id_cnt ) );
        eentries = cpl_state->proc_data[pindex].max_event_id;

        for ( eindex = 0; eindex < eentries; eindex++ )
        {
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_list[%u]:0x%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_list[eindex] ) );
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].event_id:%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].event_id ) );
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].lut_event_desc_cnt:%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].lut_event_desc_cnt ) );
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].log_enabled:%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].log_enabled ) );
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].current_event_desc:%u\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].current_event_desc ) );
            WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].previous_time_stamp:%lu\n", pindex, eindex, cpl_state->proc_data[pindex].event_data[eindex].previous_time_stamp ) );
            dentries = cpl_state->proc_data[pindex].event_data[eindex].max_event_desc;

            for ( dindex = 0; dindex < dentries; dindex++ )
                WPRINT_APP_DEBUG( ( "cpl_state->proc_data[%u].event_data[%u].event_desc_list[%u]:0x%u\n", pindex, eindex, dindex, cpl_state->proc_data[pindex].event_data[eindex].event_desc_list[dindex] ) );
        }
    }
}
#endif
static void cpl_log_buffer_init( void )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );

    cpl_log = ( cpl_log_buffer_t * ) malloc ( sizeof( cpl_log_buffer_t ) );
    if ( cpl_log == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return;
    }

    cpl_log->log_count = table_entries;

    cpl_log->cpl_log_packet = ( cpl_packet_t * ) malloc( cpl_log->log_count * sizeof( cpl_packet_t ) );
    if ( cpl_log->cpl_log_packet == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return;
    }

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        cpl_log->cpl_log_packet[table_index].proc_id = wpl_event_list_table[table_index].proc_id;
        cpl_log->cpl_log_packet[table_index].event_id = wpl_event_list_table[table_index].event_id;
        cpl_log->cpl_log_packet[table_index].event_desc = wpl_event_list_table[table_index].event_desc;
        cpl_log->cpl_log_packet[table_index].event_duration = 0;
    }
    cpl_print_log_buffer( __LINE__ );
}

static uint8_t  cpl_search_for_duplicate( uint8_t *data, uint32_t data_len, uint8_t eventid )
{
    uint8_t index;
    for ( index = 0; index < data_len; index++ )
    {
        if ( data[index] == eventid )
            return 1;
    }
    return 0;
}

static uint8_t cpl_lut_get_max_proc_id( void )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );
    uint8_t *proc_id;
    uint8_t count = 0;

    proc_id = ( uint8_t * ) malloc( table_entries );
    if ( proc_id == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return 0;
    }
    *proc_id = 0;
    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        if ( !cpl_search_for_duplicate( proc_id, count, wpl_event_list_table[table_index].proc_id ) )
        {
            proc_id[count] = wpl_event_list_table[table_index].proc_id;
            count++;
        }
    }
    free( proc_id );
    return count;
}

static uint8_t cpl_lut_get_proc_data( uint8_t *proc_id )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );
    uint8_t count = 0;

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        if ( !cpl_search_for_duplicate( proc_id, count, wpl_event_list_table[table_index].proc_id ) )
        {
            proc_id[count] = wpl_event_list_table[table_index].proc_id;
            count++;
        }
    }
    return count;
}

static uint8_t cpl_proc_index_from_lut( uint8_t proc_id )
{
    uint8_t pindex;
    uint8_t pentries;

    pentries = cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        if ( proc_id == cpl_state->proc_data[pindex].proc_id )
        {
            return pindex;
        }
    }
    return 0xFF;
}

static uint8_t cpl_lut_get_max_events_list( uint8_t proc_id )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );
    uint8_t *event_id;
    uint8_t count = 0;

    event_id = ( uint8_t * ) malloc( table_entries );
    if ( event_id == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return 0;
    }
    *event_id = 0;
    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        if ( wpl_event_list_table[table_index].proc_id == proc_id )
        {
            if ( !cpl_search_for_duplicate( event_id, count, wpl_event_list_table[table_index].event_id ) )
            {
                event_id[count] = wpl_event_list_table[table_index].event_id;
                count++;
            }
        }
    }
    free( event_id );
    return count;
}

static uint8_t cpl_lut_get_events_list( uint8_t proc_id, uint8_t *out_data )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );
    uint8_t count = 0;

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        if ( wpl_event_list_table[table_index].proc_id == proc_id )
        {
            if ( !cpl_search_for_duplicate( out_data, count, wpl_event_list_table[table_index].event_id ) )
            {
                out_data[count] = wpl_event_list_table[table_index].event_id;
                count++;
            }
        }
    }
    return count;
}

static uint8_t cpl_cmd_get_events_list( uint8_t proc_id, uint8_t **out_data )
{
    uint8_t pindex = cpl_proc_index_from_lut( proc_id );

    *out_data = cpl_state->proc_data[pindex].event_list;

    return cpl_state->proc_data[pindex].lut_event_id_cnt;

}

static uint8_t cpl_event_index_from_lut( uint8_t proc_id, uint8_t event_id )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries =  cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        if ( proc_id == cpl_state->proc_data[pindex].proc_id )
        {
             eentries = cpl_state->proc_data[pindex].max_event_id;
             for ( eindex = 0; eindex < eentries; eindex++ )
             {
                 if ( event_id == cpl_state->proc_data[pindex].event_data[eindex].event_id )
                        return eindex;
             }
        }
    }
    return 0xFF;
}

static uint8_t cpl_lut_get_max_event_desc_list( uint8_t proc_id, uint8_t event_id )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );
    uint8_t *event_dec_list;
    uint8_t count = 0;

    event_dec_list = ( uint8_t * ) malloc( table_entries );
    if ( event_dec_list == NULL )
    {
        WPRINT_APP_DEBUG( ( "%s: malloc failed to allocate at %d\n", __func__, __LINE__ ) );
        return 0;
    }

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        if ( wpl_event_list_table[table_index].proc_id == proc_id && wpl_event_list_table[table_index].event_id == event_id )
        {
            event_dec_list[count] = wpl_event_list_table[table_index].event_desc;
            count++;
        }
    }
    free( event_dec_list );
    return count;
}

static uint8_t cpl_lut_get_event_desc_list( uint8_t proc_id, uint8_t event_id, uint8_t *out_data )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );
    uint8_t count = 0;

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {
        if ( wpl_event_list_table[table_index].proc_id == proc_id && wpl_event_list_table[table_index].event_id == event_id )
        {
            out_data[count] = wpl_event_list_table[table_index].event_desc;
            count++;
        }
    }
    return count;
}

static uint8_t cpl_cmd_get_event_desc_list( uint8_t proc_id, uint8_t event_id, uint8_t **out_data )
{
    uint8_t pindex, eindex;

    pindex = cpl_proc_index_from_lut( proc_id );
    eindex = cpl_event_index_from_lut( proc_id, event_id );

    *out_data = cpl_state->proc_data[pindex].event_data[eindex].event_desc_list;

    return cpl_state->proc_data[pindex].event_data[eindex].lut_event_desc_cnt;
}

static void cpl_log_enable_all_events( wiced_bool_t enable )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries = cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
            eentries = cpl_state->proc_data[pindex].max_event_id;
            for ( eindex = 0; eindex < eentries; eindex++ )
            {
                cpl_state->proc_data[pindex].event_data[eindex].log_enabled = enable;
                cpl_deep_sleep_log_state_store( pindex, eindex );
            }

    }
}

static void cpl_log_enable_event( uint8_t proc_id, uint8_t event_id, wiced_bool_t enable )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries =  cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        if ( proc_id == cpl_state->proc_data[pindex].proc_id )
        {
             eentries = cpl_state->proc_data[pindex].max_event_id;
             for ( eindex = 0; eindex < eentries; eindex++ )
             {
                 if ( event_id == cpl_state->proc_data[pindex].event_data[eindex].event_id )
                 {
                     cpl_state->proc_data[pindex].event_data[eindex].log_enabled = enable;
                     cpl_deep_sleep_log_state_store( pindex, eindex );
                 }
             }
        }
    }
}

static wiced_result_t cpl_cmd_log_enable( uint8_t* in_data, wiced_bool_t enable )
{
    cpl_reset_timestamp( wpl_get_time_stamp( ) );

    platform_wpl_enable_logging( enable );

    if ( in_data == NULL )
        cpl_log_enable_all_events( enable );
    else
        cpl_log_enable_event( in_data[0], in_data[1], enable );
    return WICED_SUCCESS;
}

static wiced_result_t cpl_cmd_log_request( uint8_t* in_data, uint32_t* count )
{
    if( !cpl_start_log ) {
        *count = 0;
        return WICED_ERROR;
    }

    cpl_refresh_log_buffer( );

#ifdef WPL_ENABLE_WIFI_LOG
    wpl_wifi_get_power_data( );
#endif

#ifdef WPL_ENABLE_BT_LOG
    platform_wpl_update_bt_power_data();
#endif

    cpl_print_log_buffer( __LINE__ );

    memcpy( in_data, cpl_log->cpl_log_packet, cpl_log->log_count * sizeof( cpl_packet_t ) );

    cpl_reset_event_duration( );

    /* Let WPL handler know whether you have sent some logs */
    *count = cpl_log->log_count;

    return WICED_SUCCESS;
}

wiced_result_t cpl_callback_function( uint8_t cmd_id, uint32_t* len, uint8_t* in_data, uint8_t** out_data )
{
    wiced_result_t result = WICED_ERROR;
    uint8_t *cmd_data = ( uint8_t * ) in_data;

    switch( cmd_id )
    {
        case CPL_GET_EVENTS_LIST:
            *len = cpl_cmd_get_events_list( cmd_data[0], out_data );
            if( *len )
                result = WICED_SUCCESS;
            else
                result = WICED_ERROR;
            break;
        case CPL_GET_EVENTS_DESC_LIST:
           *len = cpl_cmd_get_event_desc_list( cmd_data[0], cmd_data[1], out_data );

            if( *len )
                result = WICED_SUCCESS;
            else
                result = WICED_ERROR;
            break;
        case CPL_START_LOG:
            result = cpl_cmd_log_enable( in_data, WICED_TRUE );
            break;
        case CPL_STOP_LOG:
            result = cpl_cmd_log_enable( in_data, WICED_FALSE );
            break;
        case CPL_SEND_LOG_REQUEST:
            result = cpl_cmd_log_request( in_data, len );
            break;

        default:
            result = WICED_ERROR;
            break;
    }

    return result;
}

void cpl_set_powerstate( uint8_t proc_id, uint8_t event_id, uint8_t event_state )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries = cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        if ( proc_id == cpl_state->proc_data[pindex].proc_id )
        {
            eentries = cpl_state->proc_data[pindex].max_event_id;
            for ( eindex = 0; eindex < eentries; eindex++ )
            {

                if ( event_id == cpl_state->proc_data[pindex].event_data[eindex].event_id )
                {
                    cpl_state->proc_data[pindex].event_data[eindex].current_event_desc = event_state;
                    break;
                 }
            }
            break;
        }
    }
}

void cpl_log_update( uint8_t proc_id, uint8_t event_id, uint8_t event_state, uint32_t event_data )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {

        if ( ( wpl_event_list_table[table_index].proc_id == proc_id ) &&
                ( wpl_event_list_table[table_index].event_id == event_id ) &&
                    ( wpl_event_list_table[table_index].event_desc == event_state ) )
        {
            wiced_rtos_lock_mutex( &cpl_update_lock );
            cpl_log->cpl_log_packet[table_index].event_duration    += event_data;
            wiced_rtos_unlock_mutex( &cpl_update_lock );
        }
    }
}

void cpl_log_reset_event( uint8_t proc_id, uint8_t event_id, uint8_t event_state )
{
    uint8_t table_index;
    uint8_t table_entries = sizeof( wpl_event_list_table )/sizeof( event_lookup_table_entry_t );

    for ( table_index = 0; table_index < table_entries; table_index++ )
    {

        if ( ( wpl_event_list_table[table_index].proc_id == proc_id ) &&
                ( wpl_event_list_table[table_index].event_id == event_id ) &&
                    ( wpl_event_list_table[table_index].event_desc == event_state ) )
        {
            wiced_rtos_lock_mutex( &cpl_update_lock );
            cpl_log->cpl_log_packet[table_index].event_duration    = 0;
            wiced_rtos_unlock_mutex( &cpl_update_lock );
        }
    }
}

static void cpl_refresh_log_buffer( void )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries = cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        eentries = cpl_state->proc_data[pindex].max_event_id;
        for ( eindex = 0; eindex < eentries; eindex++ )
        {
            cpl_last_log_time = wpl_get_time_stamp( );

            if( cpl_state->proc_data[pindex].event_data[eindex].log_enabled )
                cpl_log_update( cpl_state->proc_data[pindex].proc_id, cpl_state->proc_data[pindex].event_data[eindex].event_id, cpl_state->proc_data[pindex].event_data[eindex].current_event_desc, cpl_last_log_time - cpl_state->proc_data[pindex].event_data[eindex].previous_time_stamp );
            cpl_state->proc_data[pindex].event_data[eindex].previous_time_stamp = cpl_last_log_time;
         }
    }
}

void cpl_event_state_update( uint8_t proc_id, uint8_t event_id, uint8_t event_state )
{
    if( cpl_init_status )
    {
        uint32_t time_stamp;
        time_stamp = wpl_get_time_stamp( );

        if ( cpl_state->proc_data[proc_id].event_data[event_id].log_enabled )
            cpl_log_update( proc_id, event_id, cpl_state->proc_data[proc_id].event_data[event_id].current_event_desc, time_stamp - cpl_state->proc_data[proc_id].event_data[event_id].previous_time_stamp );

        cpl_state->proc_data[proc_id].event_data[event_id].current_event_desc = event_state;

        cpl_state->proc_data[proc_id].event_data[event_id].previous_time_stamp = time_stamp;
    }
}

static void cpl_print_log_buffer( uint32_t line )
{
    uint32_t pkt_cnt;
    UNUSED_PARAMETER( line );
    WPRINT_APP_DEBUG( ( "Line: %lu : Total Packets in Buffer : %lu\n", line, cpl_log->log_count ) );
    for( pkt_cnt = 0; pkt_cnt < cpl_log->log_count;pkt_cnt++ )
        WPRINT_APP_DEBUG( ( "0x%u-0x%u-0x%u-0x%lx\n", cpl_log->cpl_log_packet[pkt_cnt].proc_id, cpl_log->cpl_log_packet[pkt_cnt].event_id, cpl_log->cpl_log_packet[pkt_cnt].event_desc, cpl_log->cpl_log_packet[pkt_cnt].event_duration ) );

    WPRINT_APP_DEBUG( ( "\n" ) );

}

static void cpl_restore_log_states( void )
{
    uint8_t pindex, eindex;
    uint8_t pentries, eentries;

    pentries =  cpl_state->max_proc_id;

    for ( pindex = 0; pindex < pentries; pindex++ )
    {
        eentries = cpl_state->proc_data[pindex].max_event_id;
        for ( eindex = 0; eindex < eentries; eindex++ )
            cpl_state->proc_data[pindex].event_data[eindex].log_enabled = cpl_deep_sleep_log_state_restore( pindex, eindex );
    }
}

static void cpl_deep_sleep_log_state_store( uint8_t pindex, uint8_t eindex )
{
    cpl_deep_sleep_save_data ^= ( 1UL << ( eindex  + pindex * 4 ) );
}

static wiced_bool_t cpl_deep_sleep_log_state_restore( uint8_t pindex, uint8_t eindex )
{
    if ( cpl_deep_sleep_save_data & ( 1UL << ( eindex  + ( pindex * 4 ) ) ) )
        return WICED_TRUE;
    else
        return WICED_FALSE;
}

