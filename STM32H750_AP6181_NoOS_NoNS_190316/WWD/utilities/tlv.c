/*
 * Broadcom Proprietary and Confidential. Copyright 2016 Broadcom
 * All Rights Reserved.
 *
 * This is UNPUBLISHED PROPRIETARY SOURCE CODE of Broadcom Corporation;
 * the contents of this file may not be disclosed to third parties, copied
 * or duplicated in any form, in whole or in part, without the prior
 * written permission of Broadcom Corporation.
 */
#include <string.h>
#include "tlv.h"
#include "wiced_utilities.h"

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static uint32_t tlv_hton32_ptr(uint8_t* in, uint8_t* out);
static uint16_t tlv_hton16_ptr(uint8_t* in, uint8_t* out);

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Definitions
 ******************************************************/

tlv8_data_t* tlv_find_tlv8( const uint8_t* message, uint32_t message_length, uint8_t type )
{
    while ( message_length != 0 )
    {
        uint8_t  current_tlv_type   = message[ 0 ];
        uint16_t current_tlv_length = message[ 1 ] + 2;

        /* Check if we've overrun the buffer */
        if ( current_tlv_length > message_length )
        {
            return NULL;
        }

        /* Check if we've found the type we are looking for */
        if ( current_tlv_type == type )
        {
            return (tlv8_data_t*) message;
        }

        /* Skip current TLV */
        message        += current_tlv_length;
        message_length -= current_tlv_length;
    }
    return 0;
}

tlv16_data_t* tlv_find_tlv16( const uint8_t* message, uint32_t message_length, uint16_t type )
{
    while ( message_length != 0 )
    {
        tlv16_data_t* tlv                = (tlv16_data_t*) message;
        uint16_t      current_tlv_type   = htobe16( WICED_READ_16( &tlv->type ) );
        uint16_t      current_tlv_length = htobe16( WICED_READ_16( &tlv->length ) ) + 4;

        /* Check if we've overrun the buffer */
        if ( current_tlv_length > message_length )
        {
            return NULL;
        }

        /* Check if we've found the type we are looking for */
        if ( current_tlv_type == type )
        {
            return tlv;
        }

        /* Skip current TLV */
        message        += current_tlv_length;
        message_length -= current_tlv_length;
    }

    return NULL;
}

tlv_result_t tlv_read_value ( uint16_t type, const uint8_t* message, uint16_t message_length, void* value, uint16_t value_size, tlv_data_type_t data_type )
{
    tlv16_data_t* tlv = tlv_find_tlv16( message, message_length, type );
    if (tlv == NULL)
    {
        return TLV_NOT_FOUND;
    }

    switch (data_type)
    {
        case TLV_UINT16:
            tlv_hton16_ptr((uint8_t*) tlv->data, value );
            break;

        case TLV_UINT32:
            tlv_hton32_ptr((uint8_t*) tlv->data, value );
            break;

        default:
            memcpy( value, tlv->data, value_size );
            break;
    }

    return TLV_SUCCESS;
}

uint8_t* tlv_write_value( uint8_t* buffer, uint16_t type, uint16_t length, const void* data, tlv_data_type_t data_type )
{
    tlv16_data_t* tlv = (tlv16_data_t*) buffer;
    WICED_WRITE_16( &tlv->type,   htobe16(type)   );
    WICED_WRITE_16( &tlv->length, htobe16(length) );
    switch (data_type)
    {
        case TLV_UINT16:
            tlv_hton16_ptr((uint8_t*) data, tlv->data);
            break;

        case TLV_UINT32:
            tlv_hton32_ptr((uint8_t*) data, tlv->data);
            break;

        default:
            memcpy(tlv->data, data, length);
            break;
    }
    return buffer + sizeof(tlv16_header_t) + length;
}

uint8_t* tlv_write_header( uint8_t* buffer, uint16_t type, uint16_t length )
{
    tlv16_header_t* tlv = (tlv16_header_t*)buffer;
    WICED_WRITE_16( &tlv->type,   htobe16(type)   );
    WICED_WRITE_16( &tlv->length, htobe16(length) );
    return buffer + sizeof(tlv16_header_t);
}



static uint32_t tlv_hton32_ptr(uint8_t * in, uint8_t * out)
{
    uint32_t temp;
    temp = WICED_READ_32(in);
    temp = htobe32(temp);
    WICED_WRITE_32(out, temp);
    return temp;
}

static uint16_t tlv_hton16_ptr(uint8_t * in, uint8_t * out)
{
    uint16_t temp;
    temp = WICED_READ_16(in);
    temp = htobe16(temp);
    WICED_WRITE_16(out, temp);
    return temp;
}
