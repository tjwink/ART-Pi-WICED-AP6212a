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

#include "xml.h"
#include "wiced.h"


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
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Variable Definitions
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

wiced_result_t wiced_xml_init( wiced_xml_t *xml, char *xmlbuffer, uint32_t xmlbuffersize )
{
    if( xmlbuffer != NULL )
    {
        xml->buffer = xmlbuffer;
        xml->buffer_size = xmlbuffersize;
        xml->bytes_used = 0;
        memset( xml->buffer, 0, xmlbuffersize );
        WPRINT_APP_DEBUG(("XML init success\n"));
        return WICED_SUCCESS;
    }
    else
        WPRINT_APP_DEBUG(("XML init fail\n"));
    return WICED_ERROR;
}

wiced_result_t wiced_xml_reset( wiced_xml_t *xml )
{
    if(xml->buffer!=NULL && xml->buffer_size!=0)
    {
        xml->bytes_used = 0;
        memset( xml->buffer, 0, xml->buffer_size );
        return WICED_SUCCESS;
    }
    else
        return WICED_ERROR;
}

wiced_result_t wiced_xml_declaration( wiced_xml_t *xml, char *version, char *encoding, char *standalone )
{
    // If declaration is included, it must contain the version number
    int result = 0;
    if( encoding != NULL && standalone != NULL )
    {
        result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<?xml version=\"%s\" encoding=\"%s\" standalone=\"%s\" ?>", version, encoding, standalone);
    }
    else if (encoding != NULL )
    {
        result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<?xml version=\"%s\" encoding=\"%s\" ?>", version, encoding);
    }
    else if (standalone != NULL )
    {
        result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<?xml version=\"%s\" standalone=\"%s\" ?>", version, standalone);
    }
    else
    {
        result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<?xml version=\"%s\" ?>", version);
    }

    if ( result != -1 ){
        xml->bytes_used += result;
        WPRINT_APP_DEBUG(("%s success. Bytes written: %d\n", __FUNCTION__, result));
        WPRINT_APP_DEBUG(("Total bytes used: %d\n", (int)xml->bytes_used));
        return WICED_SUCCESS;
    }
    else
    {
        WPRINT_APP_DEBUG(("%s failed.\n", __FUNCTION__));
        return WICED_ERROR;
    }
}

wiced_result_t xml_write_tag( wiced_xml_t *xml, char *tag, wiced_xml_tag_t tag_type, char *attribute )
{
    int result = 0;
    switch(tag_type)
    {
    case XML_START_TAG:
        if(attribute == NULL)
        {
            result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<%s>", tag);
        }
        else
        {
            result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<%s %s>", tag, attribute);
        }
        break;
    case XML_END_TAG:
        result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "</%s>", tag);
        break;
    case XML_EMPTY_ELEMENT_TAG:
        if(attribute == NULL)
        {
            result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<%s />", tag);
        }
        else
        {
            result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "<%s %s />", tag, attribute);
        }
        break;
    default:
        break;
    }

    if ( result != -1 ){
        xml->bytes_used += result;
        WPRINT_APP_DEBUG(("%s success. Bytes written: %d\n", __FUNCTION__, result));
        WPRINT_APP_DEBUG(("Total bytes used: %d\n", (int)xml->bytes_used));
        return WICED_SUCCESS;
    }
    else
    {
        WPRINT_APP_INFO(("%s failed.\n", __FUNCTION__));
        return WICED_ERROR;
    }
}

wiced_result_t xml_write_text( wiced_xml_t *xml, char *text )
{
    int result;
    result = snprintf(xml->buffer+xml->bytes_used, xml->buffer_size-xml->bytes_used, "%s", text);

    if ( result != -1 ){
        xml->bytes_used += result;
        WPRINT_APP_DEBUG(("%s success. Bytes written: %d\n", __FUNCTION__, result));
        WPRINT_APP_DEBUG(("Total bytes used: %d\n", (int)xml->bytes_used));
        return WICED_SUCCESS;
    }
    else
    {
        WPRINT_APP_INFO(("%s failed.\n", __FUNCTION__));
        return WICED_ERROR;
    }
}

wiced_result_t wiced_xml_start_element( wiced_xml_t *xml, char* tag_name )
{
    return xml_write_tag( xml, tag_name, XML_START_TAG, NULL);
}

wiced_result_t wiced_xml_end_element( wiced_xml_t *xml, char* tag_name )
{
    return xml_write_tag( xml, tag_name, XML_END_TAG, NULL);
}

wiced_result_t wiced_xml_start_attribute( wiced_xml_t *xml, char *tag_name, char *attribute, wiced_xml_tag_t tag_type )
{
    if( tag_type == XML_END_TAG )
        return WICED_ERROR;
    return xml_write_tag( xml, tag_name, tag_type, attribute );
}

wiced_result_t wiced_xml_end_attribute( wiced_xml_t *xml, char *tag_name )
{
    return xml_write_tag( xml, tag_name, XML_END_TAG, NULL );
}

wiced_result_t wiced_xml_create_single_element( wiced_xml_t *xml, char* tag_name, char* text )
{
    // simple single text field element
    WICED_VERIFY( wiced_xml_start_element( xml, tag_name ) );
    WICED_VERIFY( xml_write_text( xml, text ) );
    WICED_VERIFY( wiced_xml_end_element( xml, tag_name ) );
    return WICED_SUCCESS;
}

