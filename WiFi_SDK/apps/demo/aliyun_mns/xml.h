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
#pragma once

#ifndef  __XML_H__
#define  __XML_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced.h"

#define XML_VERSION_1_0 "1.0"
#define XML_ENCODING_UTF_8 "UTF-8"
#define XML_ENCODING_UTF_16 "UTF-16"
#define XML_STANDALONE_YES "yes"
#define XML_STANDALONE_NO "no"

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

typedef struct
{
    char *buffer;
    uint32_t buffer_size;
    uint32_t bytes_used;
}wiced_xml_t;

typedef enum
{
    XML_START_TAG,
    XML_END_TAG,
    XML_EMPTY_ELEMENT_TAG,
}wiced_xml_tag_t;
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

wiced_result_t wiced_xml_init( wiced_xml_t *xml, char *xmlbuffer, uint32_t xmlbuffersize );
wiced_result_t wiced_xml_reset( wiced_xml_t *xml );
wiced_result_t wiced_xml_declaration( wiced_xml_t *xml, char *version, char *encoding, char *standalone );
wiced_result_t xml_write_tag( wiced_xml_t *xml, char *tag, wiced_xml_tag_t tag_type, char *attribute );
wiced_result_t xml_write_text( wiced_xml_t *xml, char *text );
wiced_result_t wiced_xml_start_element( wiced_xml_t *xml, char* tag_name );
wiced_result_t wiced_xml_end_element( wiced_xml_t *xml, char* tag_name );
wiced_result_t wiced_xml_start_attribute( wiced_xml_t *xml, char *tag_name, char *attribute, wiced_xml_tag_t tag_type );
wiced_result_t wiced_xml_end_attribute( wiced_xml_t *xml, char *tag_name );
wiced_result_t wiced_xml_create_single_element( wiced_xml_t *xml, char* tag_name, char* text );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  /* __XML_H__    */
