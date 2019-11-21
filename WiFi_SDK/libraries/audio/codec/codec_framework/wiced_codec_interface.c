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
/*****************************************************************************
**
**  Name:           wiced_codec_if.c
**
**  Description:    This is the public interface file for integrating
**                  third party codecs.
**
*****************************************************************************/
#include "wiced_codec_if.h"
#include <stddef.h>
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


static uint16_t current_codec_index = 0;
extern wiced_codec_interface_t codec_sbc;

#ifdef ENABLE_MPEG24_AAC_CODEC
extern wiced_codec_interface_t codec_m24; // or use any variable name that you prefer.
                                          // use the same name in the default codec table below.
#endif

wiced_codec_handle_t wiced_default_codec_table[] = {
        &codec_sbc, // should ideally be first element since default. If this changes,
                    // change the current_codec_index also to match.
#ifdef ENABLE_MPEG24_AAC_CODEC
        &codec_m24
#endif
#ifdef ENABLE_FLAC_CODEC
        &codec_flac
#endif
};

wiced_codec_handle_t* wiced_codec_table = NULL;

/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *               Static Function Declarations
 ******************************************************/

void wiced_codec_interface_init(wiced_codec_handle_t* codec_table)
{
    if(codec_table == NULL)
    {
        wiced_codec_table = wiced_default_codec_table;
        return;
    }

    /* if the codec_table is non-null,
     * then do not use the default table, use the provided table.
     */
    wiced_codec_table = codec_table;
}


wiced_codec_status_t wiced_get_supported_codecs(uint8_t* codec_type_array, int16_t* size)
{
    int count;

    if( (codec_type_array == NULL) || (size == NULL))
    {
        return -1;
    }

    if(wiced_codec_table == NULL)
        wiced_codec_table = wiced_default_codec_table;

    *size = (sizeof wiced_codec_table)/sizeof(wiced_codec_interface_t*);

    for ( count = 0; count < *size ; count++)
    {
        codec_type_array[count] = wiced_codec_table[count]->type;
    }
    return 1;
}

void wiced_register_selected_codec(wiced_codec_type_t codec)
{
    uint16_t i = 0;
    uint16_t size;

    if(wiced_codec_table == NULL)
        wiced_codec_table = wiced_default_codec_table;

    size = (sizeof wiced_codec_table)/sizeof(wiced_codec_interface_t*);

    for(i = 0; i < size; i++ )
    {
        if(wiced_codec_table[i]->type == codec)
        {
            current_codec_index = i;
            return;
        }
    }

    current_codec_index = 0xffff;
}

wiced_codec_interface_t* wiced_get_registered_codec(void)
{
    uint16_t size;

    if(wiced_codec_table == NULL)
        wiced_codec_table = wiced_default_codec_table;

    size = (sizeof wiced_codec_table)/sizeof(wiced_codec_interface_t*);

    if(current_codec_index < size)
        return wiced_codec_table[current_codec_index];
    else
        return NULL;
}

wiced_codec_interface_t* wiced_get_codec_by_type(wiced_codec_type_t type)
{
    unsigned int i = 0;
    unsigned int size;

    if(wiced_codec_table == NULL)
        wiced_codec_table = wiced_default_codec_table;

    size = (sizeof wiced_codec_table)/sizeof(wiced_codec_interface_t*);

    for(i = 0; i < size; i++ )
    {
        if (wiced_codec_table[i]->type == type)
        {
            return wiced_codec_table[i];
        }
        else
            continue;
    }

    return NULL;
}
