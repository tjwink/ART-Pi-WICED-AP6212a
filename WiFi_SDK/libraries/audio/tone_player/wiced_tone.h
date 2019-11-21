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

#ifndef WICED_TONE_H
#define WICED_TONE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "wiced_audio.h"

/**  \file    wiced_bt.h
        \brief  API header file for WICED Tone Player functionality accessed from application.
        Application can make use of Tone APIs for playing short tones stored on the device.
*/

/** \defgroup tone WICED TONE
*
* @{
*/

/** \brief Play tone
*
* This function starts playback of locally stored tone data.
*
* \param  data           pointer to tone data location.
* \param  length        length of data in bytes.
* \param  audio_info  audio configuration information, see @wiced_audio_config_t
*                               sample rate, bits per sample, number of channels and volume.
* \param  repeat       number of times the tone should be repeated, 0 - no repetition.
*                              > 0, repeat n times.
*
* \return    WICED_SUCCESS : on success;
*                WICED_ERROR   : if an error occurred
*/
wiced_result_t wiced_tone_play(uint8_t* data, uint32_t length, wiced_audio_config_t* audio_info, uint8_t repeat);


/** \brief Stop the tone currently being played
*
* This function stops the playback of tone, if currently active.
*
* \return    WICED_SUCCESS : on success;
*                WICED_ERROR   : if an error occurred
*/
wiced_result_t wiced_tone_stop( void );


/** @} */ // end of tone

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif //#ifndef WICED_TONE_H

