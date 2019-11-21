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

#ifdef __cplusplus
extern "C" {
#endif


/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_DO_NOT_CACHE    = 0,       /* Do not cache or store        */
    APOLLO_CACHE_TO_MEMORY = 1 << 0,  /* Cache to memory              */
    APOLLO_STORE_TO_NVRAM  = 1 << 1   /* Store to non-volatile memory */
} APOLLO_CACHING_MODE_T;

enum
{
    APOLLO_METADATA_TITLE_INDEX = 0,
    APOLLO_METADATA_ARTIST_INDEX,
    APOLLO_METADATA_ALBUM_INDEX,
    APOLLO_METADATA_TRACK_NUMBER_INDEX,
    APOLLO_METADATA_NUMBER_OF_TRACKS_INDEX,
    APOLLO_METADATA_GENRE_INDEX,
    APOLLO_METADATA_PLAYING_TIME_INDEX,

    APOLLO_METADATA_TYPE_COUNT_MAX
};

typedef enum
{
    APOLLO_METADATA_NONE             = 0,
    APOLLO_METADATA_TITLE            = 1 << APOLLO_METADATA_TITLE_INDEX,
    APOLLO_METADATA_ARTIST           = 1 << APOLLO_METADATA_ARTIST_INDEX,
    APOLLO_METADATA_ALBUM            = 1 << APOLLO_METADATA_ALBUM_INDEX,
    APOLLO_METADATA_TRACK_NUMBER     = 1 << APOLLO_METADATA_TRACK_NUMBER_INDEX,
    APOLLO_METADATA_NUMBER_OF_TRACKS = 1 << APOLLO_METADATA_NUMBER_OF_TRACKS_INDEX,
    APOLLO_METADATA_GENRE            = 1 << APOLLO_METADATA_GENRE_INDEX,
    APOLLO_METADATA_PLAYING_TIME     = 1 << APOLLO_METADATA_PLAYING_TIME_INDEX,
} APOLLO_METADATA_TYPE_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/**
 * Audio volume structure passed with
 * APOLLO_CMD_SENDER_COMMAND_VOLUME and APOLLO_CMD_EVENT_SET_VOLUME
 */
typedef struct apollo_volume_s
{
    int volume;
    APOLLO_CACHING_MODE_T caching_mode;
} apollo_volume_t;

/**
 * Metadata structure passed with
 * APOLLO_CMD_SENDER_COMMAND_METADATA and APOLLO_CMD_EVENT_METADATA
 */
typedef struct
{
    uint32_t type;
    uint16_t data_length[APOLLO_METADATA_TYPE_COUNT_MAX];
    uint8_t* data[APOLLO_METADATA_TYPE_COUNT_MAX];
} apollo_cmd_metadata_t;


typedef struct apollo_cmd_ambilight_s
{
    /* used on the source side */
    void         *amblt_front;
    void         *amblt_source;
    uint16_t     num_entries;
    char         **entries;
    /* used on the sink side */
    uint8_t      sink_data[255];
    uint16_t     sink_data_length;
} apollo_cmd_ambilight_t;



/******************************************************
 *                    Structures
 ******************************************************/

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /* extern "C" */
#endif
