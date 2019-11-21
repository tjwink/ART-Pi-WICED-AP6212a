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
 *                     Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/

#define APOLLO_REPORT_MAGIC_TAG         (0x5CAFF01D)

#define APOLLO_REPORT_VERSION           (1)
#define APOLLO_REPORT_STATS_VERSION     (2)

#define APOLLO_REPORT_PORT              (19705)

/******************************************************
 *                   Enumerations
 ******************************************************/

typedef enum
{
    APOLLO_REPORT_MSG_STATS = 1,
} APOLLO_REPORT_MSG_T;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    uint16_t version;
    int16_t  rssi;
    uint32_t speaker_channel;
    uint64_t rtp_packets_received;
    uint64_t rtp_packets_dropped;
    uint64_t audio_frames_played;
    uint64_t audio_frames_dropped;
    uint64_t audio_concealment_slcplc_cnt;
    uint64_t audio_concealment_fec_cnt;
} apollo_report_stats_t;

typedef struct
{
    uint32_t                magic;          /* Magic tag                        */
    uint16_t                version;        /* Apollo report header version     */
    uint8_t                 mac_addr[6];    /* MAC address of message sender    */
    uint32_t                msg_type;       /* APOLLO_REPORT_MSG_T message type */
    uint32_t                msg_length;     /* Length of message data           */
    uint8_t                 msg_data[0];    /* Message data                     */
} apollo_report_msg_t;


#define APOLLO_CSA_MSGTYPE          0x435341  /*  'C' 'S' 'A' */
#define APOLLO_CSA_PORT             (19715)
typedef struct
{
    uint32_t msg_type;       /* APOLLO_CSA_MSGTYPE */
    wiced_chan_switch_t cs;
} rmc_csa_msg_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/


#ifdef __cplusplus
} /* extern "C" */
#endif
