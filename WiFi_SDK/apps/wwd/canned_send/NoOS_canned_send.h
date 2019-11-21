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


/**
 * @file
 * Provides structures and macros for Canned Send example application
 */
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/** @cond */
/******************************************************
 *                     Macros
 ******************************************************/

#define MAKE_IPV4_ADDRESS(a, b, c, d)          ((((uint32_t) (d)) << 24) | (((uint32_t) (c)) << 16) | (((uint32_t) (b)) << 8) | ((uint32_t) (a)))
#define SWAP16( a )                            ((uint16_t)( ((((uint16_t)(a))&0xff)<<8) + ((((uint16_t)(a))&0xff00)>>8) ))

/******************************************************
 *                    Constants
 ******************************************************/

#define MIN_IOCTL_BUFFER_SIZE                  (120)
#define PACKET_SIZE                            (sizeof(udp_packet_t) + MAX_PAYLOAD)
#define ARP_OPERATION_REQUEST                  (1)
#define ARP_OPERATION_REPLY                    (2)
#define ARP_PROTOCOL_TYPE_IPV4                 (0x0800)
#define ARP_HARDWARE_TYPE_ETHERNET             (1)

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    ETHER_TYPE_IPv4  = 0x0800,
/*    ETHER_TYPE_ARP   = 0x0806, */
    ETHER_TYPE_IPv6  = 0x86DD,
} ether_type_t;

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

#pragma pack(1)

typedef struct
{
    wiced_mac_t destination;
    wiced_mac_t source;
    uint16_t    ether_type;
} ethernet_header_t;

typedef struct
{
    uint16_t    hardware_type;
    uint16_t    protocol_type;
    uint8_t     hardware_address_length;
    uint8_t     protocol_address_length;
    uint16_t    operation;
    wiced_mac_t   sender_hardware_address;
    uint32_t    sender_protocol_address;
    wiced_mac_t   target_hardware_address;
    uint32_t    target_protocol_address;
} arp_message_t;

typedef struct
{
    uint8_t  header_length : 4;
    uint8_t  version      : 4;
    uint8_t  differentiated_services;
    uint16_t total_length;
    uint16_t identification;
    uint16_t flags_fragment_offset;
    uint8_t  time_to_live;
    uint8_t  protocol;
    uint16_t header_checksum;
    uint32_t source_address;
    uint32_t destination_address;
} ipv4_header_t;

typedef struct
{
    uint16_t  source_port;
    uint16_t  dest_port;
    uint16_t  udp_length;
    uint16_t  checksum;
} udp_header_t;

typedef struct
{
    char              reserved[WICED_LINK_OVERHEAD_BELOW_ETHERNET_FRAME_MAX];
    ethernet_header_t ethernet_header;
    arp_message_t     arp_message;
} arp_packet_t;

typedef struct
{
    char              reserved[WICED_LINK_OVERHEAD_BELOW_ETHERNET_FRAME_MAX];
    ethernet_header_t ethernet_header;
    ipv4_header_t     ip_header;
    udp_header_t      udp_header;
    char              data[1];
} udp_packet_t;

#pragma pack()

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/******************************************************
 *        Local Enumerations and Structures
 ******************************************************/

/** @endcond */

#ifdef __cplusplus
} /* extern "C" */
#endif
