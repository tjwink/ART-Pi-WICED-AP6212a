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
 *  Unit Tester for DHCP server
 *
 *  Runs a suite of tests on the DHCP server to attempt
 *  to discover bugs
 */
#ifndef INCLUDED_DHCP_SERVER_UNIT_H_
#define INCLUDED_DHCP_SERVER_UNIT_H_

#include <stdio.h>
#include <string.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


#pragma pack(1)
typedef struct
{
    uint8_t  opcode;                     /* packet opcode type */
    uint8_t  hardware_type;              /* hardware addr type */
    uint8_t  hardware_addr_len;          /* hardware addr length */
    uint8_t  hops;                       /* gateway hops */
    uint32_t transaction_id;             /* transaction ID */
    uint16_t second_elapsed;             /* seconds since boot began */
    uint16_t flags;
    uint8_t  client_ip_addr[4];          /* client IP address */
    uint8_t  your_ip_addr[4];            /* 'your' IP address */
    uint8_t  server_ip_addr[4];          /* server IP address */
    uint8_t  gateway_ip_addr[4];         /* gateway IP address */
    uint8_t  client_hardware_addr[16];   /* client hardware address */
    uint8_t  legacy[192];
    /* as of RFC2131 it is variable length */
} dhcp_header_t;



#pragma pack()

#define BOOTP_OP_REQUEST                (1)
#define BOOTP_OP_REPLY                  (2)

#define BOOTP_HTYPE_ETHERNET            (1)

/* DHCP commands */
#define DHCPDISCOVER                    (1)
#define DHCPOFFER                       (2)
#define DHCPREQUEST                     (3)
#define DHCPDECLINE                     (4)
#define DHCPACK                         (5)
#define DHCPNAK                         (6)
#define DHCPRELEASE                     (7)
#define DHCPINFORM                      (8)

#define DHCP_MAGIC_COOKIE               0x63, 0x82, 0x53, 0x63

#define DHCP_OPTION_CODE_REQUEST_IP_ADDRESS (50)
#define DHCP_OPTION_CODE_LEASE_TIME         (51)
#define DHCP_OPTION_CODE_DHCP_MESSAGE_TYPE  (53)
#define DHCP_OPTION_CODE_SERVER_ID          (54)
#define DHCP_OPTION_CODE_PARAM_REQ_LIST     (55)
#define DHCP_OPTION_CODE_MAX_MSG_SIZE       (57)
#define DHCP_OPTION_CODE_CLIENT_ID          (61)
#define DHCP_OPTION_END                     (255)


typedef struct
{
        const char* bytes;
        int size;
} dhcp_option_t;

#define MAKE_DHCP_OPTION( var ) { (var), sizeof(var) }

extern const dhcp_header_t normal_dhcp_discover_header;
extern const dhcp_option_t normal_dhcp_discover_options[];
extern const dhcp_option_t dhcp_discover_w_request_options[];

#ifdef __cplusplus
} /*extern "C" */
#endif

#endif /* ifndef INCLUDED_DHCP_SERVER_UNIT_H_ */
