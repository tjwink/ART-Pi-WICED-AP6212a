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

#include "wiced_tcpip.h"
#include "wiced_rtos.h"

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define GEDDAY_RECORD_TTL       (75 * 60)

/******************************************************
 *                   Enumerations
 ******************************************************/
typedef enum
{
    SERVICE_DISCOVER_RESULT_IPV4_ADDRESS_SLOT,
    SERVICE_DISCOVER_RESULT_IPV6_ADDRESS_SLOT
} gedday_discovery_result_ip_slot_t;
/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_ip_address_t     ip[ 2 ];
    /* TODO: IPv6 address */
    uint16_t               port;
    const char*            service_name;  /* This variable is used internally */
    /* This memory for txt string must be released as soon as service structure is processed by the application */
    char*                  txt;
    /* buffers for instance_name and hostname are allocated dynamically by Gedday
     * User of gedday_discover_service must make sure that the memory gets freed as soon as this names are no longer needed */
    char*                  instance_name;
    char*                  hostname;
    wiced_semaphore_t*     semaphore;     /* This variable is used internally */
    volatile wiced_bool_t  is_resolved;
} gedday_service_t;

typedef struct
{
    char*         buffer;
    uint16_t      buffer_length;
    unsigned int  current_size;
    wiced_mutex_t mutex;
} gedday_text_record_t;

/******************************************************
 *                 Global Variables
 ******************************************************/

/******************************************************
 *               Function Declarations
 ******************************************************/

/*****************************************************************************/
/**
 *
 *  @defgroup gedday        Gedday
 *  @ingroup  ipcoms
 *
 *  @addtogroup gedday      Gedday Library
 *
 * Gedday library is a WICED implementation of mDNS
 *
 *                                     API State machine
 *                                     =================
 *
 *
 *     State machine for mDNS service discovery                    State machine for registering mDNS service
 *     ----------------------------------------                    ------------------------------------------
 *
 *                +---------------+                                            +---------------+
 *                | gedday_init() |                                            | gedday_init() |
 *                +-------+-------+                                            +-------+-------+
 *                        |                                                            |
 *                        v                                                            v
 *           +------------+--------------+                                  +----------+-----------+
 *           | gedday_discover_service() |                                  | gedday_add_service() |
 *           +------------+--------------+                                  +----------+-----------+
 *                        |                                                            |                    +--+
 *                        v                                                            v                       |
 *               +--------+--------+                                    +--------------+--------------+        |
 *               | gedday_deinit() |                                    | gedday_text_record_create() |        |
 *               +-----------------+                                    +--------------+--------------+        |
 *                                                                                     |                       |
 *                                                                                     v                       |
 *                                                                +--------------------+--------------------+  |
 *                                                                | gedday_text_record_set_key_value_pair() |  |                        (OPTIONAL)
 *                                                                +--------------------+--------------------+  |
 *                                                                                     |                       |  Create and add dynamic TXT record (containing key-value pairs)
 *                                                                                     v                       |                  with existing mDNS service
 *                                                                    +----------------+-----------------+     |
 *                                                                    | gedday_add_dynamic_text_record() |     |
 *                                                                    +----------------+-----------------+     |
 *                                                                                     |                       |
 *                                                                                     v                       |
 *                                                                      +--------------+--------------+        |
 *                                                                      | gedday_text_record_delete() |        |
 *                                                                      +--------------+--------------+        |
 *                                                                                     |                    +--+
 *                                                                                     v
 *                                                                        +------------+------------+
 *                                                                        | gedday_update_service() |                        (OPTIONAL)   Re-Advertise mDNS Service
 *                                                                        +------------+------------+
 *                                                                                     |
 *                                                                                     v
 *                                                                        +-------------------------+
 *                                                                        | gedday_remove_service() |
 *                                                                        +-------------------------+
 *                                                                                     |
 *                                                                                     v
 *                                                                             +-------+---------+
 *                                                                             | gedday_deinit() |
 *                                                                             +-----------------+
 *
 *  @{
 */
/*****************************************************************************/
/** Initializes Gedday library and it's components.
 *   - Prepares the infrastructure required for the library (Ex: threads, socket, message queues, etc).
 *   - Joins mDNS multicast group using the given network interface.
 *   - Registers IPv4 (A) and IPv6 (AAAA) records on the network.
 *
 * @param interface [in]    : Interface to be used to join multicast group.
 * @param desired_name [in] : Desired instance name for the mDNS record which is a 'null' terminated string.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_init                          ( wiced_interface_t interface, const char* desired_name );

/** DeInitializes Gedday library and it's components. Also frees all the mDNS records created after initialization.
 *
 * @return None
 */
void                  gedday_deinit                        ( void );

/** Discovers the requested mDNS service. Return immediately if record is found else times out after few seconds.
 *
 * @param service_query [in]   : Interface to be used to join multicast group.
 * @param service_result [out] : Contains the service information on return. Caller of this API should free the host-name, instance-name and txt-string after using it.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_discover_service              ( const char* service_query, gedday_service_t* service_result );

/** Registers mDNS service and advertise the same on the network.
 *
 * @param instance_name [in] : Instance name for the mDNS service.
 * @param service_name [in]  : mDNS Service name. Example: "_http._tcp.local".
 * @param port [in]          : Port number for the service.
 * @param ttl [in]           : Time-To-Live value for the mDNS record in seconds.
 * @param txt [in]           : TXT record for the service.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_add_service                   ( const char* instance_name, const char* service_name, uint16_t port, uint32_t ttl, const char* txt );

/** Adds dynamic TXT record with the existing mDNS service.
 *
 * @param instance_name [in] : Instance name of the mDNS service.
 * @param service_name [in]  : mDNS Service name. Example: "_http._tcp.local".
 * @param text_record [in]   : Dynamic TXT record information for the service.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_add_dynamic_text_record       ( const char* instance_name, const char* service_name, gedday_text_record_t* text_record );

/** Re-advertises the existing mDNS service.
 *
 * @param instance_name [in] : Instance name of the mDNS service.
 * @param service_name [in]  : mDNS Service name. Example: "_http._tcp.local".
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_update_service                ( const char* instance_name, const char* service_name );

/** Updates IPv4 address on A record and registers the same on the network.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_update_ip                     ( void );

/** Updates IPv6 address on AAAA record and registers the same on the network.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_update_ipv6                   ( void );

/** Unregisters the mDNS service from the network.
 *
 * @param instance_name [in] : Instance name of the mDNS service.
 * @param service_name [in]  : mDNS Service name. Example: "_http._tcp.local".
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_remove_service                ( const char* instance_name, const char* service_name );

/** Returns the host name of the service.
 *
 * @return Host name of the service
 */
const char*           gedday_get_hostname                  ( void );

/** Initializes the members required for creating the TXT record. This function call marks the START of TXT record creation.
 *
 * @param text_record_ptr [in] : Pointer to TXT record structure to be initialized.
 * @param buffer_length [in]   : Length of the buffer.
 * @param buffer [in]          : Pointer to the buffer used for storing key-value-pairs of TXT record.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_text_record_create            ( gedday_text_record_t* text_record_ptr, uint16_t buffer_length, void *buffer );

/** Adds the given Key-Value pair in TXT record string.
 *
 * @param text_record_ptr [in] : Pointer to TXT record structure.
 * @param key [in]             : TXT record KEY.
 * @param value [in]           : Value associated with TXT record KEY.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_text_record_set_key_value_pair( gedday_text_record_t* text_record_ptr, char* key, char* value );

/** Returns the TXT record string.
 *
 * @param text_record_ptr [in] : Pointer to TXT record structure.
 *
 * @return @ref wiced_result_t
 */
char*                 gedday_text_record_get_string        ( gedday_text_record_t* text_record_ptr );

/** Deinitializes the members required for deleting the TXT record. This function call marks the END of TXT record creation.
 *
 * @param text_record_ptr [in] : Pointer to TXT record structure to be deinitialized.
 *
 * @return @ref wiced_result_t
 */
wiced_result_t        gedday_text_record_delete            ( gedday_text_record_t* text_record_ptr );

/** @} */

#ifdef __cplusplus
} /* extern "C" */
#endif
