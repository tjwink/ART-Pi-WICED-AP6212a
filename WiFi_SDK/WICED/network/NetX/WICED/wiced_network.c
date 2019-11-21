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
 *  Wiced NetX networking layer
 */

#include "wiced.h"
#include "wiced_network.h"
#include "wiced_wifi.h"
#include "wiced_utilities.h"
#include "wiced_low_power.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "nx_api.h"
#include "nx_user.h"
#include "wwd_management.h"
#include "wwd_network.h"
#include "dhcp_server.h"
#include "dns.h"
#include "platform_dct.h"
#include "platform_ethernet.h"
#include "internal/wiced_internal_api.h"
#include "wwd_network_constants.h"
#include "wiced_framework.h"
#include "wwd_buffer_interface.h"
#ifdef ENABLE_NX_BSD_SOCKET
#include "nx_bsd.h"
#endif

/******************************************************
 *                      Macros
 ******************************************************/

#define DRIVER_FOR_IF( interface )                ( wiced_ip_driver_entries[ ( interface ) & 3 ] )
#define STACK_FOR_IF( interface )                 ( wiced_ip_stack[          ( interface ) & 3 ] )
#define ARP_FOR_IF( interface )                   ( wiced_arp_cache[         ( interface ) & 3 ] )
#define DHCP_CLIENT_IS_INITIALISED( interface )   ( DHCP_HANDLE(interface).nx_dhcp_id == NX_DHCP_ID )

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
#define DHCP_HANDLE( interface )                  ( *wiced_dhcp_handle[              ( (interface & 3) != WICED_AP_INTERFACE ? 0 : 1 ) ])
#define DHCP_RECORD_HANDLE( interface )           ( *wiced_dhcp_record_handle[       ( (interface & 3) != WICED_AP_INTERFACE ? 0 : 1 ) ])
#define DHCP_RECORD_HANDLE_VALID( interface )     ( *wiced_dhcp_record_handle_valid[ ( (interface & 3) != WICED_AP_INTERFACE ? 0 : 1 ) ])
#define DHCP_CLIENT_HOSTNAME( interface )         ( dhcp_client_hostname[            ( (interface & 3) != WICED_AP_INTERFACE ? 0 : 1 ) ])
#else
#define DHCP_HANDLE( interface )                  ( *wiced_dhcp_handle[   ( interface != WICED_ETHERNET_INTERFACE ? 0 : 1 ) ])
#define DHCP_RECORD_HANDLE( interface )           ( *wiced_dhcp_record_handle[   ( interface != WICED_ETHERNET_INTERFACE ? 0 : 1 ) ])
#define DHCP_RECORD_HANDLE_VALID( interface )     ( *wiced_dhcp_record_handle_valid[   ( interface != WICED_ETHERNET_INTERFACE ? 0 : 1 ) ])
#define DHCP_CLIENT_HOSTNAME( interface )         ( dhcp_client_hostname[ ( interface != WICED_ETHERNET_INTERFACE ? 0 : 1 ) ])
#endif
#define AUTO_IP_INITIALISED( auto_ip_handle )     ( auto_ip_handle.nx_auto_ip_id == NX_AUTO_IP_ID )

#define CONFIG_GET_FOR_IF( interface )            ( wiced_config_cache[         ( interface ) & 3 ] )
#define CONFIG_SET_FOR_IF( interface, config )      wiced_config_cache[         ( interface ) & 3 ] = config;

/******************************************************
 *                    Constants
 ******************************************************/

#ifdef WICED_USE_ETHERNET_INTERFACE
    #ifdef WICED_ETHERNET_RX_PACKET_POOL_SIZE
        #ifdef RX_PACKET_POOL_SIZE
            #undef RX_PACKET_POOL_SIZE
        #endif /* RX_PACKET_POOL_SIZE */
        #define RX_PACKET_POOL_SIZE           WICED_ETHERNET_RX_PACKET_POOL_SIZE
    #elif !defined(RX_PACKET_POOL_SIZE)
        #define RX_PACKET_POOL_SIZE           (32 /* WWD default RX DMA ring size */ + 4 /* Ethernet default RX DMA ring size */ )
    #endif /* WICED_ETHERNET_RX_PACKET_POOL_SIZE  */
#endif /* WICED_USE_ETHERNET_INTERFACE */

#ifndef TX_PACKET_POOL_SIZE
#define TX_PACKET_POOL_SIZE         (7)
#endif

#ifndef RX_PACKET_POOL_SIZE
#define RX_PACKET_POOL_SIZE         (7)
#endif

#ifndef COM_PACKET_POOL_SIZE
#define COM_PACKET_POOL_SIZE (14)
#endif
#ifndef PACKET_POOL_SIZE_128
#define PACKET_POOL_SIZE_128 (10)
#endif
#define NUM_BUFFERS_POOL_SIZE(x)    ((WICED_LINK_MTU_ALIGNED + sizeof(NX_PACKET)+1)*(x))

#define APP_COM_BUFFER_POOL_SIZE     NUM_BUFFERS_POOL_SIZE(COM_PACKET_POOL_SIZE)
#define APP_TX_BUFFER_POOL_SIZE     NUM_BUFFERS_POOL_SIZE(TX_PACKET_POOL_SIZE)
#define APP_RX_BUFFER_POOL_SIZE     NUM_BUFFERS_POOL_SIZE(RX_PACKET_POOL_SIZE)

#define NUM_PACKET_POOL_SIZE_128(x)    ((WICED_POOL_PAYLOAD_SIZE_128_ALIGNED + sizeof(NX_PACKET)+1)*(x))
#define WICED_PACKET_POOL_SIZE_128    NUM_PACKET_POOL_SIZE_128(PACKET_POOL_SIZE_128)
#define MAXIMUM_IP_ADDRESS_CHANGE_CALLBACKS           (2)

/*
 * Older versions of the NetX DHCP application parsed a set number of DNS server addresses
 * out of the DHCP options buffer. The newer versions (i.e. 5.10) store the entire options buffer
 * and return the entire set of DNS addresses from the server. The buffer used to retrieve
 * the addresses must be large enough to hold all the addresses from the options buffer
 * or the DHCP client code will return an error.
 */
#define DHCP_NUM_DNS_SERVERS                        (10)
#define DHCP_DEFAULT_CLIENT_OBJECT_NAME            ((char*)"WICED DHCP Client")

#ifdef AUTO_IP_ENABLED
#define AUTO_IP_STACK_SIZE         (4096)
#define AUTO_IP_PRIORITY           (3)
#endif /* AUTO_IP_ENABLED */

/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

typedef struct
{
    wiced_ip_address_change_callback_t callback;
    void*                              arg;
} ip_address_change_callback_t;

/******************************************************
 *                 Static Variables
 ******************************************************/
#ifdef WICED_USE_COMMON_PKT_POOL
ALIGNED_PRE(4) static uint8_t buffer_pool_memory_128 [WICED_PACKET_POOL_SIZE_128 + PLATFORM_L1_CACHE_BYTES] ALIGNED(4);
ALIGNED_PRE(4) static uint8_t buffer_pool_memory_common [APP_COM_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES] ALIGNED(4);
#else
/* consider instead of allocating more and then fixup pointer instruct compiler to align array */
ALIGNED_PRE(4) static uint8_t tx_buffer_pool_memory [APP_TX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES] ALIGNED(4);
ALIGNED_PRE(4) static uint8_t rx_buffer_pool_memory [APP_RX_BUFFER_POOL_SIZE + PLATFORM_L1_CACHE_BYTES] ALIGNED(4);
#endif

#ifdef WICED_USE_WIFI_STA_INTERFACE
    static NX_IP            wifi_sta_ip_handle;
    static char             wifi_sta_ip_stack[IP_STACK_SIZE];
    static char             wifi_sta_arp_cache[ARP_CACHE_SIZE];
    static NX_DHCP          wifi_sta_dhcp_handle;
    static wiced_hostname_t wifi_sta_dhcp_client_hostname;
    #define WIFI_STA_IP_HANDLE             &wifi_sta_ip_handle
    #define WIFI_STA_IP_STACK              wifi_sta_ip_stack
    #define WIFI_STA_ARP_CACHE             wifi_sta_arp_cache
    #define WIFI_STA_DHCP_HANDLE           &wifi_sta_dhcp_handle
    #define WIFI_STA_DHCP_CLIENT_HOSTNAME  &wifi_sta_dhcp_client_hostname
#else
    #define WIFI_STA_IP_HANDLE            (NULL)
    #define WIFI_STA_IP_STACK             (NULL)
    #define WIFI_STA_ARP_CACHE            (NULL)
    #define WIFI_STA_DHCP_HANDLE          (NULL)
    #define WIFI_STA_DHCP_CLIENT_HOSTNAME (NULL)
#endif

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    static NX_IP            wifi_sta2_ip_handle;
    static char             wifi_sta2_ip_stack[IP_STACK_SIZE];
    static char             wifi_sta2_arp_cache[ARP_CACHE_SIZE];
    static NX_DHCP          wifi_sta2_dhcp_handle;
    static wiced_hostname_t wifi_sta2_dhcp_client_hostname;
    #define WIFI_STA2_IP_HANDLE             &wifi_sta2_ip_handle
    #define WIFI_STA2_IP_STACK              wifi_sta2_ip_stack
    #define WIFI_STA2_ARP_CACHE             wifi_sta2_arp_cache
    #define WIFI_STA2_DHCP_HANDLE           &wifi_sta2_dhcp_handle
    #define WIFI_STA2_DHCP_CLIENT_HOSTNAME  &wifi_sta2_dhcp_client_hostname
#else
    /* AP should be used in this case, but define these anyway. */
    #define WIFI_STA2_IP_HANDLE            (NULL)
    #define WIFI_STA2_IP_STACK             (NULL)
    #define WIFI_STA2_ARP_CACHE            (NULL)
    #define WIFI_STA2_DHCP_HANDLE          (NULL)
    #define WIFI_STA2_DHCP_CLIENT_HOSTNAME (NULL)
#endif

#ifdef WICED_USE_WIFI_AP_INTERFACE
    static NX_IP wifi_ap_ip_handle;
    static char  wifi_ap_ip_stack[IP_STACK_SIZE];
    static char  wifi_ap_arp_cache[ARP_CACHE_SIZE];
    #define WIFI_AP_IP_HANDLE    &wifi_ap_ip_handle
    #define WIFI_AP_IP_STACK     wifi_ap_ip_stack
    #define WIFI_AP_ARP_CACHE    wifi_ap_arp_cache
#else
    #define WIFI_AP_IP_HANDLE    (NULL)
    #define WIFI_AP_IP_STACK     (NULL)
    #define WIFI_AP_ARP_CACHE    (NULL)
#endif

#ifdef WICED_USE_WIFI_P2P_INTERFACE
    static NX_IP wifi_p2p_ip_handle;
    static char  wifi_p2p_ip_stack[IP_STACK_SIZE];
    static char  wifi_p2p_arp_cache[ARP_CACHE_SIZE];
    #define WIFI_P2P_IP_HANDLE    &wifi_p2p_ip_handle
    #define WIFI_P2P_IP_STACK     wifi_p2p_ip_stack
    #define WIFI_P2P_ARP_CACHE    wifi_p2p_arp_cache
#else
    #define WIFI_P2P_IP_HANDLE    (NULL)
    #define WIFI_P2P_IP_STACK     (NULL)
    #define WIFI_P2P_ARP_CACHE    (NULL)
#endif

#ifdef WICED_USE_ETHERNET_INTERFACE
    static NX_IP             ethernet_ip_handle;
    static char              ethernet_ip_stack[IP_STACK_SIZE];
    static char              ethernet_arp_cache[ARP_CACHE_SIZE];
#ifndef WICED_USE_WIFI_TWO_STA_INTERFACE
    static NX_DHCP           ethernet_dhcp_handle;
    static wiced_hostname_t  ethernet_dhcp_client_hostname;
#endif
    #define ETHERNET_IP_HANDLE             &ethernet_ip_handle
    #define ETHERNET_IP_STACK              ethernet_ip_stack
    #define ETHERNET_ARP_CACHE             ethernet_arp_cache
    #define ETHERNET_DHCP_HANDLE           &ethernet_dhcp_handle
    #define ETHERNET_DHCP_CLIENT_HOSTNAME  &ethernet_dhcp_client_hostname
#else
    #define ETHERNET_IP_HANDLE            (NULL)
    #define ETHERNET_IP_STACK             (NULL)
    #define ETHERNET_ARP_CACHE            (NULL)
    #define ETHERNET_DHCP_HANDLE          (NULL)
    #define ETHERNET_DHCP_CLIENT_HOSTNAME (NULL)
#endif

#ifdef ENABLE_NX_BSD_SOCKET
static char    bsd_socket_stack[2048];
#endif

/* Network objects */
static char* wiced_ip_stack[ WICED_INTERFACE_MAX ] =
{
    [WICED_STA_INTERFACE]      = WIFI_STA_IP_STACK,
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    [WICED_AP_INTERFACE]       = WIFI_STA2_IP_STACK,
#else
    [WICED_AP_INTERFACE]       = WIFI_AP_IP_STACK,
#endif
    [WICED_P2P_INTERFACE]      = WIFI_P2P_IP_STACK,
    [WICED_ETHERNET_INTERFACE] = ETHERNET_IP_STACK,
};

static char* wiced_arp_cache[ WICED_INTERFACE_MAX ] =
{
    [WICED_STA_INTERFACE]      = WIFI_STA_ARP_CACHE,
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    [WICED_AP_INTERFACE]      = WIFI_STA2_ARP_CACHE,
#else
    [WICED_AP_INTERFACE]       = WIFI_AP_ARP_CACHE,
#endif
    [WICED_P2P_INTERFACE]      = WIFI_P2P_ARP_CACHE,
    [WICED_ETHERNET_INTERFACE] = ETHERNET_ARP_CACHE,
};

static wiced_network_config_t wiced_config_cache[ WICED_INTERFACE_MAX ] =
{
    [WICED_STA_INTERFACE]      = WICED_USE_EXTERNAL_DHCP_SERVER,
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    [WICED_AP_INTERFACE]       = WICED_USE_EXTERNAL_DHCP_SERVER,
#else
    [WICED_AP_INTERFACE]       = WICED_USE_INTERNAL_DHCP_SERVER,
#endif
    [WICED_P2P_INTERFACE]      = WICED_USE_EXTERNAL_DHCP_SERVER,
    [WICED_ETHERNET_INTERFACE] = WICED_USE_EXTERNAL_DHCP_SERVER,
};

/*
 * Note: The DHCP related macros determine the mapping between interface and array entry
 */
static NX_DHCP* wiced_dhcp_handle[ 2 ] =
{
    [ 0 ] = WIFI_STA_DHCP_HANDLE,
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    [ 1 ] = WIFI_STA2_DHCP_HANDLE,
#else
    [ 1 ] = ETHERNET_DHCP_HANDLE,
#endif
};

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
static NX_DHCP_CLIENT_RECORD WICED_DEEP_SLEEP_SAVED_VAR( sta2_dhcp_record_handle );
#else
static NX_DHCP_CLIENT_RECORD WICED_DEEP_SLEEP_SAVED_VAR( ethernet_dhcp_record_handle );
#endif
static NX_DHCP_CLIENT_RECORD WICED_DEEP_SLEEP_SAVED_VAR( sta_dhcp_record_handle );

#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
static wiced_bool_t WICED_DEEP_SLEEP_SAVED_VAR( sta2_dhcp_record_handle_valid ) = WICED_FALSE;
#else
static wiced_bool_t WICED_DEEP_SLEEP_SAVED_VAR( ethernet_dhcp_record_handle_valid ) = WICED_FALSE;
#endif
static wiced_bool_t WICED_DEEP_SLEEP_SAVED_VAR( sta_dhcp_record_handle_valid ) = WICED_FALSE;

/* brru: Why are these opposite of previous tables, with ethernet in index 0 and STA in 1 ?? */
static NX_DHCP_CLIENT_RECORD *wiced_dhcp_record_handle[ 2 ] =
{
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    &sta2_dhcp_record_handle,
#else
    &ethernet_dhcp_record_handle,
#endif
    &sta_dhcp_record_handle
};

static wiced_bool_t* wiced_dhcp_record_handle_valid[ 2 ] = {
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    &sta2_dhcp_record_handle_valid,
#else
    &ethernet_dhcp_record_handle_valid,
#endif
    &sta_dhcp_record_handle_valid
};
/*
 * Note: The DHCP related macros determine the mapping between interface and array entry
 */
static wiced_hostname_t* dhcp_client_hostname[ 2 ] =
{
    [ 0 ] = WIFI_STA_DHCP_CLIENT_HOSTNAME,
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    [ 1 ] = WIFI_STA2_DHCP_CLIENT_HOSTNAME,
#else
    [ 1 ] = ETHERNET_DHCP_CLIENT_HOSTNAME,
#endif
};

/******************************************************
 *                 Global Variables
 ******************************************************/

NX_IP* wiced_ip_handle[ WICED_INTERFACE_MAX ] =
{
    [WICED_STA_INTERFACE]      = WIFI_STA_IP_HANDLE,
#ifdef WICED_USE_WIFI_TWO_STA_INTERFACE
    [WICED_AP_INTERFACE]       = WIFI_STA2_IP_HANDLE,
#else
    [WICED_AP_INTERFACE]       = WIFI_AP_IP_HANDLE,
#endif
    [WICED_P2P_INTERFACE]      = WIFI_P2P_IP_HANDLE,
    [WICED_ETHERNET_INTERFACE] = ETHERNET_IP_HANDLE,
};

NX_PACKET_POOL wiced_packet_pools[ WICED_NUM_PACKET_POOLS ]; /* 0=TX/COM, 1=RX/Default */

NX_PACKET_POOL wiced_application_tx_packet_pool;
NX_PACKET_POOL wiced_application_rx_packet_pool;

#ifdef AUTO_IP_ENABLED
static NX_AUTO_IP auto_ip_handle;
static uint8_t    auto_ip_stack[ AUTO_IP_STACK_SIZE ];
ULONG             original_auto_ip_address = 0;
#endif

/* One DHCP client handle for STA interface and one for Ethernet interface */

static wiced_dhcp_server_t internal_dhcp_server;

static void (* const wiced_ip_driver_entries[ 4 ])(struct NX_IP_DRIVER_STRUCT *) =
{
    [WICED_STA_INTERFACE] = wiced_sta_netx_driver_entry,
    [WICED_AP_INTERFACE]  = wiced_ap_netx_driver_entry,
    [WICED_P2P_INTERFACE] = wiced_p2p_netx_driver_entry,

#ifdef WICED_USE_ETHERNET_INTERFACE
    [WICED_ETHERNET_INTERFACE] = wiced_ethernet_netx_driver_entry,
#endif
};

const wiced_ip_address_t INITIALISER_IPV4_ADDRESS( wiced_ip_broadcast, NX_IP_LIMITED_BROADCAST );

/* IP status callback variables */
static ip_address_change_callback_t wiced_ip_address_change_callbacks[MAXIMUM_IP_ADDRESS_CHANGE_CALLBACKS];

/* Network suspension variables */
static uint32_t     network_suspend_start_time  [ 4 ];
static uint32_t     network_suspend_end_time    [ 4 ];
static wiced_bool_t network_is_suspended = WICED_FALSE;

/* Wi-Fi power save state */
static uint8_t wifi_powersave_mode         = 0;
static uint16_t wifi_return_to_sleep_delay = 0;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static void           ip_address_changed_handler( NX_IP* ip_handle, VOID* additional_info );
static wiced_result_t dhcp_client_init( wiced_interface_t interface, NX_PACKET_POOL* packet_pool );
static wiced_result_t dhcp_client_deinit( wiced_interface_t interface );
static wiced_result_t wiced_network_restore_dhcp_state_from_deep_sleep( wiced_interface_t interface );
static void wiced_network_save_dhcp_state_for_deep_sleep( wiced_interface_t interface );

static wiced_bool_t   tcp_sockets_are_closed( wiced_interface_t interface );
static wiced_result_t wiced_network_suspend_layers( wiced_interface_t interface );
static wiced_result_t wiced_network_resume_layers( wiced_interface_t interface );

static void           wiced_call_link_up_callbacks( wiced_interface_t interface );
static void           wiced_call_link_down_callbacks( wiced_interface_t interface );

static ULONG wiced_network_init_packet_pool( NX_PACKET_POOL* pool, const char* pool_name, uint8_t* memory_pointer, uint32_t memory_size, uint16_t payload_size );

static wiced_result_t wiced_ip_driver_notify( wiced_interface_t interface, wiced_bool_t up );

/******************************************************
 *               Function Definitions
 ******************************************************/
wiced_result_t wiced_network_init( void )
{
    /* Initialize the NetX system.  */
    WPRINT_NETWORK_INFO(("Initialising NetX " NetX_VERSION "\n"));
    nx_system_initialize( );

    memset( ip_networking_inited, 0, WICED_INTERFACE_MAX * sizeof(wiced_bool_t) );

    /* Create packet pools for transmit and receive */
    WPRINT_NETWORK_INFO(("Creating Packet pools\n"));
#ifdef WICED_USE_COMMON_PKT_POOL
    if ( wiced_network_init_packet_pool( &wiced_packet_pools[0], "", buffer_pool_memory_common, sizeof(buffer_pool_memory_common), WICED_LINK_MTU_ALIGNED ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR(("Couldn't create common packet pool\n"));
        return WICED_ERROR;
    }
    if ( wiced_network_init_packet_pool( &wiced_packet_pools[1], "", buffer_pool_memory_128, sizeof(buffer_pool_memory_128), WICED_POOL_PAYLOAD_SIZE_128_ALIGNED ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR(("Couldn't create default packet pool\n"));
        return WICED_ERROR;
    }
#else
    if ( wiced_network_init_packet_pool( &wiced_packet_pools[0], "", tx_buffer_pool_memory, sizeof(tx_buffer_pool_memory), WICED_LINK_MTU_ALIGNED ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR(("Couldn't create TX packet pool\n"));
        return WICED_ERROR;
    }
    if ( wiced_network_init_packet_pool( &wiced_packet_pools[1], "", rx_buffer_pool_memory, sizeof(rx_buffer_pool_memory), WICED_LINK_MTU_ALIGNED ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR(("Couldn't create RX packet pool\n"));
        return WICED_ERROR;
    }
#endif
    if ( wwd_buffer_init( wiced_packet_pools ) != WWD_SUCCESS )
    {
        WPRINT_NETWORK_ERROR(("Could not initialize buffer interface\n"));
        return WICED_ERROR;
    }

    memset( &internal_dhcp_server, 0, sizeof( internal_dhcp_server ) );
    memset( wiced_ip_address_change_callbacks, 0, MAXIMUM_IP_ADDRESS_CHANGE_CALLBACKS * sizeof(wiced_ip_address_change_callback_t) );

    wiced_rtos_init_mutex( &link_subscribe_mutex );

    memset( link_up_callbacks_wireless,   0, sizeof( link_up_callbacks_wireless ) );
    memset( link_down_callbacks_wireless, 0, sizeof( link_down_callbacks_wireless ) );

#ifdef WICED_USE_ETHERNET_INTERFACE
    memset( link_up_callbacks_ethernet,   0, sizeof(link_up_callbacks_ethernet) );
    memset( link_down_callbacks_ethernet, 0, sizeof(link_down_callbacks_ethernet) );
#endif

    return WICED_SUCCESS;
}

wiced_result_t wiced_network_deinit( void )
{
    nx_packet_pool_delete(&wiced_packet_pools[0]);
    nx_packet_pool_delete(&wiced_packet_pools[1]);
    wiced_rtos_deinit_mutex( &link_subscribe_mutex );
    return WICED_SUCCESS;
}

wiced_result_t wiced_network_create_packet_pool( uint8_t* memory_pointer, uint32_t memory_size, wiced_network_packet_dir_t direction )
{
    wiced_result_t result = WICED_ERROR;

    if ( direction == WICED_NETWORK_PACKET_TX )
    {
        if ( wiced_network_init_packet_pool( &wiced_application_tx_packet_pool, "application_tx", memory_pointer, memory_size, WICED_LINK_MTU_ALIGNED ) != NX_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ("Couldn't create TX packet pool\n") );
        }
        else
        {
            host_buffer_add_application_defined_pool( &wiced_application_tx_packet_pool, WWD_NETWORK_TX );
            result = WICED_SUCCESS;
        }
    }
    else if ( direction == WICED_NETWORK_PACKET_RX )
    {
        if ( wiced_network_init_packet_pool( &wiced_application_rx_packet_pool, "application_rx", memory_pointer, memory_size, WICED_LINK_MTU_ALIGNED ) != NX_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ("Couldn't create RX packet pool\n") );
        }
        else
        {
            host_buffer_add_application_defined_pool( &wiced_application_rx_packet_pool, WWD_NETWORK_RX );
            result = WICED_SUCCESS;
        }
    }

    return result;
}

wiced_result_t wiced_ip_up( wiced_interface_t interface, wiced_network_config_t config, const wiced_ip_setting_t* ip_settings )
{
    UINT        status;
    /* Pool zero is TX pool or Common pool based on the flag WICED_USE_COMMON_PKT_POOL */
    UINT        pool = 0;

#ifdef WICED_USE_COMMON_PKT_POOL
#ifdef WICED_USE_128_POOL_FOR_IP_STACK
    /* If configured to use 128 byte as default netx IP pool, then change it pool 1, which is 128 byte pool */
    pool = 1;
#endif
#endif
    if ( IP_NETWORK_IS_INITED( interface ) == WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

    /* Enable the network interface  */
    if ( ( config == WICED_USE_STATIC_IP || config == WICED_USE_INTERNAL_DHCP_SERVER ) && ip_settings != NULL )
    {
        status = nx_ip_create( &IP_HANDLE(interface), (char*)"NetX IP", GET_IPV4_ADDRESS(ip_settings->ip_address), GET_IPV4_ADDRESS(ip_settings->netmask), &wiced_packet_pools[pool], DRIVER_FOR_IF( interface ), STACK_FOR_IF( interface ), IP_STACK_SIZE, 2 );
        nx_ip_gateway_address_set( &IP_HANDLE(interface), GET_IPV4_ADDRESS(ip_settings->gateway) );
    }
    else
    {
        status = nx_ip_create( &IP_HANDLE(interface), (char*)"NetX IP", IP_ADDRESS(0, 0, 0, 0), 0xFFFFF000UL, &wiced_packet_pools[pool], DRIVER_FOR_IF( interface ), STACK_FOR_IF( interface ), IP_STACK_SIZE, 2 );
    }

    if ( status != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to create IP\n" ) );
        return WICED_ERROR;
    }

    if ( wiced_ip_driver_notify( interface, WICED_TRUE ) != WICED_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to notify driver\n" ) );
        goto driver_not_notified_leave_wifi_and_delete_ip;
    }

    /* Enable ARP */
    if ( nx_arp_enable( &IP_HANDLE(interface), (void *) ARP_FOR_IF( interface ), ARP_CACHE_SIZE ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to enable ARP\n" ) );
        goto leave_wifi_and_delete_ip;
    }

    if ( nx_tcp_enable( &IP_HANDLE(interface) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to enable TCP\n" ) );
        goto leave_wifi_and_delete_ip;

    }

    if ( nx_udp_enable( &IP_HANDLE(interface) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to enable UDP\n" ) );
        goto leave_wifi_and_delete_ip;

    }

    if ( nx_icmp_enable( &IP_HANDLE(interface) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to enable ICMP\n" ) );
        goto leave_wifi_and_delete_ip;

    }

    if ( nx_igmp_enable( &IP_HANDLE(interface) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to enable IGMP\n" ) );
        goto leave_wifi_and_delete_ip;

    }

    if ( nx_ip_fragment_enable( &IP_HANDLE(interface) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR(("Failed to enable IP fragmentation\n"));
        goto leave_wifi_and_delete_ip;
    }

#ifdef ENABLE_NX_BSD_SOCKET
    if (bsd_initialize (&IP_HANDLE(interface), &wiced_packet_pools[0], bsd_socket_stack, 2048, 2) != 0 )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to initialise BSD \n" ) );
        goto leave_wifi_and_delete_ip;
    }
#endif


    /* Things need to happen in the following order, so that extra packet loss is not seen.
      * 1. disable buffering: this will allow DHCP traffic to go through in the next step
      * 2. Reinit DHCP
      * 3. Replay buffered traffic
      * This will produce less packet loss.  It allows DHCP process to finish and restore prior to sending up packets that may have been coming in to the DHCP address.
      * The side-effect of this algorithm is that out of order delivery of packets occurs.
      * Out of order delivery is supposed to be ok in tcp/ip networking; the TCP and IP protocols are built to handle it and apps that use UDP generally handle it as well.
      * So, trading off keeping packets in order for a lower loss rate seems like a good idea.
      */

    /* 1. disable buffering: this will allow DHCP traffic to go through in the next step */
    if ( WICED_DEEP_SLEEP_IS_ENABLED() )
    {
      wiced_deep_sleep_disable_packet_buffering( );
    }

    /* 2. Reinit DHCP */

    /* Obtain an IP address via DHCP if required Or Restore DHCP state if requested */
    if ( config == WICED_USE_EXTERNAL_DHCP_SERVER || config == WICED_USE_EXTERNAL_DHCP_SERVER_RESTORE )
    {
        WPRINT_NETWORK_INFO( ("Obtaining IPv4 address via DHCP\n") );

        if ( dhcp_client_init( interface, &wiced_packet_pools[0] ) != WICED_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ( "Failed to initialise DHCP client\n" ) );
            goto leave_wifi_and_delete_ip;
        }

        if ( config == WICED_USE_EXTERNAL_DHCP_SERVER_RESTORE )
        {
            if ( wiced_network_restore_dhcp_state_from_deep_sleep( interface ) != WICED_SUCCESS )
            {
                WPRINT_NETWORK_INFO( ( "Unable to restore dhcp state\n" ) );
                /* proceed without restore */
            }
        }
    }
    else if ( config == WICED_USE_INTERNAL_DHCP_SERVER )
    {
        /* Create the DHCP Server.  */
        while ( IP_HANDLE(interface).nx_ip_driver_link_up == NX_FALSE ) // This case happens after p2p moves from group negotiation to starting the group owner
        {
            host_rtos_delay_milliseconds(10);
        }

        if ( wiced_start_dhcp_server( &internal_dhcp_server, interface ) != WICED_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ( "Failed to initialise DHCP server\r\n" ) );
            goto leave_wifi_and_delete_ip;
        }
    }


    /* Check for address resolution and wait for our addresses to be ready */
    status = nx_ip_status_check( &IP_HANDLE(interface), NX_IP_ADDRESS_RESOLVED, (ULONG *) &status, WICED_DHCP_IP_ADDRESS_RESOLUTION_TIMEOUT );

    if ( status == NX_SUCCESS )
    {
        ULONG ip_address, network_mask;
        nx_ip_address_get( &IP_HANDLE(interface), &ip_address, &network_mask );
        WPRINT_NETWORK_INFO( ( "IPv4 network ready IP: %u.%u.%u.%u\n", (unsigned char) ( ( ip_address >> 24 ) & 0xff ), (unsigned char) ( ( ip_address >> 16 ) & 0xff ), (unsigned char) ( ( ip_address >> 8 ) & 0xff ), (unsigned char) ( ( ip_address >> 0 ) & 0xff ) ) );

        /* Register a handler for any address changes */
        status = nx_ip_address_change_notify( &IP_HANDLE(interface), ip_address_changed_handler, NX_NULL );
        if ( status != NX_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ( "Unable to register for IPv4 address change callback\n" ) );
            goto leave_wifi_and_delete_ip;
        }
    }
    else
    {
#ifdef AUTO_IP_ENABLED
        ULONG ip_address, network_mask;
        int   tries = 0;
#endif
        if ( DHCP_CLIENT_IS_INITIALISED( interface) )
        {
            dhcp_client_deinit( interface );
        }
#ifdef AUTO_IP_ENABLED
        WPRINT_NETWORK_INFO(("Unable to obtain IP address via DHCP. Perform AUTO_IP\r\n"));

        /* Try to get link-local address, in case dhcp is failing */
        status =  nx_auto_ip_create( &auto_ip_handle, (CHAR*)"AutoIP 0", &IP_HANDLE(interface), auto_ip_stack, 4096, AUTO_IP_PRIORITY );
        if ( status != NX_SUCCESS )
        {
            nx_auto_ip_delete( &auto_ip_handle );
            goto leave_wifi_and_delete_ip;
        }
        status =  nx_auto_ip_start( &auto_ip_handle, original_auto_ip_address );
        if ( status != NX_SUCCESS )
        {
            nx_auto_ip_delete( &auto_ip_handle );
            goto leave_wifi_and_delete_ip;
        }

        /* Wait for AUTO_IP address to be resolved.   */
        do
        {
            /* Call IP status check routine.   */
            status =  nx_ip_status_check( &IP_HANDLE(interface), NX_IP_ADDRESS_RESOLVED, (ULONG *) &status, WICED_AUTO_IP_ADDRESS_RESOLUTION_TIMEOUT );
        } while ( ( status != NX_SUCCESS ) && ( tries++ < 5 ) );

        if ( status != NX_SUCCESS )
        {
            nx_auto_ip_stop( &auto_ip_handle );
            nx_auto_ip_delete( &auto_ip_handle );
            goto leave_wifi_and_delete_ip;
        }

        nx_ip_address_get( &IP_HANDLE(interface), &ip_address, &network_mask );
        WPRINT_NETWORK_INFO( ( "IPv4  AUTO_IP network ready IP: %u.%u.%u.%u\n", (unsigned char) ( ( ip_address >> 24 ) & 0xff ), (unsigned char) ( ( ip_address >> 16 ) & 0xff ), (unsigned char) ( ( ip_address >> 8 ) & 0xff ), (unsigned char) ( ( ip_address >> 0 ) & 0xff ) ) );

        /* Register a handler for any address changes */
        status = nx_ip_address_change_notify( &IP_HANDLE(interface), ip_address_changed_handler, &auto_ip_handle );
        if ( status != NX_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ( "Unable to register for IPv4 address change callback\n" ) );
            goto leave_wifi_and_delete_ip;
        }

#else

        WPRINT_NETWORK_INFO(("Unable to obtain IP address via DHCP\r\n"));
        goto leave_wifi_and_delete_ip;
#endif
    }

    if ( config == WICED_USE_EXTERNAL_DHCP_SERVER )
    {
        UCHAR              dns_ip_string[DHCP_NUM_DNS_SERVERS * sizeof(ULONG)];
        UINT               size;
        wiced_ip_address_t address;
        UINT               i = 0;
        UINT               dns_address_count;
        UCHAR*             dns_address_ptr;

        /* Obtain the IP addresses of the DNS servers. */
        size = sizeof( dns_ip_string );
        if ( nx_dhcp_user_option_retrieve( &DHCP_HANDLE(interface), NX_DHCP_OPTION_DNS_SVR, dns_ip_string, &size ) == NX_SUCCESS )
        {
            /* Calculate the DNS Server address count.  */
            dns_address_count = size / sizeof(ULONG);

            /* Output the DNS Server addresses.  */
            for(i = 0; i < dns_address_count; i++)
            {
                /* Set the DNS address pointer.  */
                dns_address_ptr = (dns_ip_string + i * sizeof(ULONG));

                /* Add gateway DNS server */
                SET_IPV4_ADDRESS( address, nx_dhcp_user_option_convert( dns_address_ptr ) );
                dns_client_add_server_address( address );
            }
        }

        /* Add Google DNS server (8.8.8.8) */
        memset( dns_ip_string, 8, 4 );
        SET_IPV4_ADDRESS( address, nx_dhcp_user_option_convert( dns_ip_string ) );
        dns_client_add_server_address( address );
    }

    wiced_init_autoipv6( interface );

    CONFIG_SET_FOR_IF( interface, config );

    SET_IP_NETWORK_INITED(interface, WICED_TRUE);

    /* 3. Replay buffered traffic: send up stack now that IP addresses are configured  */
    if ( WICED_DEEP_SLEEP_IS_ENABLED() )
    {
        wiced_deep_sleep_set_networking_ready();
    }

    return WICED_SUCCESS;

leave_wifi_and_delete_ip:
    wiced_ip_driver_notify( interface, WICED_FALSE );
driver_not_notified_leave_wifi_and_delete_ip:
    if ( interface == WICED_STA_INTERFACE )
    {
        wiced_leave_ap( interface );
    }

    /* 3. Replay buffered traffic: the replay here may result in lost packets, which seems ok since IP addresses weren't obtained properly */
    if ( WICED_DEEP_SLEEP_IS_ENABLED() )
    {
        wiced_deep_sleep_set_networking_ready();
    }

    nx_ip_delete( &IP_HANDLE(interface));
    return WICED_ERROR;
}

wiced_result_t wiced_ip_down( wiced_interface_t interface )
{
    if ( IP_NETWORK_IS_INITED( interface ) == WICED_TRUE )
    {
        /* Cleanup DHCP & DNS */
        if ( CONFIG_GET_FOR_IF( interface ) == WICED_USE_INTERNAL_DHCP_SERVER )
        {
            wiced_stop_dhcp_server( &internal_dhcp_server );
        }
        else  /* External DHCP */
        {
            if ( DHCP_CLIENT_IS_INITIALISED( interface) )
            {
                dhcp_client_deinit( interface );
            }
            dns_client_remove_all_server_addresses( );
        }

#ifdef AUTO_IP_ENABLED
        if ( AUTO_IP_INITIALISED(auto_ip_handle) )
        {
            nx_auto_ip_get_address( &auto_ip_handle, &original_auto_ip_address );
            nx_auto_ip_stop( &auto_ip_handle );
            nx_auto_ip_delete( &auto_ip_handle );
        }
#endif /* WICED_AUTO_IP_ENABLE */

#ifdef ENABLE_NXD_BSD_SOCKET
        bsd_deinit();
#endif
        /* Tell driver interface went down */
        wiced_ip_driver_notify( interface, WICED_FALSE );

        /* Delete the network interface */
        if ( nx_ip_delete( &IP_HANDLE(interface) ) != NX_SUCCESS)
        {
            WPRINT_NETWORK_ERROR( ( "Could not delete IP instance\n" ) );
            return WICED_TCPIP_ERROR;
        }
        memset( &IP_HANDLE(interface), 0, sizeof(NX_IP));

        SET_IP_NETWORK_INITED(interface, WICED_FALSE);
    }

    return WICED_SUCCESS;
}

static void wiced_network_save_dhcp_state_for_deep_sleep( wiced_interface_t interface )
{
    UINT err;
    WPRINT_NETWORK_DEBUG(("%s\n", __FUNCTION__));

    if ( WICED_DEEP_SLEEP_IS_ENABLED() == WICED_TRUE )
    {
        err = nx_dhcp_client_get_record( &DHCP_HANDLE(interface), &DHCP_RECORD_HANDLE(interface) );

        if ( err == NX_SUCCESS )
        {
            DHCP_RECORD_HANDLE_VALID( interface ) = WICED_TRUE;
            WPRINT_NETWORK_DEBUG(("dhcp record saved\n"));
        }
        else
        {
            DHCP_RECORD_HANDLE_VALID( interface ) = WICED_FALSE;
            WPRINT_NETWORK_DEBUG(("dhcp record saved\n"));
        }
    }
    else
    {
        WPRINT_NETWORK_DEBUG(( "Not saving DHCP state; deep sleep disabled\n" ));
    }
}

static wiced_result_t wiced_network_restore_dhcp_state_from_deep_sleep( wiced_interface_t interface )
{
    wiced_result_t result = WICED_ERROR;

    if ( WICED_DEEP_SLEEP_IS_ENABLED() == WICED_FALSE )
    {
        WPRINT_NETWORK_ERROR(("Deep sleep not enabled for this platform; error restoring DHCP state\n"));
    }
    else if ( DHCP_RECORD_HANDLE_VALID( interface ) == WICED_TRUE )
    {
        /* Restore the DHCP record; use the ticks since deep sleep value to fill out the ticks since save value */
        UINT err = nx_dhcp_client_restore_record( &DHCP_HANDLE(interface), &DHCP_RECORD_HANDLE(interface), wiced_deep_sleep_ticks_since_enter( ) );

        /* used once, now invalid */
        DHCP_RECORD_HANDLE_VALID( interface ) = WICED_FALSE;

        if ( err == NX_SUCCESS )
        {
            result = WICED_SUCCESS;
            WPRINT_NETWORK_DEBUG(("dhcp record restored\n"));
        }
        else
        {
            WPRINT_NETWORK_DEBUG(("err %d on dhcp restore\n", err));
        }
    }
    return result;
}

wiced_result_t wiced_network_suspend( void )
{
    wiced_assert( "Network is already suspended", network_is_suspended == WICED_FALSE );

    if ( network_is_suspended == WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

#ifdef WICED_USE_ETHERNET_INTERFACE
    if( wiced_network_is_up( WICED_ETHERNET_INTERFACE ) )
    {
        if( wiced_network_suspend_layers( WICED_ETHERNET_INTERFACE ) != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }
#endif

    if( wiced_network_is_up( WICED_STA_INTERFACE ) )
    {
        if( wiced_network_suspend_layers( WICED_STA_INTERFACE ) != WICED_SUCCESS )
        {
            return WICED_ERROR;
        }
    }

    network_is_suspended = WICED_TRUE;

    return WICED_SUCCESS;
}

wiced_result_t wiced_network_resume( void )
{
    wiced_assert( "Network was not suspended previously", network_is_suspended == WICED_TRUE );

    /* Ensure network was previously suspended */
    if ( network_is_suspended != WICED_TRUE )
    {
        return WICED_SUCCESS;
    }

#ifdef WICED_USE_ETHERNET_INTERFACE
    if( wiced_network_is_up( WICED_ETHERNET_INTERFACE ) )
    {
        if( wiced_network_resume_layers( WICED_ETHERNET_INTERFACE ) == WICED_ERROR )
        {
            return WICED_ERROR;
        }
    }
#endif

    if( wiced_network_is_up( WICED_STA_INTERFACE ) )
    {
        if( wiced_network_resume_layers( WICED_STA_INTERFACE ) == WICED_ERROR )
        {
            return WICED_ERROR;
        }
    }

    network_is_suspended = WICED_FALSE;

    return WICED_SUCCESS;
}

void wiced_network_notify_link_up( wiced_interface_t interface )
{
    IP_HANDLE(interface).nx_ip_driver_link_up = NX_TRUE;
}

void wiced_network_notify_link_down( wiced_interface_t interface )
{
    IP_HANDLE(interface).nx_ip_driver_link_up = NX_FALSE;
}

static void wiced_call_link_down_callbacks( wiced_interface_t interface )
{
    int i;

    for ( i = 0; i < WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS; i++ )
    {
        if ( (LINK_DOWN_CALLBACKS_LIST( interface ))[i] != NULL )
        {
            (LINK_DOWN_CALLBACKS_LIST( interface ))[i]( );
        }
    }
}

static void wiced_call_link_up_callbacks( wiced_interface_t interface )
{
    int i;

    for ( i = 0; i < WICED_MAXIMUM_LINK_CALLBACK_SUBSCRIPTIONS; i++ )
    {
        if ( (LINK_UP_CALLBACKS_LIST( interface ))[i] != NULL )
        {
            (LINK_UP_CALLBACKS_LIST( interface ))[i]();
        }
    }
}

wiced_result_t wiced_wireless_link_down_handler( void* arg )
{
    const wiced_interface_t interface = WICED_STA_INTERFACE;
    wiced_result_t          result    = WICED_SUCCESS;

    UNUSED_PARAMETER( arg );

    WPRINT_NETWORK_DEBUG( ("Wireless link DOWN!\n\r") );

    if ( DHCP_CLIENT_IS_INITIALISED( interface ) )
    {
        UINT res = nx_dhcp_stop( &DHCP_HANDLE( interface ) );

        if ( ( res != NX_SUCCESS ) && ( res != NX_DHCP_NOT_STARTED ) )
        {
            WPRINT_NETWORK_ERROR( ("Stopping DHCP failed!\n\r") );
            result = WICED_ERROR;
        }
    }

    if ( nx_arp_dynamic_entries_invalidate( &IP_HANDLE(  interface  ) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ("Clearing ARP cache failed!\n\r") );
        result = WICED_ERROR;
    }

    /* Inform all subscribers about an event */
    wiced_call_link_down_callbacks( interface );

    /* Kick the radio chip if it's in power save mode in case the link down event is due to missing beacons. Setting the chip to the same power save mode is sufficient. */
    wifi_powersave_mode = wiced_wifi_get_powersave_mode();
    if ( wifi_powersave_mode == PM1_POWERSAVE_MODE )
    {
        wiced_wifi_enable_powersave();
    }
    else if ( wifi_powersave_mode == PM2_POWERSAVE_MODE )
    {
        wifi_return_to_sleep_delay = wiced_wifi_get_return_to_sleep_delay();
        wiced_wifi_enable_powersave_with_throughput( wifi_return_to_sleep_delay );
    }

    return result;
}

#ifdef WICED_USE_ETHERNET_INTERFACE
wiced_result_t wiced_ethernet_link_down_handler( void )
{
    const wiced_interface_t interface = WICED_ETHERNET_INTERFACE;
    wiced_result_t          result    = WICED_SUCCESS;

    WPRINT_NETWORK_DEBUG( ("Ethernet link DOWN!\n\r") );

    if ( DHCP_CLIENT_IS_INITIALISED( interface ) )
    {
        UINT res = nx_dhcp_stop( &DHCP_HANDLE( interface ) );

        if ( ( res != NX_SUCCESS ) && ( res != NX_DHCP_NOT_STARTED ) )
        {
            WPRINT_NETWORK_ERROR( ("Stopping DHCP failed!\n\r") );
            result = WICED_ERROR;
        }
    }
#ifdef AUTO_IP_ENABLED
    else if ( AUTO_IP_INITIALISED(auto_ip_handle) )
    {
        nx_auto_ip_stop( &auto_ip_handle );
    }
#endif /* AUTO_IP_ENABLED */

    if ( nx_arp_dynamic_entries_invalidate( &IP_HANDLE(  interface  ) ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ("Clearing ARP cache failed!\n\r") );
        result = WICED_ERROR;
    }

    /* Inform all subscribers about an event */
    wiced_call_link_down_callbacks( interface );

    return result;
}
#endif

wiced_result_t wiced_wireless_link_up_handler( void* arg )
{
    const wiced_interface_t interface = WICED_STA_INTERFACE;
    wiced_result_t          result    = WICED_SUCCESS;

    UNUSED_PARAMETER( arg );

    WPRINT_NETWORK_DEBUG( ("Wireless link UP!\n") );

    if ( DHCP_CLIENT_IS_INITIALISED( interface ) )
    {
        UINT res;

        /* Save the current power save state */
        wifi_powersave_mode = wiced_wifi_get_powersave_mode();
        wifi_return_to_sleep_delay = wiced_wifi_get_return_to_sleep_delay();

        /* Disable power save for the DHCP exchange */
        if ( wifi_powersave_mode != NO_POWERSAVE_MODE )
        {
            wiced_wifi_disable_powersave( );
        }

        res = nx_dhcp_start( &DHCP_HANDLE( interface ) );
        if ( res == NX_DHCP_ALREADY_STARTED )
        {
            nx_dhcp_force_renew( &DHCP_HANDLE( interface ) );
        }
        else if ( res != NX_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ( "Failed to initiate DHCP transaction\n" ) );
            result = WICED_ERROR;
        }

        /* Wait a little to allow DHCP a chance to complete, but we can't block here */
        host_rtos_delay_milliseconds(10);

        if ( wifi_powersave_mode == PM1_POWERSAVE_MODE )
        {
            wiced_wifi_enable_powersave();
        }
        else if ( wifi_powersave_mode == PM2_POWERSAVE_MODE )
        {
            wiced_wifi_enable_powersave_with_throughput( wifi_return_to_sleep_delay );
        }
    }

    /* Inform all subscribers about an event */
    wiced_call_link_up_callbacks( interface );

    return result;
}

#ifdef WICED_USE_ETHERNET_INTERFACE
wiced_result_t wiced_ethernet_link_up_handler( void )
{
    const wiced_interface_t interface = WICED_ETHERNET_INTERFACE;
    wiced_result_t          result    = WICED_SUCCESS;

    WPRINT_NETWORK_DEBUG( ("Ethernet link UP!\n\r") );

    if ( DHCP_CLIENT_IS_INITIALISED( interface ) )
    {
        UINT res;

        res = nx_dhcp_start( &DHCP_HANDLE( interface ) );
        if ( res == NX_DHCP_ALREADY_STARTED )
        {
            nx_dhcp_force_renew( &DHCP_HANDLE( interface ) );
        }
        else if ( res != NX_SUCCESS )
        {
            WPRINT_NETWORK_ERROR( ( "Failed to initiate DHCP transaction\n" ) );
            result = WICED_ERROR;
        }
    }
#ifdef AUTO_IP_ENABLED
    else if ( AUTO_IP_INITIALISED(auto_ip_handle) )
    {
        ULONG ip_address;

        nx_auto_ip_get_address(&auto_ip_handle, &ip_address);
        nx_auto_ip_start(&auto_ip_handle, ip_address);
    }
#endif /* AUTO_IP_ENABLED */

    /* Inform all subscribers about an event */
    wiced_call_link_up_callbacks( interface );

    return result;
}
#endif

wiced_result_t wiced_wireless_link_renew_handler( void* arg )
{
    wiced_result_t result = WICED_SUCCESS;
    wiced_ip_address_t ipv4_address;

    UNUSED_PARAMETER( arg );

    wiced_ip_get_gateway_address( WICED_STA_INTERFACE, &ipv4_address );

    if ( nx_arp_dynamic_entry_set( &IP_HANDLE( WICED_STA_INTERFACE ), IP_ADDRESS((unsigned int)((GET_IPV4_ADDRESS(ipv4_address) >> 24) & 0xFF),
                        (unsigned int)((GET_IPV4_ADDRESS(ipv4_address) >> 16) & 0xFF),
                        (unsigned int)((GET_IPV4_ADDRESS(ipv4_address) >> 8) & 0xFF),
                        (unsigned int)((GET_IPV4_ADDRESS(ipv4_address) >> 0) & 0xFF)), 0x0, 0x0 ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ("Dynamically changing the ARP entry failed!\n\r") );
        result = WICED_ERROR;

    }
    /* Try do a DHCP renew. This may not be successful if we've had a link down event previously */
    if ( DHCP_CLIENT_IS_INITIALISED( WICED_STA_INTERFACE ) )
    {
        nx_dhcp_force_renew( &DHCP_HANDLE( WICED_STA_INTERFACE ) );
    }

    return result;
}

wiced_result_t wiced_ip_register_address_change_callback( wiced_ip_address_change_callback_t callback, void* arg )
{
    uint8_t i;
    for ( i = 0; i < MAXIMUM_IP_ADDRESS_CHANGE_CALLBACKS; i++ )
    {
        if ( wiced_ip_address_change_callbacks[i].callback == NULL )
        {
            wiced_ip_address_change_callbacks[i].callback = callback;
            wiced_ip_address_change_callbacks[i].arg = arg;
            return WICED_SUCCESS;
        }
    }

    WPRINT_NETWORK_ERROR( ( "Out of callback storage space\n" ) );

    return WICED_ERROR;
}

wiced_result_t wiced_ip_deregister_address_change_callback( wiced_ip_address_change_callback_t callback )
{
    uint8_t i;
    for ( i = 0; i < MAXIMUM_IP_ADDRESS_CHANGE_CALLBACKS; i++ )
    {
        if ( wiced_ip_address_change_callbacks[i].callback == callback )
        {
            memset( &wiced_ip_address_change_callbacks[i], 0, sizeof( wiced_ip_address_change_callback_t ) );
            return WICED_SUCCESS;
        }
    }

    WPRINT_NETWORK_ERROR( ( "Unable to find callback to deregister\n" ) );

    return WICED_ERROR;
}

wiced_bool_t wiced_ip_is_any_pending_packets( wiced_interface_t interface )
{
    NX_IP* ip_handle;
    ULONG  i;

    if ( IP_NETWORK_IS_INITED( interface ) == WICED_FALSE )
    {
        return WICED_FALSE;
    }

    ip_handle = &IP_HANDLE(interface);

    for ( i = 0; i < ip_handle->nx_ip_arp_total_entries; i++ )
    {
        if ( ip_handle->nx_ip_arp_cache_memory[i].nx_arp_packets_waiting )
        {
            return WICED_TRUE;
        }
    }

#ifndef NX_DISABLE_FRAGMENTATION
    if ( ip_handle->nx_ip_received_fragment_head != NULL )
    {
        return WICED_TRUE;
    }
#endif

    return WICED_FALSE;
}

wiced_result_t wiced_ip_get_clients_ip_address_list( wiced_interface_t interface, void* ip_address_list )
{
    if ( CONFIG_GET_FOR_IF( interface ) != WICED_USE_INTERNAL_DHCP_SERVER )
    {
        return WICED_ERROR;
    }

    return wiced_get_clients_ip_address_list_dhcp_server( &internal_dhcp_server, ip_address_list );
}

/******************************************************
 *            Static Function Definitions
 ******************************************************/

static wiced_result_t dhcp_client_init( wiced_interface_t interface, NX_PACKET_POOL* packet_pool )
{
    wiced_result_t    result;
    wiced_hostname_t* dhcp_hostname = DHCP_CLIENT_HOSTNAME(interface);
    NX_IP*            ip_handle     = &IP_HANDLE(interface);
    NX_DHCP*          dhcp_handle   = &DHCP_HANDLE(interface);

    /* get hostname */
    result = wiced_network_get_hostname( dhcp_hostname );
    if ( result != WICED_SUCCESS )
    {
        return result;
    }
    WPRINT_NETWORK_INFO( ("DHCP CLIENT hostname %s\n", dhcp_hostname->value) );

    /* clear DHCP info to start */
    memset( dhcp_handle, 0, sizeof( *dhcp_handle ) );

    /* Create the DHCP instance. */
    if ( nx_dhcp_create( dhcp_handle, ip_handle, dhcp_hostname->value ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to Create DHCP thread\n" ) );
        return WICED_ERROR;
    }

    // nx_dhcp_packet_pool_set( dhcp_handle, packet_pool ); /* Only available in NetX-Duo */
    UNUSED_PARAMETER( packet_pool );

    nx_dhcp_request_client_ip(dhcp_handle, ip_handle->nx_ip_address, NX_TRUE);

    /* Start DHCP. */
    if ( nx_dhcp_start( dhcp_handle ) != NX_SUCCESS )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to initiate DHCP transaction\n" ) );
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t dhcp_client_deinit( wiced_interface_t interface )
{
    NX_DHCP* dhcp_handle = &DHCP_HANDLE(interface);
    UINT     res;

    res = nx_dhcp_stop( dhcp_handle );
    if ( ( res != NX_SUCCESS ) && ( res != NX_DHCP_NOT_STARTED ) )
    {
        WPRINT_NETWORK_ERROR( ( "Failed to stop DHCP client\n" ) );
    }

    /* The following function is called because NetX creates its own packet pool, unlike NetX Duo which makes use of an existing pool. */
    nx_dhcp_delete( dhcp_handle );

    /* Clear the dhcp handle structure and name array */
    memset( dhcp_handle->nx_dhcp_name, 0x00, ( HOSTNAME_SIZE + 1 ) );
    memset( dhcp_handle, 0, sizeof( *dhcp_handle ) );

    return WICED_SUCCESS;
}

static void ip_address_changed_handler( NX_IP *ip_ptr, VOID *additional_info )
{
    uint8_t i;

    UNUSED_PARAMETER( ip_ptr );
    UNUSED_PARAMETER( additional_info );

#ifdef AUTO_IP_ENABLED
    {
        ULONG ip_address;
        ULONG network_mask;
        NX_AUTO_IP* handle =  (NX_AUTO_IP*) additional_info;

        nx_ip_address_get(ip_ptr, &ip_address, &network_mask);
        if ( ip_address == 0 && ( ( original_auto_ip_address & 0xFFFF0000UL ) == IP_ADDRESS(169, 254, 0, 0) ) )
        {
            nx_auto_ip_get_address(handle, &ip_address);
            nx_auto_ip_start(handle, ip_address);
        }
    }
#endif /* AUTO_IP_ENABLED */

    for ( i = 0; i < MAXIMUM_IP_ADDRESS_CHANGE_CALLBACKS; i++ )
    {
        if ( wiced_ip_address_change_callbacks[i].callback != NULL )
        {
            ( *wiced_ip_address_change_callbacks[i].callback )( wiced_ip_address_change_callbacks[i].arg );
        }
    }
}

static wiced_bool_t tcp_sockets_are_closed( wiced_interface_t interface )
{
    ULONG tcp_connections = 0;
    UINT  result;

    result = nx_tcp_info_get( &IP_HANDLE(interface), NX_NULL, NX_NULL, NX_NULL, NX_NULL, NX_NULL, NX_NULL, NX_NULL, &tcp_connections, NX_NULL, NX_NULL, NX_NULL);
    if ( result == NX_SUCCESS && tcp_connections == 0)
    {
        return WICED_TRUE;
    }
    return WICED_FALSE;
}

static ULONG wiced_network_init_packet_pool( NX_PACKET_POOL* pool, const char* pool_name, uint8_t* memory_pointer, uint32_t memory_size, uint16_t payload_size )
{
    void* memory_pointer_aligned = PLATFORM_L1_CACHE_PTR_ROUND_UP( memory_pointer );
    uint32_t memory_size_aligned = PLATFORM_L1_CACHE_ROUND_DOWN  ( memory_size - ( (uint32_t)memory_pointer_aligned - (uint32_t)memory_pointer ) );

    wiced_static_assert(packet_header_not_cache_aligned, sizeof(NX_PACKET) == PLATFORM_L1_CACHE_ROUND_UP(sizeof(NX_PACKET)));

    return nx_packet_pool_create( pool, (CHAR*)pool_name, payload_size, memory_pointer_aligned, memory_size_aligned );
}

static wiced_result_t wiced_network_suspend_layers( wiced_interface_t interface )
{
    /* save dhcp state, in case needed for restore after deep sleep */
    wiced_network_save_dhcp_state_for_deep_sleep( interface );

    /* Ensure all current TCP sockets are closed */
    if ( tcp_sockets_are_closed( interface ) != WICED_TRUE )
    {
        return WICED_ERROR;
    }

    if ( DHCP_CLIENT_IS_INITIALISED( interface) )
    {
        if (!(DHCP_HANDLE(interface).nx_dhcp_interface_record[0].nx_dhcp_record_valid) ||
             (DHCP_HANDLE(interface).nx_dhcp_interface_record[0].nx_dhcp_state != NX_DHCP_STATE_BOUND))
        {
            return WICED_ERROR;
        }
    }

    /* Suspend IP layer. This will deactivate IP layer periodic timers */
    if ( nx_ip_suspend( &IP_HANDLE(interface) ) != TX_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Suspend DHCP client */
    if ( DHCP_CLIENT_IS_INITIALISED( interface) )
    {
        if ( nx_dhcp_suspend( &DHCP_HANDLE(interface) ) != TX_SUCCESS )
        {
            return WICED_ERROR;
        }
    }

    /* TODO: Suspend IGMP */

    /* Suspend TCP. This will deactivate tcp fast periodic timer processing */
    if ( nx_tcp_suspend( &IP_HANDLE(interface) ) != NX_SUCCESS )
    {
        return WICED_ERROR;
    }

    /* Remember when the network was suspended, it will be used to update the DHCP lease time */
    if ( wiced_time_get_time( &network_suspend_start_time[WICED_TO_WWD_INTERFACE(interface)] ) != WICED_SUCCESS )
    {
        return WICED_ERROR;
    }

    return WICED_SUCCESS;
}

static wiced_result_t wiced_network_resume_layers( wiced_interface_t interface )
{
    uint32_t number_of_ticks_network_was_suspended;

    /* Resume DHCP */
    if ( DHCP_CLIENT_IS_INITIALISED( interface) )
    {
        if ( nx_dhcp_resume( &DHCP_HANDLE(interface) ) != TX_SUCCESS )
        {
            wiced_assert("WICED can't resume DHCP client", 0 != 0 );
            return WICED_ERROR;
        }
    }

    /* Resume IP timers */
    if ( nx_ip_resume( &IP_HANDLE(interface)) != TX_SUCCESS )
    {
        wiced_assert("WICED can't resume IP timers", 0 != 0 );
        return WICED_ERROR;
    }

    /* TODO: Resume IGMP */

    /* Resume TCP */
    if ( nx_tcp_resume( &IP_HANDLE(interface)) != NX_SUCCESS )
    {
        wiced_assert("WICED can't resume TCP timers", 0 != 0 );
        return WICED_ERROR;
    }

    /* Calculate the length of time we were suspended */
    if ( wiced_time_get_time( &network_suspend_end_time[WICED_TO_WWD_INTERFACE(interface)] ) != WICED_SUCCESS )
    {
        wiced_assert("Error getting system time", 0 != 0 );
        return WICED_ERROR;
    }
    number_of_ticks_network_was_suspended = network_suspend_end_time[WICED_TO_WWD_INTERFACE(interface)] - network_suspend_start_time[WICED_TO_WWD_INTERFACE(interface)];

    /* Update DHCP time related variables */
    if ( DHCP_CLIENT_IS_INITIALISED( interface) )
    {
        if ( nx_dhcp_client_update_time_remaining( &DHCP_HANDLE(interface), number_of_ticks_network_was_suspended ) != NX_SUCCESS )
        {
            wiced_assert( "Error updating DHCP client time", 0 != 0 );
            return WICED_ERROR;
        }
    }

    return WICED_SUCCESS;
}

static wiced_result_t wiced_ip_driver_notify( wiced_interface_t interface, wiced_bool_t up )
{
    wiced_result_t result = WICED_SUCCESS;

#ifdef WICED_USE_ETHERNET_INTERFACE
    if ( interface == WICED_ETHERNET_INTERFACE )
    {
        if ( up )
        {
            result = ( platform_ethernet_start( ) == PLATFORM_SUCCESS ) ? WICED_SUCCESS : WICED_ERROR;
        }
        else
        {
            result = ( platform_ethernet_stop( ) == PLATFORM_SUCCESS ) ? WICED_SUCCESS : WICED_ERROR;
        }
    }
#else
    UNUSED_PARAMETER( interface );
    UNUSED_PARAMETER( up );
#endif

    return result;
}

wiced_result_t wiced_init_autoipv6( wiced_interface_t interface )
{
    UNUSED_PARAMETER( interface );

    return WICED_ERROR;
}

