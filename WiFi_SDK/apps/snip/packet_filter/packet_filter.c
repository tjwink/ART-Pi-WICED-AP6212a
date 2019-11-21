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
 *
 * Ethernet Packet Filter Application
 *
 * This application demonstrates usage of WLAN packet filters combined with monitor mode.
 *
 * Packet filters allow an application to specify exactly which kind of
 * packets it wants by setting templates that all incoming packets will
 * be matched against.
 *
 * Each filter has an option that determines whether matching packets are passed
 * up to the app processor or dropped, defining inclusive or exclusive filters.
 *
 * Filters can also be used without monitor mode (ie without recieving the packets) when only
 * the number of matches to a given filter is needed, without the actual data.
 *
 * Note that if no filters are installed and monitor mode is on, all incoming packets wll be
 * sent to the callback.  But using filters to cut down the number of packets captured is
 * recomended in order to cut down on system resource usage,
 *
 * Some packet parsing is done here to show usage but is not meant to be definitive nor
 * exhaustive.
 *
 * Both IEEE802.11 layer 2 packets (802.11) and layer 3 packets (IP, ARP, etc) filtering
 * is demonstrated here.
 *
 * Features demonstrated
 *  - Defining filters.
 *  - Adding, Enabling/Disabling, Listing & removing filters
 *  - Setting up monitor mode with callbacks.
 *  - Recieving packets from the filters.
 *  - Demonstrates various 802.11 and IP filters.
 *  - Examining stats from filters.
 *  - Parsing layer 2 802.11 packets.
 *  - Parsing layer 3 IP packets.
 *
 *
 * Application Instructions
 *   1. Modify the CLIENT_AP_SSID/CLIENT_AP_PASSPHRASE Wi-Fi credentials
 *      in the wifi_config_dct.h header file to match your Wi-Fi access point
 *   2. Connect a PC terminal to the serial port of the WICED Eval board,
 *      then build and download the application as described in the WICED
 *      Quick Start Guide
 *
 *   Once the device has joined a Wi-Fi network and received an IP address, it will enable
 *   monitor mode and start 'sniffing' for packets that match the filters and sending those
 *   packets upstream to the host processor.
 *
 *
 * Discussion
 *   When a packet filter(s) is installed, incoming packets received by the WLAN chip are
 *   run through the pre-installed filter(s). Filter criteria are added using the 'add'
 *   API function. If the WLAN chip receives a packet that matches one of the currently
 *   installed filters, the host MCU is notified, and the packet is forwarded to the MCU.
 *   Packets that do not match any of the installed filters are dropped by the WLAN chip.
 *
 *   For full documentation and a list of packet filter API functions, please read the
 *   packet filter API documentation contained in the file <WICED-SDK>/doc/API.html
 *
 * References
 *   http://en.wikipedia.org/wiki/EtherType
 *
 */

#include "wiced.h"
#include "network/wwd_buffer_interface.h"
#include <proto/bcmip.h>
#include <proto/802.11.h>

/******************************************************
 *                      Macros
 ******************************************************/

/******************************************************
 *                    Constants
 ******************************************************/
#define MAX_COUNT                       8
#define MAX_MASK_PATTERN_SIZE         0xFF

#define MACDBG "%02x:%02x:%02x:%02x:%02x:%02x"
#define MAC2STRDBG(ea) (ea)[0], (ea)[1], (ea)[2], (ea)[3], (ea)[4], (ea)[5]
#define MACADDR_SZ 6
/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

struct workspace_t {
    wiced_semaphore_t sniff_complete;
} mywork;
struct workspace_t *workspace = &mywork;

/******************************************************
 *               Static Function Declarations
 ******************************************************/

static wiced_result_t print_packet_filter_list ( void );
static wiced_result_t print_packet_filter_stats( uint8_t filter_id );
static void pktfilter_enable(char *filter_list, unsigned int len);
static void pktfilter_disable(char *filter_list, unsigned int len);
static void pktfilter_stats(char *filter_list, unsigned int len);
static void pktfilter_remove(char *filter_list, unsigned int len);

static void raw_packet_callback( wiced_buffer_t buffer, wwd_interface_t interface );
extern void host_buffer_release(wiced_buffer_t buffer, wwd_buffer_dir_t direction );
extern uint8_t* host_buffer_get_current_piece_data_pointer(wiced_buffer_t buffer );

/******************************************************
 *               Variable Definitions
 ******************************************************/

/*
 * Example 1 : Filter for forwarding packets with EtherType[1] = 0x08xx
 *             ie. IPv4 packets (0x0800), ARP packets (0x0806), Wake-on-LAN packets (0x0842) etc.
 *                 IPV6 packets (0x86DD)
 *   - offset       = 12 (EtherType field)
 *   - bitmask size = 2 bytes
 *   - bitmask      = 0xff00 (mask out lower byte, don't want to match on this byte)
 *   - pattern      = 0x0800
 *   - rule         = positive matching
 */

static const wiced_packet_filter_t filter_ethertype_pattern =
{
    .id           = 1,
    .rule         = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .offset       = FILTER_OFFSET_ETH_HEADER_ETHERTYPE,
    .mask_size    = 2,
    .mask         = (uint8_t*)"\xff\x00",
    .pattern      = (uint8_t*)"\x08\x00",   /* llc.type == 0x0800  */
};

/*
 * Example 2 : Filter definition for packets to a specific IP address
 *   - offset       = 30          (offset relative to the start of the packet)
 *   - bitmask size = 4 bytes
 *   - bitmask      = 0xffffffff  (don't mask out any bits)
 *   - pattern      =
 *   - rule         = positive matching
 */
static uint32_t my_ipv4_address;
static const uint8_t ip_mask[]  = {0xFF, 0xFF, 0xFF, 0xFF};

static const wiced_packet_filter_t filter_specific_ip_address_pattern =
{
    .id           = 2,
    .rule         = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .offset       = FILTER_OFFSET_IPV4_HEADER_DESTINATION_ADDR,
    .mask_size    = 4,
    .mask         = (uint8_t*)ip_mask,             /* Network byte order updated */
    .pattern      = (uint8_t*)&my_ipv4_address,    /* at run-time by the app     */
};

static const uint8_t macaddr_mask[]         = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static wiced_mac_t my_bssid;                /* bssid == mac address of AP */

#ifdef FILTER_ON_A1_SETTINGS
/* Example: Filter on A1 802.11 Address */
static const wiced_packet_filter_t a1_settings =
{
    .id           = 10,
    .rule         = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .offset       = 4,
    .mask_size    = 6,
    .mask         = (uint8_t*)macaddr_mask,
    .pattern      = (uint8_t*)my_bssid.octet,
};
#endif /* #ifdef FILTER_ON_A1_SETTINGS */

/* Example: Filter on A3 802.11 Address */
static const wiced_packet_filter_t a3_settings =
{
    .id           = 11,
    .rule         = WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING,
    .offset       = 16,
    .mask_size    = 6,
    .mask         = (uint8_t*)macaddr_mask,
    .pattern      = (uint8_t*)my_bssid.octet,
};

/* List of filters are using. */
char filter_list[] = {1, 2, 11};

/******************************************************
 *               Function Definitions
 ******************************************************/
void application_start( )
{
    wiced_ip_address_t ip_address;

    wiced_init( );

    /* Join the default network */
    wiced_network_up( WICED_STA_INTERFACE, WICED_USE_EXTERNAL_DHCP_SERVER, NULL );

    /* Get our IP4 address so we can filter on it */
    wiced_ip_get_ipv4_address( WICED_STA_INTERFACE, &ip_address );
    my_ipv4_address = htonl( ip_address.ip.v4 );

    /* Get our BSSID so we can filter on it */
    wwd_wifi_get_bssid(&my_bssid);

    /* Filters can either drop matching packets or
     * forward them up to the host.  Lets forward up to host.
     */
    wiced_wifi_set_packet_filter_mode( WICED_PACKET_FILTER_MODE_FORWARD );

    /* Add filters */
    wiced_wifi_add_packet_filter(&filter_ethertype_pattern);
    wiced_wifi_add_packet_filter(&filter_specific_ip_address_pattern);
    wiced_wifi_add_packet_filter( &a3_settings );

    /* Register callback for packets that match filters */
    wwd_wifi_set_raw_packet_processor( raw_packet_callback );

    memset(&workspace->sniff_complete, 0, sizeof(workspace->sniff_complete));
    wiced_rtos_init_semaphore( &workspace->sniff_complete );

    /* Enable Filters */
    pktfilter_enable(filter_list, sizeof(filter_list));

    /* Display current filter settings */
    print_packet_filter_list();

    /* Demonstrate filters are all zeros to start. */
    pktfilter_stats(filter_list, sizeof(filter_list));

    /* Enable monitor mode so we will recieve the packets */
    wwd_wifi_enable_monitor_mode( );

    /* Collect packets for 5 seconds. */
    wiced_rtos_get_semaphore( &workspace->sniff_complete, 5 * 1000 );

    /* Done sniffing.  Disable monitor mode */
    wwd_wifi_disable_monitor_mode( );

    /* Show number of hits.  Note that even without monitor mode the stats will still show number of matches. */
    pktfilter_stats(filter_list, sizeof(filter_list));

    /* Deregister our callback routine */
    wwd_wifi_set_raw_packet_processor( NULL );

    /* Mission Complete, disable & uninstall filters */
    pktfilter_disable(filter_list, sizeof(filter_list));
    pktfilter_remove(filter_list, sizeof(filter_list));

    wiced_rtos_deinit_semaphore( &workspace->sniff_complete );
}

/* Query and print the filters */
static wiced_result_t print_packet_filter_list( void )
{
    wiced_result_t          result;
    wiced_packet_filter_t   filter_list[MAX_COUNT];
    uint32_t                filter_count = 0;
    uint32_t                size_out = 0;
    uint32_t                a;
    uint8_t                 mask_array[MAX_MASK_PATTERN_SIZE];
    uint8_t                 pattern_array[MAX_MASK_PATTERN_SIZE];
    uint8_t                *mask    = (uint8_t *)&mask_array;
    uint8_t                *pattern = (uint8_t *)&pattern_array;

    result = wiced_wifi_get_packet_filters ( MAX_COUNT, 0, filter_list, &filter_count );

    if ( result != WICED_SUCCESS )
    {
        WPRINT_APP_INFO(("get_packet_filters failed\n"));
        return result;
    }

    WPRINT_APP_INFO(("\nFilter List:\n"));
    for ( a = 0; a < filter_count; a++ )
    {
        uint16_t b;

        WPRINT_APP_INFO( ("Filter ID: %u\n", (unsigned int)filter_list[a].id));
        WPRINT_APP_INFO(("     %s, Offset %u  Len %u\r\n",
            (filter_list[a].rule == WICED_PACKET_FILTER_RULE_POSITIVE_MATCHING ) ? "Up to Host" : "Drop",
            (unsigned int)filter_list[a].offset,
            (unsigned int)filter_list[a].mask_size));

        wiced_wifi_get_packet_filter_mask_and_pattern (filter_list[a].id, filter_list[a].mask_size, mask, pattern, &size_out);

        WPRINT_APP_INFO( ("     Mask: ") );
        b = filter_list[a].mask_size;
        while ( b > 0 ) {
            WPRINT_APP_INFO( ("%02x ", *mask) );
            mask++;
            b--;
        }

        WPRINT_APP_INFO( ("\r\n") );
        WPRINT_APP_INFO( ("     Value: ") );
        b = filter_list[a].mask_size;
        while ( b > 0 ) {
            WPRINT_APP_INFO( ("%02x ", *pattern) );
            pattern++;
            b--;
        }
        WPRINT_APP_INFO( ("\r\n") );
    }
    return result;
}

/* Query and print stats from each filter */
static wiced_result_t print_packet_filter_stats( uint8_t filter_id )
{
    wiced_result_t              status;
    wiced_packet_filter_stats_t stats;

    status = wiced_wifi_get_packet_filter_stats( filter_id, &stats );

    if ( status == WICED_TIMEOUT )
    {
        WPRINT_APP_INFO(( "Timeout: Getting Packet Filter Statistics\r\n" ));
        return status;
    }
    WPRINT_APP_INFO((" %u:   %u \t%u \t%u\n",
        (unsigned int)filter_id, (unsigned int)stats.num_pkts_matched,
        (unsigned int)stats.num_pkts_forwarded, (unsigned int)stats.num_pkts_discarded));

    return status;
}

static void
dump_ipv4(struct ipv4_hdr *v4)
{
    char prot_buf[16];

    switch(v4->prot) {
    case 1: sprintf(prot_buf, "ICMP"); break;
    case 2: sprintf(prot_buf, "IGMP"); break;
    case 4: sprintf(prot_buf, "IPv4"); break;
    case 6: sprintf(prot_buf, "TCP"); break;
    case 17: sprintf(prot_buf, "UDP"); break;
    case 41: sprintf(prot_buf,"IPv6"); break;
    case 121: sprintf(prot_buf, "SMP"); break;
    default:
        sprintf(prot_buf, "Prot %d", v4->prot);
        break;
    }
    WPRINT_APP_INFO(("%s ", prot_buf));
    WPRINT_APP_INFO( ( " Src %u.%u.%u.%u",
                            v4->src_ip[0], v4->src_ip[1], v4->src_ip[2], v4->src_ip[3]));
    WPRINT_APP_INFO( ( " Dest %u.%u.%u.%u",
                            v4->dst_ip[0], v4->dst_ip[1], v4->dst_ip[2], v4->dst_ip[3]));

    WPRINT_APP_INFO((" Ver %d IPv4 Hdr_len %02d TOS %02x Tot len %d IP.ID %04u Frag %u TTL %02u\n",
        (v4->version_ihl >> 4) & 0xf,  (v4->version_ihl & 0xf) * 4, v4->tos, ntohs(v4->tot_len), ntohs(v4->id), v4->frag, v4->ttl));
}

char *type_mng_subtypes[] = {
/* FC_SUBTYPE_ASSOC_REQ            0    */ "Assoc Req",
/* FC_SUBTYPE_ASSOC_RESP           1    */ "Assoc Resp",
/* FC_SUBTYPE_REASSOC_REQ          2    */ "Reassoc Req",
/* FC_SUBTYPE_REASSOC_RESP         3    */ "Reassoc Resp",
/* FC_SUBTYPE_PROBE_REQ            4    */ "Probe Req",
/* FC_SUBTYPE_PROBE_RESP           5    */ "Probe Resp",
/* EMPTY_6                              */ "6 Empty",
/* EMPTY_7                              */ "7 Empty",
/* FC_SUBTYPE_BEACON               8    */ "Beacon",
/* FC_SUBTYPE_ATIM                 9    */ "ATIM",
/* FC_SUBTYPE_DISASSOC             10   */ "Disassoc",
/* FC_SUBTYPE_AUTH                 11   */ "Auth",
/* FC_SUBTYPE_DEAUTH               12   */ "DeAuth",
/* FC_SUBTYPE_ACTION               13   */ "Action"
};

char *type_data_subtypes[] = {
/* FC_SUBTYPE_DATA                  0   */ "Data",
/* FC_SUBTYPE_DATA_CF_ACK           1   */ "Data + CF-ACK",
/* FC_SUBTYPE_DATA_CF_POLL          2   */ "Data + CF-Poll",
/* FC_SUBTYPE_DATA_CF_ACK_POLL      3   */ "Data + CF-Ack + CF-Poll",
/* FC_SUBTYPE_NULL                  4   */ "Null",
/* FC_SUBTYPE_CF_ACK                5   */ "CF-Ack",
/* FC_SUBTYPE_CF_POLL               6   */ "CF-Poll",
/* FC_SUBTYPE_CF_ACK_POLL           7   */ "CF-Ack + CF-Poll",
/* FC_SUBTYPE_QOS_DATA              8   */ "QoS Data",
/* FC_SUBTYPE_QOS_DATA_CF_ACK       9   */ "QoS Data + CF-Ack",
/* FC_SUBTYPE_QOS_DATA_CF_POLL      10  */ "QoS Data + CF-Poll",
/* FC_SUBTYPE_QOS_DATA_CF_ACK_POLL  11  */ "QoS Data + CF-Ack + CF-Poll",
/* FC_SUBTYPE_QOS_NULL              12  */ "QoS Null",
/* Empty */                                "Subtype 13"
/* FC_SUBTYPE_QOS_CF_POLL           14  */ "QoS CF-Poll",
/* FC_SUBTYPE_QOS_CF_ACK_POLL       15  */ "QoS CF-Ack + CF-Poll"
};

char *type_ctrl_subtypes[] = {
/* FC_SUBTYPE_CTL_WRAPPER           7 */  "Control Wrapper",
/* FC_SUBTYPE_BLOCKACK_REQ          8 */  "Block Ack Req",
/* FC_SUBTYPE_BLOCKACK              9 */  "Block Ack",
/* FC_SUBTYPE_PS_POLL               10 */ "PS poll",
/* FC_SUBTYPE_RTS                   11 */ "RTS",
/* FC_SUBTYPE_CTS                   12 */ "CTS",
/* FC_SUBTYPE_ACK                   13 */ "ACK",
/* FC_SUBTYPE_CF_END                14 */ "CF-END",
/* FC_SUBTYPE_CF_END_ACK            15 */ "CF-END ACK"
};

/* For debug, dump bytes, 40 per line */
static void
dump_bytes(void *stuff, int len)
{
    uint8_t *ptr = (uint8_t *)stuff;
    int j = 0;
    while (len--) {
        WPRINT_APP_INFO(("%02x ", *(ptr++)));
        j++;
        if ((j % 40) == 0)
            WPRINT_APP_INFO(("\n"));
    }
    WPRINT_APP_INFO(("\n"));
}

static void
dump_layer_3(uint8_t *h)
{
    uint16 ether_type = *(uint16 *)(h+FILTER_OFFSET_ETH_HEADER_ETHERTYPE);
    if ((ether_type & 0xff) == 0x08 || (ether_type & 0xff) == 0x86 || (ether_type & 0xff) == 0x88)  {
        switch (ntohs(ether_type)) {
        case 0x0800:
            WPRINT_APP_INFO(("     "));
            dump_ipv4((struct ipv4_hdr *)(h + FILTER_OFFSET_ETH_DATA));
            break;
        case 0x0806:
            WPRINT_APP_INFO(("     "));
            WPRINT_APP_INFO(("ARP\n"));
            break;
        case 0x86DD:
            WPRINT_APP_INFO(("     "));
            WPRINT_APP_INFO(("IPV6\n"));
            break;
        case 0x0842:
            WPRINT_APP_INFO(("     "));
            WPRINT_APP_INFO(("Wake on Lan\n"));
            break;
        default: {
            WPRINT_APP_INFO(("     "));
            WPRINT_APP_INFO(("Unknown Ether Type 0x%04x\n", ntohs(ether_type)));
            dump_bytes(h, 40);
            break;
        }
      }
    }
}

static void
dump_layer_2(struct dot11_header *h)
{
    uint16_t fc = h->fc;
    uint8_t type = FC_TYPE(fc);
    uint8_t type_sub = FC_SUBTYPE(fc);

    uint8_t retry = fc & FC_RETRY ? 1 : 0;
    uint8_t fromds = (fc & FC_FROMDS) >> FC_FROMDS_SHIFT;
    uint8_t tods = (fc & FC_TODS) >> FC_TODS_SHIFT;

    switch (type) {
    case FC_TYPE_MNG: {
      /* Managment Frames */
        WPRINT_APP_INFO(("M %-15s %d/%d ", type_mng_subtypes[type_sub], type, type_sub));
        WPRINT_APP_INFO(("%c FromDS %d ToDS %d Seq %d ", retry ? 'R' : ' ', fromds, tods, h->seq >> SEQNUM_SHIFT));
        WPRINT_APP_INFO((""MACDBG"  "MACDBG"  "MACDBG"\n",
            MAC2STRDBG(h->a1.octet), MAC2STRDBG(h->a2.octet), MAC2STRDBG(h->a3.octet)));
        }
        break;

    case FC_TYPE_CTL: {
        /* Control Frames */
        WPRINT_APP_INFO(("C %-15s %d/%d ", type_ctrl_subtypes[type_sub - FC_SUBTYPE_CTL_WRAPPER], type, type_sub));
        WPRINT_APP_INFO(("%c FromDS %d ToDS %d Seq %d ", retry ? 'R' : ' ', fromds, tods, h->seq >> SEQNUM_SHIFT));
        WPRINT_APP_INFO((""MACDBG"  "MACDBG"  "MACDBG"\n",
            MAC2STRDBG(h->a1.octet), MAC2STRDBG(h->a2.octet), MAC2STRDBG(h->a3.octet)));
        }
        break;
    case FC_TYPE_DATA: {
        /* Data Frames */
        WPRINT_APP_INFO(("D %-15s %d/%d ", type_data_subtypes[type_sub], type, type_sub));
        WPRINT_APP_INFO(("%c FromDS %d ToDS %d Seq %d ", retry ? 'R' : ' ', fromds, tods, h->seq >> SEQNUM_SHIFT));
        WPRINT_APP_INFO((""MACDBG"  "MACDBG"  "MACDBG"\n",
            MAC2STRDBG(h->a1.octet), MAC2STRDBG(h->a2.octet), MAC2STRDBG(h->a3.octet)));
        }
        break;
    default:
        /*  No 802.11 header present  */
        WPRINT_APP_DEBUG(("Unknown FC_TYPE fc 0x%x, type 0x%x, subtype 0x %x\n", fc, type, type_sub));
        break;
    }
}

void raw_packet_callback( wiced_buffer_t buffer, wwd_interface_t interface )
{
    UNUSED_PARAMETER( interface );

    void *raw = host_buffer_get_current_piece_data_pointer( buffer );
    dump_layer_2(raw);
    dump_layer_3(raw);

    host_buffer_release( buffer, WWD_NETWORK_RX );
}

static void
pktfilter_enable(char *filter_list, unsigned int len)
{
    for (int i = 0; i < len; i++) {
        wiced_wifi_enable_packet_filter(filter_list[i]);
    }
    return;
}
static void
pktfilter_disable(char *filter_list, unsigned int len)
{
    for (int i = 0; i < len; i++) {
        wiced_wifi_disable_packet_filter(filter_list[i]);
    }
    return;
}
static void
pktfilter_stats(char *filter_list, unsigned int len)
{
    WPRINT_APP_INFO(("\n"));
    WPRINT_APP_INFO(("               \t Total    Total\n"));
    WPRINT_APP_INFO(("ID:   Matched  \tSent Up  Dropped\n"));
    for (int i = 0; i < len; i++) {
        print_packet_filter_stats(filter_list[i]);
    }
    return;
}
static void
pktfilter_remove(char *filter_list, unsigned int len)
{
    for (int i = 0; i < len; i++) {
        wiced_wifi_remove_packet_filter(filter_list[i]);
    }
    return;
}
