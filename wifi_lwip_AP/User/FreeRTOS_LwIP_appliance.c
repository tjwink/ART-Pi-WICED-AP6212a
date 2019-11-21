#include "wwd_wifi.h"
#include "wwd_management.h"
#include "wwd_network.h"
#include "network/wwd_buffer_interface.h"
#include "platform/wwd_platform_interface.h"
#include "lwip/opt.h"
#include "lwip/mem.h"
#include <string.h>
#include "lwip/init.h"
#include "lwip/tcpip.h"
#include "netif/etharp.h"
#include "lwip/sockets.h"  /* equivalent of <sys/socket.h> */
#include "lwip/inet.h"
#include "lwip/dhcp.h"
#include "wwd_wifi.h"
#include "wwd_management.h"
#include "network/wwd_buffer_interface.h"
#include "network/wwd_network_interface.h"
#include "platform/wwd_platform_interface.h"
//#include "web_server.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include <math.h>
#include "appliance.h"
#include "lwip/prot/dhcp.h"

/******************************************************
 *                      宏定义
 ******************************************************/
/** @cond */
#define MAKE_IPV4_ADDRESS(a, b, c, d)          ((((uint32_t) (a)) << 24) | (((uint32_t) (b)) << 16) | (((uint32_t) (c)) << 8) | ((uint32_t) (d)))
/** @endcond */

/******************************************************
 *                    常量
 ******************************************************/

#define COUNTRY                 WICED_COUNTRY_AUSTRALIA
#define AP_IP_ADDR              MAKE_IPV4_ADDRESS( 192, 168, 1,  200 )
#define AP_NETMASK              MAKE_IPV4_ADDRESS( 255, 255, 255,   0 )
#define JOIN_TIMEOUT            (10000)   /** 加入无线网络的超时时间（以毫秒为单位）= 10秒 */

#define SAMPLES_PER_SEND        (26)
#define TIME_MS_BETWEEN_SAMPLES (1000)
#define DESTINATION_UDP_PORT    (50007)
#define AP_SSID_START           "fire_"
#define AP_PASS                 "12345678"
#define AP_SEC                  WICED_SECURITY_WPA2_AES_PSK  /* WICED_SECURITY_OPEN */
#define AP_CHANNEL              (1)
#define APP_THREAD_STACKSIZE    (5120)
#define DHCP_THREAD_STACKSIZE   (800)
//#define WEB_SERVER_STACK_SIZE   (1024)

/******************************************************
 *                   枚举
 ******************************************************/

/******************************************************
 *                 类型定义
 ******************************************************/           

/******************************************************
 *                    结构
 ******************************************************/

/******************************************************
 *              静态函数声明
 ******************************************************/

/******************************************************
 *              变量定义
 ******************************************************/
static struct netif wiced_if;


/* 传感器配置设置变量 */
appliance_config_t       appliance_config   = { .config_type = CONFIG_NONE };

/******************************************************
 *             静态函数原型
 ******************************************************/
static void run_ap_server( void );


/******************************************************
 *              功能定义
 ******************************************************/

/**
 * Main appliance app thread
 *
 * Obtains network information and credentials via AP mode web server
 *
 */

void app_main( void )
{
    run_ap_server( );
}

/**
 * 启动热点 DHCP服务器
 *
 *此功能将初始化802.11设备，创建802.11接入点，然后启动Web服务器
 * 和DHCP服务器，以允许浏览器客户端进行连接。
 */
extern void tcpecho_thread(void *arg);
extern void dhcp_thread( void * thread_input );
static void run_ap_server( void )
{
    wiced_ssid_t ap_ssid;
    wiced_mac_t  my_mac;
    ip4_addr_t ap_ipaddr;
    ip4_addr_t ap_netmask;
    wwd_result_t result;

    /* 初始化Wiced */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init( NULL );
    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

    /* 创建SSID*/
    wwd_wifi_get_mac_address( &my_mac, WWD_STA_INTERFACE );
    sprintf( (char*) ap_ssid.value, AP_SSID_START "%02X%02X%02X%02X%02X%02X", my_mac.octet[0], my_mac.octet[1], my_mac.octet[2], my_mac.octet[3], my_mac.octet[4], my_mac.octet[5] );
    ap_ssid.length = strlen( (char*)ap_ssid.value );

    WPRINT_APP_INFO(("Starting Access Point: SSID: %s\n", (char*)ap_ssid.value ));

    /*启动接入点 */
    wwd_wifi_start_ap( &ap_ssid, AP_SEC, (uint8_t*) AP_PASS, sizeof( AP_PASS ) - 1, AP_CHANNEL );

    /* 设置网络接口*/
    ip4_addr_set_u32(&ap_ipaddr, htonl( AP_IP_ADDR ));
    ip4_addr_set_u32(&ap_netmask, htonl( AP_NETMASK ));

    if ( NULL == netif_add( &wiced_if, &ap_ipaddr, &ap_netmask, &ap_ipaddr, (void*) WWD_AP_INTERFACE, ethernetif_init, ethernet_input ) )
    {
        WPRINT_APP_ERROR( ( "Failed to start network interface\n" ) );
        return;
    }
    netif_set_default( &wiced_if );
    netif_set_up( &wiced_if );

    WPRINT_APP_INFO( ( "Network ready IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&wiced_if))));
		/*开始运行 dhcp 服务器，也可以说是 启动热点*/
		start_dhcp_server( ap_ipaddr.addr );
		
    quit_dhcp_server();
		
    /* 关掉一切 */
    netif_set_down( &wiced_if );
    netif_remove( &wiced_if );

    if ( WWD_SUCCESS != wwd_wifi_stop_ap( ) )
    {
        WPRINT_APP_ERROR(("Failed to stop WICED access point\n"));
    }
/*
    if ( WICED_SUCCESS != wwd_management_wifi_off( ) )
    {
        WINFO_APP_ERROR(("WICED de-initialization failed\n"));
    }
*/
}


