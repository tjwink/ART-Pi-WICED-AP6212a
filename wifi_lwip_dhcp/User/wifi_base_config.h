#ifndef WIFI_BASE_CONFIG_H
#define WIFI_BASE_CONFIG_H
#include "lwip/opt.h"
#include "lwip/icmp.h"
#include "lwip/inet_chksum.h"
#include "lwip/sockets.h"
#include "lwip/mem.h"
#include "lwip/inet.h"
#include "netif/etharp.h"
#include "lwip/tcpip.h"
#include "lwip/dhcp.h"
#include "lwip/prot/dhcp.h"
#include "wwd_network.h"
#include "wwd_management.h"
#include "wwd_wifi.h"
#include "wwd_debug.h"
#include "wwd_assert.h"
#include "platform/wwd_platform_interface.h"
#include "RTOS/wwd_rtos_interface.h"
#include "wwd_buffer_interface.h"

/******************************************************
 *                      Macros
 ******************************************************/
/** @cond */
#define MAKE_IPV4_ADDRESS(a, b, c, d)          ((((uint32_t) (a)) << 24) | (((uint32_t) (b)) << 16) | (((uint32_t) (c)) << 8) | ((uint32_t) (d)))

/* 在发行版中，所有UART打印都被抑制，从而消除
了removef和malloc依赖，从而显着减少了内存使用*/
#ifndef DEBUG
#undef  WPRINT_APP_INFO
#define WPRINT_APP_INFO(args) printf args
#undef  WPRINT_APP_ERROR
#define WPRINT_APP_ERROR(args) printf args
#endif


/******************************************************
 *                    配置
 ******************************************************/
//#define AP_SSID              "embedfire_AP6181"    /* 路由名称 */
#define AP_SSID              "fire_AP6181"    /* 路由名称 */
#define AP_PASS              "123456789"         /* 路由密码 */
#define AP_SEC               WICED_SECURITY_WPA2_MIXED_PSK  /* 路由加密 */

#define COUNTRY              WICED_COUNTRY_AUSTRALIA    /* 选择城市 据说澳大利亚的信号更强一些，这里选择为澳大利亚 */
//#define USE_DHCP             WICED_TRUE     /* 是否使用 DHCP */
#define USE_DHCP             WICED_FALSE     /* 是否使用 DHCP */
#define IP_ADDR              MAKE_IPV4_ADDRESS( 192, 168,   100,  128 )  /* 如果USE_DHCP为WICED_TRUE，则不需要 */
#define GW_ADDR              MAKE_IPV4_ADDRESS( 192, 168,   100,   1 )   /* 如果USE_DHCP为WICED_TRUE，则不需要 */
#define NETMASK              MAKE_IPV4_ADDRESS( 255, 255, 255,   0 )    /* 如果USE_DHCP为WICED_TRUE，则不需要 */
/* #define PING_TARGET          MAKE_IPV4_ADDRESS( 192, 168,   1, 2 ) */  /* 如果要ping特定IP而不是网关，请取消注释*/

#define DEST_IP_ADDR0               192
#define DEST_IP_ADDR1               168
#define DEST_IP_ADDR2               100
#define DEST_IP_ADDR3               2

#define DEST_PORT                  5001

#define IP_ADDR0                    192
#define IP_ADDR1                    168
#define IP_ADDR2                    100
#define IP_ADDR3                    200
/*
*/
#define LOCAL_PORT 									5001

static const wiced_ssid_t ap_ssid =
{
    .length = sizeof(AP_SSID)-1,
    .value  = AP_SSID,
};


#define PING_RCV_TIMEOUT     (1000)    /** ping接收超时-以毫秒为单位*/
#define PING_DELAY           (1000)    /** ping响应/超时与下一次ping发送之间的延迟-以毫秒为单位 */
#define PING_ID              (0xAFAF)
#define PING_DATA_SIZE       (32)      /** ping 其他数据大小*/
#define JOIN_TIMEOUT         (10000)   /** 加入无线网络的超时时间（以毫秒为单位）= 10秒*/
#define APP_THREAD_STACKSIZE (5120)

static struct netif wiced_if;
extern ip4_addr_t target;
extern int socket_hnd;
extern wwd_time_t send_time;



extern void Config_WIFI_LwIP_Info(void);
err_t ping_send( int socket_hnd, ip4_addr_t *addr );
err_t ping_recv( int socket_hnd );
void app_main( void );

#endif 
