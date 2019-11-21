#include "wifi_base_config.h"


extern void netio_init(void);
ip4_addr_t ipaddr, netmask, gw;
char wiced_if_str[100];
int wiced_if_str_len=0;
/**
 * @brief 打印地址信息
 *
 * @param temp_addr 临时地址变量
 */
//void log_sin_addr_info(struct sockaddr_in  temp_addr)
void log_sin_addr_info()
{
#define LOG_SW  	//打印开关
#if defined(LOG_SW)

	char str_temp[100];
	sprintf(str_temp,"%d.%d.%d.%d\n",192,168,0,1);
	printf("addr->%s",str_temp);			
	
#endif 
}
/*
配置wifi lwip信息
*/
void Config_WIFI_LwIP_Info()
{
    wwd_result_t result;

    /* 启动WICED（启动802.11设备） */
    WPRINT_APP_INFO(("Starting Wiced v" WICED_VERSION "\n"));

    wwd_buffer_init( NULL );
    result = wwd_management_wifi_on( COUNTRY );
    if ( result != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Error %d while starting WICED!\n", result));
    }

	      
    /*尝试加入Wi-Fi网络 */
    WPRINT_APP_INFO(("Joining : " AP_SSID "\n"));
    while ( wwd_wifi_join( &ap_ssid, AP_SEC, (uint8_t*) AP_PASS, sizeof( AP_PASS ) - 1, NULL, WWD_STA_INTERFACE ) != WWD_SUCCESS )
    {
        WPRINT_APP_INFO(("Failed to join  : " AP_SSID "   .. retrying\n"));
    }
    WPRINT_APP_INFO(("Successfully joined : " AP_SSID "\n"));
    
    /* 设置IP配置*/
    if ( USE_DHCP == WICED_TRUE )
    {
        ip4_addr_set_zero( &gw );
        ip4_addr_set_zero( &ipaddr );
        ip4_addr_set_zero( &netmask );
    }
    else
    {
        ipaddr.addr  = htonl( IP_ADDR );
        gw.addr      = htonl( GW_ADDR );
        netmask.addr = htonl( NETMASK );
    }

    if ( NULL == netif_add( &wiced_if, &ipaddr, &netmask, &gw, (void*) WWD_STA_INTERFACE, ethernetif_init, ethernet_input ) )
    {
        WPRINT_APP_ERROR( ( "Failed to start network interface\n" ) );
        return;
    }

    netif_set_default( &wiced_if );
    netif_set_up( &wiced_if );

    /* 做DHCP协商*/
    WPRINT_APP_INFO(("Obtaining IP address via DHCP\n"));
    struct dhcp netif_dhcp;
    dhcp_set_struct( &wiced_if, &netif_dhcp );
    dhcp_start( &wiced_if );
    while ( netif_dhcp.state != DHCP_STATE_BOUND )
    {
        /* 等待 */
        sys_msleep( 10 );
    }

    WPRINT_APP_INFO( ( "Network ready IP: %s\n", ip4addr_ntoa(netif_ip4_addr(&wiced_if))));
		wiced_if_str_len=sprintf(wiced_if_str,"%s",ip4addr_ntoa(netif_ip4_addr(&wiced_if)));
#if 0
    WPRINT_APP_INFO( ( "Shutting down WICED\n" ) );

    /* 再次关闭一切*/
    dhcp_stop( &wiced_if );
    netif_set_down( &wiced_if );
    netif_remove( &wiced_if );
    wwd_wifi_leave( WWD_STA_INTERFACE );
    wwd_wifi_set_down();
    if ( WWD_SUCCESS != wwd_management_wifi_off( ) )
    {
        WPRINT_APP_ERROR(("WICED de-initialization failed\n"));
    }
#endif

		
}



