#include "lwip/sockets.h"  /* equivalent of <sys/socket.h> */
#include <string.h>
#include <stdint.h>
#include "appliance.h"
#include "network/wwd_network_constants.h"
#include "RTOS/wwd_rtos_interface.h"
#include "lwip/sys.h"
#include "stm32h7xx.h"
#include "tcpecho.h"
#define PRINTF printf


/******************************************************
 *                      宏定义
 ******************************************************/

/******************************************************
 *                    常量
 ******************************************************/
#define DHCP_STACK_SIZE               (800)

/* BOOTP operations */
#define BOOTP_OP_REQUEST                (1)
#define BOOTP_OP_REPLY                  (2)

/* DHCP commands */
#define DHCPDISCOVER                    (1)
#define DHCPOFFER                       (2)
#define DHCPREQUEST                     (3)
#define DHCPDECLINE                     (4)
#define DHCPACK                         (5)
#define DHCPNAK                         (6)
#define DHCPRELEASE                     (7)
#define DHCPINFORM                      (8)

/* DHCP服务器和客户端的UDP端口号*/
#define IPPORT_DHCPS                   (67)
#define IPPORT_DHCPC                   (68)

/* DHCP套接字超时值（以毫秒为单位）。 修改此设置以使线程退出更具响应性 */
#define DHCP_SOCKET_TIMEOUT     500



/******************************************************
 *                   Enumerations
 ******************************************************/

/******************************************************
 *                 Type Definitions
 ******************************************************/

/******************************************************
 *                    Structures
 ******************************************************/

/* DHCP数据结构 */
typedef struct
{
    uint8_t  opcode;                     /* 包操作码类型 */
    uint8_t  hardware_type;              /* 硬件地址类型*/
    uint8_t  hardware_addr_len;          /* 硬件地址长度 */
    uint8_t  hops;                       /* 网关 */
    uint32_t transaction_id;             /* 交易编号 */
    uint16_t second_elapsed;             /* 开机开始后的秒数 */
    uint16_t flags;
    uint8_t  client_ip_addr[4];          /* 客户端IP地址 */
    uint8_t  your_ip_addr[4];            /* 你的IP 地址*/
    uint8_t  server_ip_addr[4];          /* 服务器IP地址*/
    uint8_t  gateway_ip_addr[4];         /* 网关IP地址 */
    uint8_t  client_hardware_addr[16];   /* 客户端硬件地址 */
    uint8_t  legacy[192];
    uint8_t  magic[4];
    uint8_t  options[275];               /* 选项区 */
    /* as of RFC2131 it is variable length */
} dhcp_header_t;

/******************************************************
 *             静态函数声明
 ******************************************************/
static unsigned char * find_option( dhcp_header_t* request, unsigned char option_num );
static void dhcp_thread( void * thread_input );

/******************************************************
 *              变量定义
 ******************************************************/
static char             new_ip_addr[4]                = { 192, 168, 0, 0 };
static uint16_t         next_available_ip_addr        = ( 1 << 8 ) + 100;
static char             subnet_option_buff[]          = { 1, 4, 255, 255, 0, 0 };
static char             server_ip_addr_option_buff[]  = { 54, 4, 192, 168, 1, 200 };
static char             mtu_option_buff[]             = { 26, 2, WICED_PAYLOAD_MTU>>8, WICED_PAYLOAD_MTU&0xff };
static char             dhcp_offer_option_buff[]      = { 53, 1, DHCPOFFER };
static char             dhcp_ack_option_buff[]        = { 53, 1, DHCPACK };
static char             dhcp_nak_option_buff[]        = { 53, 1, DHCPNAK };
static char             lease_time_option_buff[]      = { 51, 4, 0x00, 0x01, 0x51, 0x80 }; /* 1 day lease */
static char             dhcp_magic_cookie[]           = { 0x63, 0x82, 0x53, 0x63 };
static volatile char    dhcp_quit_flag = 0;
static int              dhcp_socket_handle            = -1;

/*任务句柄*/
static xTaskHandle      dhcp_thread_handle;
static dhcp_header_t    dhcp_header_buff;


#define ADDR_ARY_LEN    10
struct sockaddr_in      source_addr_ary[ADDR_ARY_LEN];
int addr_atry_index=0;
/******************************************************
 *              功能定义
 ******************************************************/
extern void tcpecho_thread(void *arg);
/**
 * @brief start_dhcp_server
 *
 * @param local_addr 热点地址（服务器IP）
 */
void start_dhcp_server( uint32_t local_addr )
{
	/*************************************************************/
	//lwip 相关的应用可以放在这里	
	//tcpecho_init();
	//
	//这
	//里
	//添
	//加
	//对
	//应
	//的
	//lwip
	//demo
	//兼容大部分的 lwip  demo
	/**************************************************************/	
		dhcp_thread((void*)local_addr);	//在最后一直扫描是否有wifi连接				
}



void quit_dhcp_server( void )
{
    dhcp_quit_flag = 1;
}
#define MAKE_IPV4_ADDRESS(a, b, c, d)          ((((uint32_t) (a)) << 24) | (((uint32_t) (b)) << 16) | (((uint32_t) (c)) << 8) | ((uint32_t) (d)))

/**
 * @brief 打印地址信息
 *
 * @param temp_addr 临时地址变量
 */
void log_sin_addr_info(struct sockaddr_in  temp_addr)
{
#define LOG_SW  	//打印开关
#if defined(LOG_SW)

			PRINTF("%d.%d.%d.%d\n",  \
		((temp_addr.sin_addr.s_addr)&0x000000ff),       \
		(((temp_addr.sin_addr.s_addr)&0x0000ff00)>>8),  \
		(((temp_addr.sin_addr.s_addr)&0x00ff0000)>>16), \
		((temp_addr.sin_addr.s_addr)&0xff000000)>>24); 
	
#endif 
}



/**
 * @brief 记录热点的IP
 *
 * @param addr_0 IP
 * @param addr_1 IP
 * @param addr_2 IP
 * @param addr_3 IP
 */
void recording_AP_IP(uint8_t addr_0,uint8_t addr_1,uint8_t addr_2,uint8_t addr_3)
{
	int i=0;
	/*将IP收录在数组中*/
	source_addr_ary[addr_atry_index].sin_addr.s_addr=MAKE_IPV4_ADDRESS(addr_3, addr_2, addr_1, addr_0);
	addr_atry_index++;
	if(addr_atry_index>ADDR_ARY_LEN)
	{
		addr_atry_index=0;
	}
#define FOR_PRINTF_SW
#if defined(FOR_PRINTF_SW)
	/*打印当前已经连接过的IP*/
	PRINTF("->>>>>>>>IP addr <<<<<<<<<<<-\r\n");
	for(i=0;i<addr_atry_index;i++)
	{
		PRINTF("-%d-> ",i);
		log_sin_addr_info(source_addr_ary[i]);
	}
#endif
	
}

/**
 * @brief dhcp_thread
 * 服务器将始终为DISCOVER命令提供下一个可用地址
 * 服务器将使不要求下一个可用地址的任何REQUEST命令无效
 * 服务器将确认用于下一个可用地址的任何请求命令，然后递增下一个可用地址
 * @param void * thread_input 热点的IP地址
 */
void dhcp_thread( void * thread_input )
{
    struct sockaddr_in    my_addr;
    struct sockaddr_in    source_addr;
    struct sockaddr_in    destination_addr;
    int                   status;
    char*                 option_ptr;
    socklen_t             source_addr_len;
    static dhcp_header_t* dhcp_header_ptr = &dhcp_header_buff;
    int                   recv_timeout = DHCP_SOCKET_TIMEOUT;
	
    /* 保存要在DHCP数据包中发送的本地IP地址 */
    my_addr.sin_addr.s_addr = (u32_t) thread_input;//热点的IP地址
    my_addr.sin_family = AF_INET;
		/* 将端口号添加到IP地址 */
    my_addr.sin_port = htons( IPPORT_DHCPS );			//端口号
	
    memcpy( &server_ip_addr_option_buff[2], &my_addr.sin_addr.s_addr, 4 );

		/*打印输出*/
//		PRINTF("my_addr->> \n");
//		log_sin_addr_info(my_addr);
	
    /* 创建DHCP套接字 */
    dhcp_socket_handle = socket(AF_INET, SOCK_DGRAM, 0);
    setsockopt( dhcp_socket_handle, SOL_SOCKET, SO_RCVTIMEO, (char*)&recv_timeout, sizeof( recv_timeout ) );

    /* 将套接字绑定到本地IP地址 */
    status = bind(dhcp_socket_handle, (struct sockaddr*)&my_addr, sizeof(struct sockaddr_in));

    /*设置目标IP地址和端口号 */
    destination_addr.sin_port   = htons( IPPORT_DHCPC );
    destination_addr.sin_family = AF_INET;
		
		/*目标IP为255.255.255.255  广播地址*/
    memset( &destination_addr.sin_addr.s_addr, 0xff, 4 );
				
		/*打印输出*/
//		PRINTF("destination_addr->> \n");
//		log_sin_addr_info(destination_addr);

    /* 无限循环 */
    while ( dhcp_quit_flag == 0 )
    {
        /* 休眠直到从套接字接收到数据。 */
        status = recvfrom( dhcp_socket_handle, (char *)dhcp_header_ptr, sizeof( dhcp_header_buff ), 0 , (struct sockaddr *) &source_addr, &source_addr_len);
        if ( status > 0 )
        {
            /* 检查DHCP命令 */
            switch ( dhcp_header_ptr->options[2] )
            {
                case DHCPDISCOVER://DHCP  发现  新连接
                    {
                        /* Discover命令-发回OFFER响应 */
                        dhcp_header_ptr->opcode = BOOTP_OP_REPLY;

                        /* 清除DHCP选项列表*/
                        memset( &dhcp_header_ptr->options, 0, sizeof( dhcp_header_ptr->options ) );

                        /*创建要约的IP地址 */
                        new_ip_addr[2] = next_available_ip_addr >> 8;
                        new_ip_addr[3] = next_available_ip_addr & 0xff;//组成IP地址
																						
                        memcpy( &dhcp_header_ptr->your_ip_addr, new_ip_addr, 4 );//将新连接的从机IP地址复制到结构体
																					
                        /* 复制魔术DHCP号码*/
                        memcpy( dhcp_header_ptr->magic, dhcp_magic_cookie, 4 );

                        /*新增选项 */
                        option_ptr = (char *) &dhcp_header_ptr->options;
                        memcpy( option_ptr, dhcp_offer_option_buff, 3 );       /* DHCP消息类型 */
                        option_ptr += 3;
                        memcpy( option_ptr, server_ip_addr_option_buff, 6 );   /* 服务器标识符 */
                        option_ptr += 6;
                        memcpy( option_ptr, lease_time_option_buff, 6 );       /* Lease Time */
                        option_ptr += 6;
                        memcpy( option_ptr, subnet_option_buff, 6 );           /* 子网掩码 */
                        option_ptr += 6;
                        memcpy( option_ptr, server_ip_addr_option_buff, 6 );   /*路由器（网关） */
                        option_ptr[0] = 3; /* Router id */
                        option_ptr += 6;
                        memcpy( option_ptr, server_ip_addr_option_buff, 6 );   /* DNS 服务器 */
                        option_ptr[0] = 6; /* DNS server id */
                        option_ptr += 6;
                        memcpy( option_ptr, mtu_option_buff, 4 );              /* 接口MTU */
                        option_ptr += 4;
                        option_ptr[0] = 0xff; /* 结束选项 */
                        option_ptr++;

												/* 发送包 */
                        sendto( dhcp_socket_handle, (char *)dhcp_header_ptr, (int)(option_ptr - (char*)&dhcp_header_buff), 0 , (struct sockaddr *) &destination_addr, sizeof( destination_addr ));
                    }
                    break;

                case DHCPREQUEST://DHCP请求
                    {
                        /* REQUEST command - send back ACK or NAK */
                        unsigned char* requested_address;
                        uint32_t*      server_id_req;
                        uint32_t*      req_addr_ptr;
                        uint32_t*      newip_ptr;

                        /* 检查请求是否为此服务器使用 */
                        server_id_req = (uint32_t*) find_option( dhcp_header_ptr, 54 );
                        if ( ( server_id_req != NULL ) &&
                             ( my_addr.sin_addr.s_addr != *server_id_req ) )
                        {
                            break; /* 服务器ID与本地IP地址不匹配 */
                        }

                        dhcp_header_ptr->opcode = BOOTP_OP_REPLY;

                        /* 在选项中找到所需的地址 */
                        requested_address = find_option( dhcp_header_ptr, 50 );

                        /* 复制要求的地址 */
                        memcpy( &dhcp_header_ptr->your_ip_addr, requested_address, 4 );

                        /* 空白选项列表*/
                        memset( &dhcp_header_ptr->options, 0, sizeof( dhcp_header_ptr->options ) );

                        /*将DHCP幻数复制到数据包中*/
                        memcpy( dhcp_header_ptr->magic, dhcp_magic_cookie, 4 );

                        option_ptr = (char *) &dhcp_header_ptr->options;

                        /*检查是否请求下一个可用IP地址 */
                        req_addr_ptr = (uint32_t*) dhcp_header_ptr->your_ip_addr;
                        newip_ptr = (uint32_t*) new_ip_addr;
                        if ( *req_addr_ptr != ( ( *newip_ptr & 0x0000ffff ) | ( ( next_available_ip_addr & 0xff ) << 24 ) | ( ( next_available_ip_addr & 0xff00 ) << 8 ) ) )
                        {
                            /* 请求不是下一个可用IP的请求-通过发送NAK强制客户端获取下一个可用IP */
                            /* 添加适当的选项*/
                            memcpy( option_ptr, dhcp_nak_option_buff, 3 );  /* DHCP消息类型 */
                            option_ptr += 3;
                            memcpy( option_ptr, server_ip_addr_option_buff, 6 ); /* 服务器标识符*/
                            option_ptr += 6;
                            memset( &dhcp_header_ptr->your_ip_addr, 0, sizeof( dhcp_header_ptr->your_ip_addr ) ); /*清除“您的地址”字段*/
                        }
                        else
                        {
                            /* 请求不是下一个可用IP的请求-通过发送NAK强制客户端获取下一个可用IP
                             * 添加适当的选项
                             */
                            memcpy( option_ptr, dhcp_ack_option_buff, 3 );       /*DHCP消息类型 */
                            option_ptr += 3;
                            memcpy( option_ptr, server_ip_addr_option_buff, 6 ); /* 服务器标识符 */
                            option_ptr += 6;
                            memcpy( option_ptr, lease_time_option_buff, 6 );     /* 租期 */
                            option_ptr += 6;
                            memcpy( option_ptr, subnet_option_buff, 6 );         /*子网掩码*/
                            option_ptr += 6;
                            memcpy( option_ptr, server_ip_addr_option_buff, 6 ); /* 路由器（网关）*/
                            option_ptr[0] = 3; /* Router id */
                            option_ptr += 6;
                            memcpy( option_ptr, server_ip_addr_option_buff, 6 ); /* DNS 服务器 */
                            option_ptr[0] = 6; /* DNS server id */
                            option_ptr += 6;
                            memcpy( option_ptr, mtu_option_buff, 4 );            /* 接口MTU */
                            option_ptr += 4;
														//分配新的IP地址192.168.1.100
                            //WPRINT_APP_INFO(("Assigned new IP address %d.%d.%d.%d\n", (uint8_t)new_ip_addr[0], (uint8_t)new_ip_addr[1], next_available_ip_addr>>8, next_available_ip_addr&0xff )); 
														recording_AP_IP((uint8_t)new_ip_addr[0], (uint8_t)new_ip_addr[1], next_available_ip_addr>>8, next_available_ip_addr&0xff);
														
														tcpecho_init();

                            /* 递增IP地址*/
                            next_available_ip_addr++;
                            if ( ( next_available_ip_addr & 0xff ) == 0xff ) /* 处理低字节翻转 */
                            {
                                next_available_ip_addr += 101;
                            }
                            if ( ( next_available_ip_addr >> 8 ) == 0xff ) /* 处理高字节翻转*/
                            {
                                next_available_ip_addr += ( 2 << 8 );
                            }
                        }
                        option_ptr[0] = 0xff; /*结束项*/
                        option_ptr++;

                        /* 发送包 */
                        sendto( dhcp_socket_handle, (char *)&dhcp_header_buff, (int)(option_ptr - (char*)&dhcp_header_buff), 0 , (struct sockaddr *) &destination_addr, sizeof( destination_addr ));
                    }
                    break;

                default:
                    break;
            }
        }
    }
		

    /* 删除 DHCP 连接 */
    lwip_close( dhcp_socket_handle );
		
		PRINTF("删除 DHCP 连接\r\n");
    /* 清除这个开始线程*/
    vTaskDelete( dhcp_thread_handle );
}

/**
 * @brief 查找指定的DHCP选项
 *
 * @param request DHCP请求结构
 * @param option_num 查找哪个DHCP选项号
 * @return 指向DHCP选项数据的指针，如果找不到，则为NULL
 */
static unsigned char * find_option( dhcp_header_t* request, unsigned char option_num )
{
    unsigned char* option_ptr = (unsigned char*) request->options;
    while ( ( option_ptr[0] != 0xff ) &&
            ( option_ptr[0] != option_num ) &&
            ( option_ptr < ( (unsigned char*) request ) + sizeof( dhcp_header_t ) ) )
    {
        option_ptr += option_ptr[1] + 2;
    }
    if ( option_ptr[0] == option_num )
    {
        return &option_ptr[2];
    }
    return NULL;

}

