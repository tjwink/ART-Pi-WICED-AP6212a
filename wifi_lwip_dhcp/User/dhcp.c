#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "lwip/dhcp.h"
#include "lwip/inet.h"
#include "dhcp.h"
#include "stdio.h"
#include "wifi_base_config.h"
#include "dhcp.h"
#include "stm32h7xx.h"

#define PRINTF printf
#if LWIP_NETCONN

#define MAX_BUFFER_LEN 256

char sendbuf[MAX_BUFFER_LEN];
extern ip4_addr_t ipaddr;
extern char wiced_if_str[100];
static void dhcp_thread(void *arg)
{
	struct netconn *conn;
	ip4_addr_t ipaddr;
  int ret;
	int strlen = 0;
  
  PRINTF("目地IP地址:%d.%d.%d.%d \t 端口号:%d\n\n",      \
          DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3,DEST_PORT);
  
  PRINTF("请将电脑上位机设置为TCP Server.在wiofi_base_config.h文件中将目标IP地址修改为您电脑上的IP地址\n\n");
  
  PRINTF("修改对应的宏定义:DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3,DEST_PORT\n\n");
  
	while(1)
	{
    conn = netconn_new(NETCONN_TCP);
    if (conn == NULL)
    {
      PRINTF("create conn failed!\n");
      vTaskDelay(10);
      continue;
    }

    PRINTF("create conn success...\n");
    
    //构造服务器IP地址
    IP4_ADDR(&ipaddr,DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3); 			
    
    ret = netconn_connect(conn,&ipaddr,DEST_PORT,0);	        //连接服务器，端口号5001
    if (ret == -1)
    {
      PRINTF("Connect failed!\n");
      netconn_close(conn);
      vTaskDelay(10);
      continue;
    }
    
    PRINTF("Connect to server successful!\n");

    strlen = sprintf(sendbuf,"A LwIP client Using DHCP Address: %s\n",\
    wiced_if_str);

    while(1)
    {
      PRINTF("%s",sendbuf);
      ret=netconn_write(conn,sendbuf, strlen, NETCONN_NOCOPY);
      if (ret == ERR_OK)
      {
         PRINTF("write success...\n");
       }
	   

      vTaskDelay(1000); 
    }
    
//    printf("Connection failed \n");
//    netconn_close(conn); 						//关闭连接
//    netconn_delete(conn);						//删除连接结构
  }
}


void dhcp_netconn_init()
{
  sys_thread_new("dhcp_thread", dhcp_thread, NULL, 2048, 4);
}

#endif /* LWIP_NETCONN*/
