/*
 * Copyright (c) 2001-2003 Swedish Institute of Computer Science.
 * All rights reserved. 
 * 
 * Redistribution and use in source and binary forms, with or without modification, 
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED 
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT 
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING 
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
 * OF SUCH DAMAGE.
 *
 * This file is part of the lwIP TCP/IP stack.
 * 
 * Author: Adam Dunkels <adam@sics.se>
 *
 */
#include "dns.h"

#include "lwip/opt.h"

#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"
#include "lwip/dns.h"

#include <string.h>
#include <stdio.h>
#include "stm32h7xx.h"
#include "appliance.h"
#include "wifi_base_config.h"

#define PRINTF printf
#define RECV_BUFSZ      300

/*-----------------------------------------------------------------------------------*/
char *recvbuf;
extern char wiced_if_str[100];
static void 
dns_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err;
  ip4_addr_t dns_ip;
  
  char *recvbuf;
  
  recvbuf = (char *) pvPortMalloc(RECV_BUFSZ);
    if (!recvbuf) return ;
  
  LWIP_UNUSED_ARG(arg);

  /* Create a new connection identifier. */
  /* Bind connection to well known port number 5001. */
#if LWIP_IPV6
  conn = netconn_new(NETCONN_TCP_IPV6);
  netconn_bind(conn, IP6_ADDR_ANY, DEST_PORT);
#else /* LWIP_IPV6 */
  conn = netconn_new(NETCONN_TCP);
  netconn_bind(conn, IP_ADDR_ANY, DEST_PORT);
#endif /* LWIP_IPV6 */
  LWIP_ERROR("tcpecho: invalid conn", (conn != NULL), return;);
  
  PRINTF("请在电脑上位机软件中将电脑设置为client，连接到开发板上...\n\n");
  PRINTF("请将电脑上位机设置为TCP Client.并且连接到开发板,开发板IP地址是:%s \t 端口号:%d\n\n",\
          wiced_if_str,LOCAL_PORT);
  PRINTF("可以使用   -> mqtt.heclouds.com <- 域名进行测试...\n\n");
	
  /* Tell connection to go into listening mode. */
  netconn_listen(conn);

  while (1) {

    /* Grab new connection. */
    err = netconn_accept(conn, &newconn);
    /*printf("accepted new connection %p\n", newconn);*/
    /* Process the new connection. */
    if (err == ERR_OK) {
      struct netbuf *buf;
      void *data;
      u16_t len;
      
      sprintf(recvbuf,"连接成功...请发送一个域名进行dns解析...\n\n");
      PRINTF("%s",recvbuf);

      netconn_write(newconn, recvbuf, strlen(recvbuf), NETCONN_COPY);

      while ((err = netconn_recv(newconn, &buf)) == ERR_OK) {
        do {
             netbuf_data(buf, &data, &len);
             if(len == 0)
               continue;
             
             memset(recvbuf,0,RECV_BUFSZ);
             memcpy(recvbuf, data, len); 
             //请求域名服务器解析出对应的ip地址
             err = netconn_gethostbyname(recvbuf, &dns_ip);
             if(err == ERR_OK)
             {
               len = sprintf(recvbuf,"%s = %s\n",recvbuf,ip_ntoa(&dns_ip));
               PRINTF("%s",recvbuf);
               netconn_write(newconn, recvbuf, len, NETCONN_COPY);
             }
             else
             {
               len = sprintf(recvbuf,"get host name fail...\n");
               PRINTF("%s",recvbuf);
               netconn_write(newconn, recvbuf, len, NETCONN_COPY);
             }
             
          
        } while (netbuf_next(buf) >= 0);
        netbuf_delete(buf);
      }
      /*printf("Got EOF, looping\n");*/ 
      /* Close connection and discard connection identifier. */
      netconn_close(newconn);
      netconn_delete(newconn);
      PRINTF("连接已断开....\n\n");
    }
  }
}/*-----------------------------------------------------------------------------------*/
void
app_dns_init(void)
{
  sys_thread_new("dns_thread", dns_thread, NULL, 512, 4);
}
/*-----------------------------------------------------------------------------------*/

#endif /* LWIP_NETCONN */
