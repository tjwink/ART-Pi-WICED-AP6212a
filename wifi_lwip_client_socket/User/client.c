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
#include "client.h"
#include "lwip/opt.h"
#include "lwip/sys.h"
#include "lwip/api.h"
#include "arch/sys_arch.h"
#include "wifi_base_config.h"
#include "string.h"
#define  PRINTF printf

static void client(void *thread_param)
{
  int sock = -1;
  struct sockaddr_in client_addr;
  
  ip4_addr_t ipaddr;
  
  uint8_t send_buf[]= "This is a TCP Client test...\n";
  
  PRINTF("目地IP地址:%d.%d.%d.%d \t 端口号:%d\n\n",      \
          DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3,DEST_PORT);
  
  PRINTF("请将电脑上位机设置为TCP Server.在wifi_base_config文件中将目标IP地址修改为您电脑上的IP地址\n\n");
  
  PRINTF("修改对应的宏定义:DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3,DEST_PORT\n\n");
  
  IP4_ADDR(&ipaddr,DEST_IP_ADDR0,DEST_IP_ADDR1,DEST_IP_ADDR2,DEST_IP_ADDR3);
  while(1)
  {
    sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0)
    {
      PRINTF("Socket error\n");
      vTaskDelay(10);
      continue;
    } 

    client_addr.sin_family = AF_INET;      
    client_addr.sin_port = htons(DEST_PORT);   
    client_addr.sin_addr.s_addr = ipaddr.addr;
    memset(&(client_addr.sin_zero), 0, sizeof(client_addr.sin_zero));    

    if (connect(sock, 
               (struct sockaddr *)&client_addr, 
                sizeof(struct sockaddr)) == -1) 
    {
        PRINTF("Connect failed!\n");
        closesocket(sock);
        vTaskDelay(10);
        continue;
    }                                           
    
    PRINTF("Connect to server successful!\n");
    
    while (1)
    {
      if(write(sock,send_buf,sizeof(send_buf)) < 0)
        break;
   
      vTaskDelay(1000);
    }
    
    closesocket(sock);
  }

}

void
client_init(void)
{
  sys_thread_new("client", client, NULL, 512, 4);
}
