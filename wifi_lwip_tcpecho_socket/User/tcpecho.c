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
#include "tcpecho.h"
#include "lwip/opt.h"
#include "wifi_base_config.h"
#include "stm32h7xx.h"
#define PRINTF printf
#if LWIP_NETCONN

#include "lwip/sys.h"
#include "lwip/api.h"
/*-----------------------------------------------------------------------------------*/

#define RECV_DATA         (1024)

char *recv_data;
static void 
tcpecho_thread(void *arg)
{
  int sock = -1,connected;

  struct sockaddr_in server_addr,client_addr;
  socklen_t sin_size;
  int recv_data_len;
  
  PRINTF("本地端口号是%d\n\n",LOCAL_PORT);
  
  recv_data = (char *)pvPortMalloc(RECV_DATA);
  if (recv_data == NULL)
  {
      PRINTF("No memory\n");
      goto __exit;
  }
  
  sock = socket(AF_INET, SOCK_STREAM, 0);
  if (sock < 0)
  {
      PRINTF("Socket error\n");
      goto __exit;
  }
  
  server_addr.sin_family = AF_INET;
  server_addr.sin_addr.s_addr = INADDR_ANY;
  server_addr.sin_port = htons(LOCAL_PORT);
  memset(&(server_addr.sin_zero), 0, sizeof(server_addr.sin_zero));
  
  if (bind(sock, (struct sockaddr *)&server_addr, sizeof(struct sockaddr)) == -1)
  {
      PRINTF("Unable to bind\n");
      goto __exit;
  }
  
  if (listen(sock, 5) == -1)
  {
      PRINTF("Listen error\n");
      goto __exit;
  }
  
  while(1)
  {
    sin_size = sizeof(struct sockaddr_in);

    connected = accept(sock, (struct sockaddr *)&client_addr, &sin_size);

    PRINTF("new client connected from (%s, %d)\n",
            inet_ntoa(client_addr.sin_addr), ntohs(client_addr.sin_port));
    {
      int flag = 1;
      
      setsockopt(connected,
                 IPPROTO_TCP,     /* set option at TCP level */
                 TCP_NODELAY,     /* name of option */
                 (void *) &flag,  /* the cast is historical cruft */
                 sizeof(int));    /* length of option value */
    }
    
    while(1)
    {
      recv_data_len = recv(connected, recv_data, RECV_DATA, 0);
       PRINTF("Rev len %d \r\n",recv_data_len);
      if (recv_data_len <= 0) 
        break;
            
      write(connected,recv_data,recv_data_len);
      
    }
    if (connected >= 0) 
      closesocket(connected);
    
    connected = -1;
  }
__exit:
  if (sock >= 0) closesocket(sock);
  if (recv_data) free(recv_data);
}
/*-----------------------------------------------------------------------------------*/
void
tcpecho_init(void)
{
  sys_thread_new("tcpecho_thread", tcpecho_thread, NULL, 512, 4);
}
/*-----------------------------------------------------------------------------------*/

#endif /* LWIP_NETCONN */
