
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"

#include "httpserver-netconn.h"
#include "appliance.h"
#include "wifi_base_config.h"
#include "./led/bsp_led.h"   

#define PRINTF printf

#define bool _Bool
#define true 1
#define false 0

#if LWIP_NETCONN

#ifndef HTTPD_DEBUG
#define HTTPD_DEBUG         LWIP_DBG_OFF
#endif

#define	FALSE  0		/* 假 */
#define	TRUE  !FALSE	/* 真 */

const static char http_html_hdr[] = "HTTP/1.1 200 OK\r\nContent-type: text/html\r\n\r\n";

const unsigned char Led1On_Data[] ="<HTML> \
                                    <head><title>HTTP LED Control</title></head> \
                                    <center> \
                                    <p> \
                                    <font size=\"6\">LED<font style = \"color:red\">已打开！</font> \
                                    <form method=post action=\"off\" name=\"ledform\"> \
                                    <input type=\"submit\" value=\"关闭\" style=\"width:80px;height:30px;\"></form> \
                                    </center> \
                                    </HTML> ";

	//当LED灭时，向浏览器返回如下html信息，显示结果如下图15-7所示
	const unsigned char Led1Off_Data[] =" <HTML> \
                                        <head><title>HTTP LED Control</title></head> \
                                        <center> \
                                        <p> \
                                        <font size=\"6\">LED<font style = \"color:red\">已关闭！</font> \
                                        <form method=post action=\"on\" name=\"ledform\"> \
                                        <input type=\"submit\" value=\"打开\" style=\"width:80px;height:30px;\"></form> \
                                        </center> \
                                        </HTML> ";
  
  static const char http_index_html[] = "<html><head><title>Congrats!</title></head>\
                                         <body><h2 align=\"center\">Welcome to Fire lwIP HTTP Server!</h2>\
                                         <p align=\"center\">This is a small test page : http control led.</p>\
                                         <p align=\"center\"><a href=\"http://www.firebbs.cn/forum.php/\"> <font size=\"6\"> 野火电子论坛 </font> </a></p>\
                                         <a href=\"http://www.firebbs.cn/forum.php/\">\
                                         <img src=\"http://www.firebbs.cn/data/attachment/portal/201806/05/163015rhz7mbgbt0zfujzh.jpg\"/></a>\
                                         </body></html>";
static bool led_on = FALSE;

/*发送网页数据*/
void httpserver_send_html(struct netconn *conn, bool led_status)
{
  /* 发送数据头 */
  netconn_write(conn, http_html_hdr, sizeof(http_html_hdr)-1, NETCONN_NOCOPY);
	/* 根据LED状态，发送不同的LED数据 */
	if(led_status == TRUE)
     netconn_write(conn, Led1On_Data, sizeof(Led1On_Data)-1, NETCONN_NOCOPY);
	else
	  netconn_write(conn, Led1Off_Data, sizeof(Led1Off_Data)-1, NETCONN_NOCOPY);
  netconn_write(conn, http_index_html, sizeof(http_index_html)-1, NETCONN_NOCOPY);

}
/** Serve one HTTP connection accepted in the http thread */

static void httpserver_serve(struct netconn *conn)
{
  struct netbuf *inbuf;
  char *buf;
  u16_t buflen;
  err_t err;
  
  /*等待客户端的命令数据*/
  err = netconn_recv(conn, &inbuf);
  
  if (err == ERR_OK) {
    netbuf_data(inbuf, (void**)&buf, &buflen);
    /* “GET”命令*/
    if (buflen>=5 &&
        buf[0]=='G' &&
        buf[1]=='E' &&
        buf[2]=='T' &&
        buf[3]==' ' &&
        buf[4]=='/' ) {
      
      /* “POST” 命令*/
      httpserver_send_html(conn, led_on);
    }
	else if(buflen>=8&&buf[0]=='P'&&buf[1]=='O'&&buf[2]=='S'&&buf[3]=='T')
	{
		if(buf[6]=='o'&&buf[7]=='n'){		//请求打开LED
		    led_on = TRUE;
        LED1_ON;
        PRINTF("LED ON!\n");
		}else if(buf[6]=='o'&&buf[7]=='f'&&buf[8]=='f'){	//请求关闭LED
		    led_on = FALSE;
         LED1_OFF;
        PRINTF("LED OFF!\n");
	    }
		//发送数据
		httpserver_send_html(conn, led_on);
	}

	netbuf_delete(inbuf);
  }
  /*  关闭*/
  netconn_close(conn);
}

/** The main function, never returns! */
static void
httpserver_thread(void *arg)
{
  struct netconn *conn, *newconn;
  err_t err;
  LWIP_UNUSED_ARG(arg);
  
  /* 创建连接结构 */
  conn = netconn_new(NETCONN_TCP);
  LWIP_ERROR("http_server: invalid conn", (conn != NULL), return;);

  led_on = TRUE;
  LED1_ON;
  
  /* 绑定IP地址与端口号  */
  netconn_bind(conn, NULL, LOCAL_PORT);
  
  /* 监听 */
  netconn_listen(conn);
  
  do {
    err = netconn_accept(conn, &newconn);
    if (err == ERR_OK) {
      httpserver_serve(newconn);
      netconn_delete(newconn);
    }
  } while(err == ERR_OK);
  LWIP_DEBUGF(HTTPD_DEBUG,
    ("http_server_netconn_thread: netconn_accept received error %d, shutting down",
    err));
  netconn_close(conn);
  netconn_delete(conn);
}

/** 初始化HTTP 服务器*/
void
httpserver_init()
{
  sys_thread_new("http_server_netconn", httpserver_thread, NULL, 1024, 4);
}

#endif /* LWIP_NETCONN*/
