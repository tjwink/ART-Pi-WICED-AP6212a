#include "wifi_base_config.h"
#include "appliance.h"
#include "stm32H7xx.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "httpserver-netconn.h"

#define PRINTF printf
/**
 * @brief 现象及实验说明
 *
 */
void ExperimentalDescription()
{
	PRINTF("本例程演示开发板做http服务器\n\n");

	PRINTF("实验中使用TCP协议传输数据，电脑浏览器作为TCP Client ，开发板作为TCP Server\n\n");

	PRINTF("热点的IP地址、端口号在wifi_base_config.h文件中修改\n\n");

	PRINTF("本例程参考lwip部分可以参考 <<LwIP应用实战开发指南>>第20章 HTTP 服务器\n\n");
	
	PRINTF("输入电脑浏览器的IP地址为 -> Network ready IP: *.*.*.*  \n\n");
	
	PRINTF("打开电脑浏览器，输入开发板的IP地址，按下回车即可\n\n");
	
}
/**
 * @brief app_main
 *
 */
void app_main( void )
{
	
		/*配置wifi lwip信息*/
		Config_WIFI_LwIP_Info();
	
		ExperimentalDescription();
	
		http_server_netconn_init();

}
