#include "wifi_base_config.h"
#include "appliance.h"
#include "stm32H7xx.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "udpecho.h"

#define PRINTF printf

/**
 * @brief app_main
 *
 */
void app_main( void )
{
	
		/*配置wifi lwip信息*/
		Config_WIFI_LwIP_Info();
		printf("\r\n请使用网络调试助手连接\r\n");
		printf("\r\n连接成功后，发送数据即可得到回显\r\n");
	
		udpecho_init();
}
