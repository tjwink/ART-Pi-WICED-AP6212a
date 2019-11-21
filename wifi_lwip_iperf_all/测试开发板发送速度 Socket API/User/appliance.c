#include "wifi_base_config.h"
#include "appliance.h"
#include "stm32H7xx.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "iperf_client.h"

#define PRINTF printf

/**
 * @brief app_main
 *
 */
void app_main( void )
{
	
		/*配置wifi lwip信息*/
		Config_WIFI_LwIP_Info();
	
		iperf_client_init();
}
