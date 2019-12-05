#include "wifi_base_config.h"
#include "appliance.h"
#include "stm32h7xx.h"
#include "debug.h"
/* FreeRTOS头文件 */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "./sdram/bsp_sdram.h" 
#include "./camera/bsp_ov2640.h"
				
#include "camera_data_queue.h"
#include "tcp_server.h"
#include "stdlib.h"
#include "string.h"

				
				
/**
 * @brief app_main
 *
 */
#define fire_demo_log(M, ...) custom_log("WIFI", M, ##__VA_ARGS__)

extern uint32_t frame_counter;
extern int send_fream;
extern int cbReadFinish_num;
int time_miao=0;
void app_main( void )//0xD0000000
{
		host_thread_type_t    wwd_thread;
		camera_data * cambuf;
		int32_t err = kNoErr;

//		/*配置wifi lwip信息*/
		Config_WIFI_LwIP_Info();

		err = camera_queue_init();
	
		cambuf = cbWrite(&cam_circular_buff);
	
		err = open_camera((uint32_t *)cambuf->head, CAMERA_QUEUE_DATA_LEN);
//	
	
			/*配置wifi lwip信息*/
//		Config_WIFI_LwIP_Info();

//		err = camera_queue_init();
////	
//		cambuf = cbWrite(&cam_circular_buff);
//	
//		err = open_camera((uint32_t *)0xD0000000, CAMERA_QUEUE_DATA_LEN);
	
	
		printf("初始化 TCP_server\r\n");
	
		host_rtos_create_thread( &wwd_thread, (void *)tcp_server_thread, "TCP_server", NULL,1024*10, 1);

		frame_counter=0;//帧计数器清零
		send_fream=0;//帧计数器清零
    while(1)
    {
			/*延时*/
			time_miao++;
			vTaskDelay(1000);
			/*输出帧率*/
			printf("-time_miao-%d----->>>>>>>>frame_counter=%d fps/s ,send_fream ->%d fps/s \r\n",time_miao,frame_counter,send_fream);
			frame_counter=0;			
			send_fream=0;
    }

}

