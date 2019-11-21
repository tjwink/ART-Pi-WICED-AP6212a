#include "wifi_base_config.h"
#include "appliance.h"
#include "stm32h7xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "mqttclient.h"
#include "cJSON_Process.h"
#include "./delay/core_delay.h"    
#include "./dht11/bsp_dht11.h"
#include "./led/bsp_led.h"
#include "debug.h"

#define PRINTF printf
/**************************** 任务句柄 ********************************/
/* 
 * 任务句柄是一个指针，用于指向一个任务，当任务创建好之后，它就具有了一个任务句柄
 * 以后我们要想操作这个任务都需要通过这个任务句柄，如果是自身的任务操作自己，那么
 * 这个句柄可以为NULL。
 */
static TaskHandle_t AppTaskCreate_Handle = NULL;/* 创建任务句柄 */
static TaskHandle_t Test1_Task_Handle = NULL;/* LED任务句柄 */
static TaskHandle_t Test2_Task_Handle = NULL;/* KEY任务句柄 */

/********************************** 内核对象句柄 *********************************/
/*
 * 信号量，消息队列，事件标志组，软件定时器这些都属于内核的对象，要想使用这些内核
 * 对象，必须先创建，创建成功之后会返回一个相应的句柄。实际上就是一个指针，后续我
 * 们就可以通过这个句柄操作这些内核对象。
 *
 * 内核对象说白了就是一种全局的数据结构，通过这些数据结构我们可以实现任务间的通信，
 * 任务间的事件同步等各种功能。至于这些功能的实现我们是通过调用这些内核对象的函数
 * 来完成的
 * 
 */
 
QueueHandle_t MQTT_Data_Queue =NULL;

/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
DHT11_Data_TypeDef DHT11_Data;

/******************************* 宏定义 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些宏定义。
 */
#define  MQTT_QUEUE_LEN    4   /* 队列的长度，最大可包含多少个消息 */
#define  MQTT_QUEUE_SIZE   4   /* 队列中每个消息大小（字节） */

/*
*************************************************************************
*                             函数声明
*************************************************************************
*/
static void AppTaskCreate(void);/* 用于创建任务 */

static void Test1_Task(void* pvParameters);/* Test1_Task任务实现 */
static void Test2_Task(void* pvParameters);/* Test2_Task任务实现 */
/**
 * @brief app_main
 *
 */
void app_main( void )
{
	
		/*配置wifi lwip信息*/
		Config_WIFI_LwIP_Info();
	
		AppTaskCreate();
	
  /* 正常不会执行到这里 */    
    while (1)
    {
    }

}

/***********************************************************************
  * @ 函数名  ： AppTaskCreate
  * @ 功能说明： 为了方便管理，所有的任务创建函数都放在这个函数里面
  * @ 参数    ： 无  
  * @ 返回值  ： 无
  **********************************************************************/
static void AppTaskCreate(void)
{
  BaseType_t xReturn = pdPASS;/* 定义一个创建信息返回值，默认为pdPASS */
 
  /* 创建Test_Queue */
  MQTT_Data_Queue = xQueueCreate((UBaseType_t ) MQTT_QUEUE_LEN,/* 消息队列的长度 */
                                 (UBaseType_t ) MQTT_QUEUE_SIZE);/* 消息的大小 */
  if(NULL != MQTT_Data_Queue)
    PRINTF("创建MQTT_Data_Queue消息队列成功!\r\n");
    
  PRINTF("本例程使用开发板接入百度云，并且上报温湿度数据\n\n");
  
  PRINTF("网络连接模型如下：\n\t 电脑<--网线-->路由<--网线-->开发板\n\n 路由器必须能连接到外网(能上网)\n\n");
  
  PRINTF("实验中使用MQTT协议传输数据(依赖TCP协议) ，开发板作为MQTT Client\n\n");
  
  PRINTF("本例程的IP地址均在User/arch/sys_arch.h文件中修改\n\n");
    
  PRINTF("本例程LWIP部分可以参考<<LwIP应用实战开发指南>>第22章 连接到百度天工物接入\n\n");
   
  PRINTF("在开发板的dht11接口接上DHT11温湿度传感器\n\n");  
  
  PRINTF("基本信息(根据在百度云创建的设备进行修改，具体参考书籍第22章 连接到百度天工物接入)\n\n");
  
  PRINTF("百度云域名/IP地址 : %s \t 端口号 : %d \n\n",HOST_NAME,HOST_PORT);  
  
  PRINTF("百度云CLIENT_ID : %s\n\n",CLIENT_ID); 
  
  PRINTF("百度云USER_NAME : %s\n\n",USER_NAME); 
  
  PRINTF("百度云PASSWORD : %s\n\n",PASSWORD);  
  
  PRINTF("百度云TOPIC : %s\n\n",TOPIC);  
  
  PRINTF("百度云TEST_MESSAGE : %s\n\n",TEST_MESSAGE);    
  
  mqtt_thread_init();

  taskENTER_CRITICAL();           //进入临界区
 
  /* 创建Test1_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )Test1_Task, /* 任务入口函数 */
                        (const char*    )"Test1_Task",/* 任务名字 */
                        (uint16_t       )1024,   /* 任务栈大小 */
                        (void*          )NULL,	/* 任务入口函数参数 */
                        (UBaseType_t    )2,	    /* 任务的优先级 */
                        (TaskHandle_t*  )&Test1_Task_Handle);/* 任务控制块指针 */
  if(pdPASS == xReturn)
    PRINT_DEBUG("Create Test1_Task sucess...\r\n");
  
  /* 创建Test2_Task任务 */
  xReturn = xTaskCreate((TaskFunction_t )Test2_Task,  /* 任务入口函数 */
                        (const char*    )"Test2_Task",/* 任务名字 */
                        (uint16_t       )512,  /* 任务栈大小 */
                        (void*          )NULL,/* 任务入口函数参数 */
                        (UBaseType_t    )1, /* 任务的优先级 */
                        (TaskHandle_t*  )&Test2_Task_Handle);/* 任务控制块指针 */ 
  if(pdPASS == xReturn)
    PRINT_DEBUG("Create Test2_Task sucess...\n\n");
  
  vTaskDelete(AppTaskCreate_Handle); //删除AppTaskCreate任务
  
  taskEXIT_CRITICAL();            //退出临界区
}



/**********************************************************************
  * @ 函数名  ： Test1_Task
  * @ 功能说明： Test1_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Test1_Task(void* parameter)
{	
  uint8_t res;
  //DHT11初始化
  DHT11_GPIO_Config();
  DHT11_Data_TypeDef* send_data;
  while (1)
  {
    taskENTER_CRITICAL();           //进入临界区
    res = Read_DHT11(&DHT11_Data);
    taskEXIT_CRITICAL();            //退出临界区
    send_data = &DHT11_Data;
    if(SUCCESS == res)
    {
//      PRINTF("humidity = %f , temperature = %f\n",
//             (float)DHT11_Data.humi_int,(float)DHT11_Data.temp_int);
      xQueueSend( MQTT_Data_Queue, /* 消息队列的句柄 */
                  &send_data,/* 发送的消息内容 */
                  0 );        /* 等待时间 0 */
    }

    vTaskDelay(1000);/* 延时1000个tick */
  }

}

/**********************************************************************
  * @ 函数名  ： Test2_Task
  * @ 功能说明： Test2_Task任务主体
  * @ 参数    ：   
  * @ 返回值  ： 无
  ********************************************************************/
static void Test2_Task(void* parameter)
{	 
  while (1)
  {
    LED2_TOGGLE;
    vTaskDelay(2000);/* 延时2000个tick */
  }
}

/********************************END OF FILE****************************/





