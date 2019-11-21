#include "mqttclient.h"
#include "transport.h"
#include "MQTTPacket.h"
#include "FreeRTOS.h"
#include "task.h"
#include "string.h"
#include "debug.h"
#include "lwip/sockets.h"

#include "lwip/opt.h"

#include "lwip/sys.h"
#include "lwip/api.h"

#include "lwip/sockets.h"

#include "cJSON_Process.h"
#include "./dht11/bsp_dht11.h"
#include "wifi_base_config.h"

/******************************* 全局变量声明 ************************************/
/*
 * 当我们在写应用程序的时候，可能需要用到一些全局变量。
 */
extern QueueHandle_t MQTT_Data_Queue;


//定义用户消息结构体
MQTT_USER_MSG  mqtt_user_msg;

int32_t MQTT_Socket = 0;


void deliverMessage(MQTTString *TopicName,MQTTMessage *msg,MQTT_USER_MSG *mqtt_user_msg);

/************************************************************************
** 函数名称: MQTT_Connect								
** 函数功能: 初始化客户端并登录服务器
** 入口参数: int32_t sock:网络描述符
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注: 
************************************************************************/
uint8_t MQTT_Connect(void)
{
    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    uint8_t buf[200];
    int buflen = sizeof(buf);
    int len = 0;
    data.clientID.cstring = CLIENT_ID;                   //随机
    data.keepAliveInterval = KEEPLIVE_TIME;         //保持活跃
    data.username.cstring = USER_NAME;              //用户名
    data.password.cstring = PASSWORD;               //秘钥
    data.MQTTVersion = MQTT_VERSION;                //3表示3.1版本，4表示3.11版本
    data.cleansession = 1;
    //组装消息
    len = MQTTSerialize_connect((unsigned char *)buf, buflen, &data);
    //发送消息
    transport_sendPacketBuffer(buf, len);

    /* 等待连接响应 */
    if (MQTTPacket_read(buf, buflen, transport_getdata) == CONNACK)
    {
        unsigned char sessionPresent, connack_rc;
        if (MQTTDeserialize_connack(&sessionPresent, &connack_rc, buf, buflen) != 1 || connack_rc != 0)
        {
          PRINT_DEBUG("无法连接，错误代码是: %d！\n", connack_rc);
            return Connect_NOK;
        }
        else 
        {
            PRINT_DEBUG("用户名与秘钥验证成功，MQTT连接成功！\n");
            return Connect_OK;
        }
    }
    else
        PRINT_DEBUG("MQTT连接无响应！\n");
        return Connect_NOTACK;
}


/************************************************************************
** 函数名称: MQTT_PingReq								
** 函数功能: 发送MQTT心跳包
** 入口参数: 无
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注: 
************************************************************************/
int32_t MQTT_PingReq(int32_t sock)
{
	  int32_t len;
		uint8_t buf[200];
		int32_t buflen = sizeof(buf);	 
		fd_set readfd;
	  struct timeval tv;
	  tv.tv_sec = 5;
	  tv.tv_usec = 0;
	
	  FD_ZERO(&readfd);
	  FD_SET(sock,&readfd);			
	
		len = MQTTSerialize_pingreq(buf, buflen);
		transport_sendPacketBuffer(buf, len);
	
		//等待可读事件
		if(select(sock+1,&readfd,NULL,NULL,&tv) == 0)
			return -1;
		
	  //有可读事件
		if(FD_ISSET(sock,&readfd) == 0)
			return -2;
		
		if(MQTTPacket_read(buf, buflen, transport_getdata) != PINGRESP)
			return -3;
		
		return 0;
	
}


/************************************************************************
** 函数名称: MQTTSubscribe								
** 函数功能: 订阅消息
** 入口参数: int32_t sock：套接字
**           int8_t *topic：主题
**           enum QoS pos：消息质量
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注: 
************************************************************************/
int32_t MQTTSubscribe(int32_t sock,char *topic,enum QoS pos)
{
	  static uint32_t PacketID = 0;
	  uint16_t packetidbk = 0;
	  int32_t conutbk = 0;
		uint8_t buf[100];
		int32_t buflen = sizeof(buf);
	  MQTTString topicString = MQTTString_initializer;  
		int32_t len;
	  int32_t req_qos,qosbk;
	
		fd_set readfd;
	  struct timeval tv;
	  tv.tv_sec = 2;
	  tv.tv_usec = 0;
	
	  FD_ZERO(&readfd);
	  FD_SET(sock,&readfd);		
	
	  //复制主题
    topicString.cstring = (char *)topic;
		//订阅质量
	  req_qos = pos;
	
	  //串行化订阅消息
    len = MQTTSerialize_subscribe(buf, buflen, 0, PacketID++, 1, &topicString, &req_qos);
		//发送TCP数据
	  if(transport_sendPacketBuffer(buf, len) < 0)
				return -1;
	  
    //等待可读事件--等待超时
		if(select(sock+1,&readfd,NULL,NULL,&tv) == 0)
				return -2;
		//有可读事件--没有可读事件
		if(FD_ISSET(sock,&readfd) == 0)
				return -3;

		//等待订阅返回--未收到订阅返回
		if(MQTTPacket_read(buf, buflen, transport_getdata) != SUBACK)
				return -4;	
		
		//拆订阅回应包
		if(MQTTDeserialize_suback(&packetidbk,1, &conutbk, &qosbk, buf, buflen) != 1)
				return -5;
		
		//检测返回数据的正确性
		if((qosbk == 0x80)||(packetidbk != (PacketID-1)))
				return -6;
		
    //订阅成功
		return 0;
}


/************************************************************************
** 函数名称: UserMsgCtl						
** 函数功能: 用户消息处理函数
** 入口参数: MQTT_USER_MSG  *msg：消息结构体指针
** 出口参数: 无
** 备    注: 
************************************************************************/
void UserMsgCtl(MQTT_USER_MSG  *msg)
{
		//这里处理数据只是打印，用户可以在这里添加自己的处理方式
	  PRINT_DEBUG("*****收到订阅的消息！******\n");
		//返回后处理消息
	  switch(msg->msgqos)
		{
			case 0:
				    PRINT_DEBUG("MQTT>>消息质量：QoS0\n");
				    break;
			case 1:
				    PRINT_DEBUG("MQTT>>消息质量：QoS1\n");
				    break;
			case 2:
				    PRINT_DEBUG("MQTT>>消息质量：QoS2\n");
				    break;
			default:
				    PRINT_DEBUG("MQTT>>错误的消息质量\n");
				    break;
		}
		PRINT_DEBUG("MQTT>>消息主题：%s\n",msg->topic);	
		PRINT_DEBUG("MQTT>>消息类容：%s\n",msg->msg);	
		PRINT_DEBUG("MQTT>>消息长度：%d\n",msg->msglenth);	 
    Proscess(msg->msg);
	  //处理完后销毁数据
	  msg->valid  = 0;
}

/************************************************************************
** 函数名称: GetNextPackID						
** 函数功能: 产生下一个数据包ID
** 入口参数: 无
** 出口参数: uint16_t packetid:产生的ID
** 备    注: 
************************************************************************/
uint16_t GetNextPackID(void)
{
	 static uint16_t pubpacketid = 0;
	 return pubpacketid++;
}
 /********************************************************
** 函数名称: mqtt_msg_publish
** 函数功能: 用户推送消息
** 入口参数: MQTT_USER_MSG  *msg：消息结构体指针
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注:
*******************************************************/
int32_t MQTTMsgPublish(int32_t sock,
                       char *topic,
                       int8_t qos,
                       uint8_t* msg,
                       uint16_t msg_len)
{
    int8_t retained = 0;      //保留标志位
    // uint32_t msg_len;         //数据长度
    uint8_t buf[MSG_MAX_LEN];
    int32_t buflen = sizeof(buf),len;
    MQTTString topicString = MQTTString_initializer;
    uint16_t packid = 0,packetidbk;

    //填充主题
    topicString.cstring = (char *)topic;

    //填充数据包ID
    if ((qos == QOS1)||(qos == QOS2))
    {
        packid = GetNextPackID();
    }
    else
    {
        qos = QOS0;
        retained = 0;
        packid = 0;
    }

    // msg_len = strlen((char *)msg);
    //推送消息
    len = MQTTSerialize_publish(buf, buflen, 0, qos, retained,
                                packid, topicString,
                                (unsigned char*)msg, msg_len);
    if (len <= 0)
        return -1;
    if (transport_sendPacketBuffer(buf, len) < 0)
        return -2;

    //质量等级0，不需要返回
    if (qos == QOS0)
    {
        return 0;
    }

    //等级1
    if (qos == QOS1)
    {
        //等待PUBACK
        if (WaitForPacket(sock,PUBACK,5) < 0)
            return -3;
        return 1;

    }
    //等级2
    if (qos == QOS2)
    {
        //等待PUBREC
        if (WaitForPacket(sock,PUBREC,5) < 0)
            return -3;
        //发送PUBREL
        len = MQTTSerialize_pubrel(buf, buflen,0, packetidbk);
        if (len == 0)
            return -4;
        if (transport_sendPacketBuffer(buf, len) < 0)
            return -6;
        //等待PUBCOMP
        if (WaitForPacket(sock,PUBREC,5) < 0)
            return -7;
        return 2;
    }
    //等级错误
    return -8;
}




/************************************************************************
 ** 函数名称: MQTTMsgPublish2dp
 ** 函数功能: 用户推送消息到'$dp'系统主题
 ** 入口参数: MQTT_USER_MSG  *msg：消息结构体指针
 ** 出口参数: >=0:发送成功 <0:发送失败
 ** 备    注:
 ************************************************************************/
 int32_t MQTTMsgPublish2dp(int32_t sock, int8_t qos, int8_t type,uint8_t* msg)
 {
     int32_t ret;
     uint16_t msg_len = 0;
     msg_len = strlen((char *)msg);
     uint8_t* q = pvPortMalloc(msg_len+3); //目前只支持1、3、4类型的json数据
     switch (type)
     {
     case TopicType1:
         *(uint8_t*)&q[0] = 0x01;
         break;
     case TopicType3:
         *(uint8_t*)&q[0] = 0x03;
         break;
     case TopicType5:
         *(uint8_t*)&q[0] = 0x05;
         break;
     default:
         goto publish2dpfail;
     }
     *(uint8_t*)&q[0] = 0x03;
     *(uint8_t*)&q[1] = ((msg_len)&0xff00)>>8;
     *(uint8_t*)&q[2] = (msg_len)&0xff;
     memcpy((uint8_t*)(&q[3]),(uint8_t*)msg,msg_len);
 
     //发布消息
     ret = MQTTMsgPublish(MQTT_Socket,(char*)"$dp",qos,(uint8_t*)q,msg_len+3);
 
 publish2dpfail:
     vPortFree(q);
     q = NULL;
     return ret;
 }

 
 
/************************************************************************
** 函数名称: ReadPacketTimeout					
** 函数功能: 阻塞读取MQTT数据
** 入口参数: int32_t sock:网络描述符
**           uint8_t *buf:数据缓存区
**           int32_t buflen:缓冲区大小
**           uint32_t timeout:超时时间--0-表示直接查询，没有数据立即返回
** 出口参数: -1：错误,其他--包类型
** 备    注: 
************************************************************************/
int32_t ReadPacketTimeout(int32_t sock,uint8_t *buf,int32_t buflen,uint32_t timeout)
{
		fd_set readfd;
	  struct timeval tv;
	  if(timeout != 0)
		{
				tv.tv_sec = timeout;
				tv.tv_usec = 0;
				FD_ZERO(&readfd);
				FD_SET(sock,&readfd); 

				//等待可读事件--等待超时
				if(select(sock+1,&readfd,NULL,NULL,&tv) == 0)
						return -1;
				//有可读事件--没有可读事件
				if(FD_ISSET(sock,&readfd) == 0)
						return -1;
	  }
		//读取TCP/IP事件
		return MQTTPacket_read(buf, buflen, transport_getdata);
}


/************************************************************************
** 函数名称: deliverMessage						
** 函数功能: 接受服务器发来的消息
** 入口参数: MQTTMessage *msg:MQTT消息结构体
**           MQTT_USER_MSG *mqtt_user_msg:用户接受结构体
**           MQTTString  *TopicName:主题
** 出口参数: 无
** 备    注: 
************************************************************************/
void deliverMessage(MQTTString  *TopicName,MQTTMessage *msg,MQTT_USER_MSG *mqtt_user_msg)
{
		//消息质量
		mqtt_user_msg->msgqos = msg->qos;
		//保存消息
		memcpy(mqtt_user_msg->msg,msg->payload,msg->payloadlen);
		mqtt_user_msg->msg[msg->payloadlen] = 0;
		//保存消息长度
		mqtt_user_msg->msglenth = msg->payloadlen;
		//消息主题
		memcpy((char *)mqtt_user_msg->topic,TopicName->lenstring.data,TopicName->lenstring.len);
		mqtt_user_msg->topic[TopicName->lenstring.len] = 0;
		//消息ID
		mqtt_user_msg->packetid = msg->id;
		//标明消息合法
		mqtt_user_msg->valid = 1;		
}


/************************************************************************
** 函数名称: mqtt_pktype_ctl						
** 函数功能: 根据包类型进行处理
** 入口参数: uint8_t packtype:包类型
** 出口参数: 无
** 备    注: 
************************************************************************/
void mqtt_pktype_ctl(uint8_t packtype,uint8_t *buf,uint32_t buflen)
{
	  MQTTMessage msg;
		int32_t rc;
	  MQTTString receivedTopic;
	  uint32_t len;
		switch(packtype)
		{
			case PUBLISH:
        //拆析PUBLISH消息
        if(MQTTDeserialize_publish(&msg.dup,(int*)&msg.qos, &msg.retained, &msg.id, &receivedTopic,
          (unsigned char **)&msg.payload, &msg.payloadlen, buf, buflen) != 1)
            return;	
        //接受消息
        deliverMessage(&receivedTopic,&msg,&mqtt_user_msg);
        
        //消息质量不同，处理不同
        if(msg.qos == QOS0)
        {
           //QOS0-不需要ACK
           //直接处理数据
           UserMsgCtl(&mqtt_user_msg);
           return;
        }
        //发送PUBACK消息
        if(msg.qos == QOS1)
        {
            len =MQTTSerialize_puback(buf,buflen,mqtt_user_msg.packetid);
            if(len == 0)
              return;
            //发送返回
            if(transport_sendPacketBuffer(buf,len)<0)
               return;	
            //返回后处理消息
            UserMsgCtl(&mqtt_user_msg); 
            return;												
        }

        //对于质量2,只需要发送PUBREC就可以了
        if(msg.qos == QOS2)
        {
           len = MQTTSerialize_ack(buf, buflen, PUBREC, 0, mqtt_user_msg.packetid);			                
           if(len == 0)
             return;
           //发送返回
           transport_sendPacketBuffer(buf,len);	
        }		
        break;
			case  PUBREL:				           
        //解析包数据，必须包ID相同才可以
        rc = MQTTDeserialize_ack(&msg.type,&msg.dup, &msg.id, buf,buflen);
        if((rc != 1)||(msg.type != PUBREL)||(msg.id != mqtt_user_msg.packetid))
          return ;
        //收到PUBREL，需要处理并抛弃数据
        if(mqtt_user_msg.valid == 1)
        {
           //返回后处理消息
           UserMsgCtl(&mqtt_user_msg);
        }      
        //串行化PUBCMP消息
        len = MQTTSerialize_pubcomp(buf,buflen,msg.id);	                   	
        if(len == 0)
          return;									
        //发送返回--PUBCOMP
        transport_sendPacketBuffer(buf,len);										
        break;
			case   PUBACK://等级1客户端推送数据后，服务器返回
				break;
			case   PUBREC://等级2客户端推送数据后，服务器返回
				break;
			case   PUBCOMP://等级2客户端推送PUBREL后，服务器返回
        break;
			default:
				break;
		}
}

/************************************************************************
** 函数名称: WaitForPacket					
** 函数功能: 等待特定的数据包
** 入口参数: int32_t sock:网络描述符
**           uint8_t packettype:包类型
**           uint8_t times:等待次数
** 出口参数: >=0:等到了特定的包 <0:没有等到特定的包
** 备    注: 
************************************************************************/
int32_t WaitForPacket(int32_t sock,uint8_t packettype,uint8_t times)
{
	  int32_t type;
		uint8_t buf[MSG_MAX_LEN];
	  uint8_t n = 0;
		int32_t buflen = sizeof(buf);
		do
		{
				//读取数据包
				type = ReadPacketTimeout(sock,buf,buflen,2);
			  if(type != -1)
					mqtt_pktype_ctl(type,buf,buflen);
				n++;
		}while((type != packettype)&&(n < times));
		//收到期望的包
		if(type == packettype)
			 return 0;
		else 
			 return -1;		
}



void Client_Connect(void)
{
    char* host_ip;
  
#ifdef  LWIP_DNS
    ip4_addr_t dns_ip;
    netconn_gethostbyname(HOST_NAME, &dns_ip);
    host_ip = ip_ntoa(&dns_ip);
    PRINT_DEBUG("host name : %s , host_ip : %s\n",HOST_NAME,host_ip);
#else
    host_ip = HOST_NAME;
#endif  
MQTT_START: 
  
		//创建网络连接
		PRINT_DEBUG("1.开始连接对应云平台的服务器...\n");
    PRINT_DEBUG("服务器IP地址：%s，端口号：%0d！\n",host_ip,HOST_PORT);
		while(1)
		{
				//连接服务器
				MQTT_Socket = transport_open((int8_t*)host_ip,HOST_PORT);
				//如果连接服务器成功
				if(MQTT_Socket >= 0)
				{
						PRINT_DEBUG("连接云平台服务器成功！\n");
						break;
				}
				PRINT_DEBUG("连接云平台服务器失败，等待3秒再尝试重新连接！\n");
				//等待3秒
				vTaskDelay(3000);
		}
    
    PRINT_DEBUG("2.MQTT用户名与秘钥验证登陆...\n");
    //MQTT用户名与秘钥验证登陆
    if(MQTT_Connect() != Connect_OK)
    {
         //重连服务器
         PRINT_DEBUG("MQTT用户名与秘钥验证登陆失败...\n");
          //关闭链接
         transport_close();
         goto MQTT_START;	 
    }
    
		//订阅消息
		PRINT_DEBUG("3.开始订阅消息...\n");
    //订阅消息
    if(MQTTSubscribe(MQTT_Socket,(char *)TOPIC,QOS1) < 0)
    {
         //重连服务器
         PRINT_DEBUG("客户端订阅消息失败...\n");
          //关闭链接
         transport_close();
         goto MQTT_START;	   
    }	

		//无限循环
		PRINT_DEBUG("4.开始循环接收订阅的消息...\n");

}





/************************************************************************
** 函数名称: mqtt_recv_thread								
** 函数功能: MQTT任务
** 入口参数: void *pvParameters：任务参数
** 出口参数: 无
** 备    注: MQTT连云步骤：
**           1.连接对应云平台的服务器
**           2.MQTT用户与秘钥验证登陆
**           3.订阅指定主题
**           4.等待接收主题的数据与上报主题数据
************************************************************************/
void mqtt_recv_thread(void *pvParameters)
{
	  uint32_t curtick;
		uint8_t no_mqtt_msg_exchange = 1;
		uint8_t buf[MSG_MAX_LEN];
		int32_t buflen = sizeof(buf);
    int32_t type;
    fd_set readfd;
	  struct timeval tv;      //等待时间
	  tv.tv_sec = 0;
	  tv.tv_usec = 10;

  
MQTT_START: 
    //开始连接
    Client_Connect();
    //获取当前滴答，作为心跳包起始时间
		curtick = xTaskGetTickCount();
		while(1)
		{
				//表明无数据交换
				no_mqtt_msg_exchange = 1;
			
				FD_ZERO(&readfd);
				FD_SET(MQTT_Socket,&readfd);						  

				//等待可读事件
				select(MQTT_Socket+1,&readfd,NULL,NULL,&tv);
				
				//判断MQTT服务器是否有数据
				if(FD_ISSET(MQTT_Socket,&readfd) != 0)
				{
						//读取数据包--注意这里参数为0，不阻塞
						type = ReadPacketTimeout(MQTT_Socket,buf,buflen,0);
						if(type != -1)
						{
								mqtt_pktype_ctl(type,buf,buflen);
								//表明有数据交换
								no_mqtt_msg_exchange = 0;
								//获取当前滴答，作为心跳包起始时间
								curtick = xTaskGetTickCount();
						}
				}
        
        //这里主要目的是定时向服务器发送PING保活命令
        if((xTaskGetTickCount() - curtick) >(KEEPLIVE_TIME/2*1000))
        {
            curtick = xTaskGetTickCount();
            //判断是否有数据交换
            if(no_mqtt_msg_exchange == 0)
            {
               //如果有数据交换，这次就不需要发送PING消息
               continue;
            }
            
            if(MQTT_PingReq(MQTT_Socket) < 0)
            {
               //重连服务器
               PRINT_DEBUG("发送保持活性ping失败....\n");
               goto CLOSE;	 
            }
            
            //心跳成功
            PRINT_DEBUG("发送保持活性ping作为心跳成功....\n");
            //表明有数据交换
            no_mqtt_msg_exchange = 0;
        }   
		}

CLOSE:
	 //关闭链接
	 transport_close();
	 //重新链接服务器
	 goto MQTT_START;	
}

void mqtt_send_thread(void *pvParameters)
{
    int32_t ret;
    uint8_t no_mqtt_msg_exchange = 1;
    uint32_t curtick;
    uint8_t res;
    /* 定义一个创建信息返回值，默认为pdTRUE */
    BaseType_t xReturn = pdTRUE;
    /* 定义一个接收消息的变量 */
//    uint32_t* r_data;	
    DHT11_Data_TypeDef* recv_data;
    //初始化json数据
    cJSON* cJSON_Data = NULL;
    cJSON_Data = cJSON_Data_Init();
    double a,b;
MQTT_SEND_START:
  
    while(1)
    {
        
    xReturn = xQueueReceive( MQTT_Data_Queue,    /* 消息队列的句柄 */
                             &recv_data,      /* 发送的消息内容 */
                             1000); /* 等待时间 3000ms */
      if(xReturn != pdTRUE)
      {
//        a = recv_data->temperature;
//        b = recv_data->humidity;
				a = recv_data->temp_int;
        b = recv_data->humi_int;
//        printf("a = %f,b = %f\n",a,b);
        //更新数据      
        res = cJSON_Update(cJSON_Data,TEMP_NUM,&a);
        res = cJSON_Update(cJSON_Data,HUM_NUM,&b);
      
        if(UPDATE_SUCCESS == res)
        {
            //更新数据成功，
            char* p = cJSON_Print(cJSON_Data);
            //发布消息到'$dp'系统主题
           ret = MQTTMsgPublish2dp(MQTT_Socket,QOS0,TopicType3,(uint8_t*)p);
            if(ret >= 0)
            {
                //表明有数据交换
                no_mqtt_msg_exchange = 0;
                //获取当前滴答，作为心跳包起始时间
                curtick = xTaskGetTickCount();				
            }
            vPortFree(p);
            p = NULL;
        }
        else
          PRINT_DEBUG("update fail\n");
      }
      //这里主要目的是定时向服务器发送PING保活命令
      if((xTaskGetTickCount() - curtick) >(KEEPLIVE_TIME/2*1000))
      {
          curtick = xTaskGetTickCount();
          //判断是否有数据交换
          if(no_mqtt_msg_exchange == 0)
          {
             //如果有数据交换，这次就不需要发送PING消息
             continue;
          }
          
          if(MQTT_PingReq(MQTT_Socket) < 0)
          {
             //重连服务器
             PRINT_DEBUG("发送保持活性ping失败....\n");
             goto MQTT_SEND_CLOSE;	 
          }
          
          //心跳成功
          PRINT_DEBUG("发送保持活性ping作为心跳成功....\n");
          //表明有数据交换
          no_mqtt_msg_exchange = 0;
      } 
  }
MQTT_SEND_CLOSE:
	 //关闭链接
	 transport_close(); 
   //开始连接
   Client_Connect();
   goto MQTT_SEND_START;
}

void
mqtt_thread_init(void)
{
  sys_thread_new("mqtt_recv_thread", mqtt_recv_thread, NULL, 2048, 6);
  sys_thread_new("mqtt_send_thread", mqtt_send_thread, NULL, 2048, 7);
}

