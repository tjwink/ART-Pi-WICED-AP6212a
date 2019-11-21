#ifndef __MALLOC_H
#define __MALLOC_H
#include "stm32h7xx.h"

#include "lwipopts.h"


#define   MSG_MAX_LEN     500
#define   MSG_TOPIC_LEN   50
#define   KEEPLIVE_TIME   50
#define   MQTT_VERSION    4


/*以下信息需要自己注册并填写*/

/*ok*/
/*阿里云*/
//#if    LWIP_DNS
//#define   HOST_NAME       "a1f9KKUJPv0.iot-as-mqtt.cn-shanghai.aliyuncs.com"     //服务器域名
//#else
//#define   HOST_NAME       "139.196.135.135"     //服务器IP地址
//#endif


//#define   HOST_PORT     1883    //由于是TCP连接，端口必须是1883

//#define   CLIENT_ID     "12345|securemode=3,signmethod=hmacsha1,timestamp=10|"         //
//#define   USER_NAME     "fire_temp_hum&a1f9KKUJPv0"     //用户名
//#define   PASSWORD      "a6295691a034391c3431a38830c4d7ace972896d"  //秘钥

//#define   TOPIC         "/a1f9KKUJPv0/fire_temp_hum/user/temp_hum"      //订阅的主题
//#define   TEST_MESSAGE  "test_message"  //发送测试消息


///*OneNET*/
#if    LWIP_DNS
#define   HOST_NAME       "mqtt.heclouds.com"     //服务器域名
#else
#define   HOST_NAME       "183.230.40.39"     //服务器IP地址
#endif


#define   HOST_PORT     6002    

#define   CLIENT_ID     "548829788"         //
#define   USER_NAME     "278712"     //用户名
#define   PASSWORD      "12345"  //秘钥

#define   TOPIC         "temp_hum"      //订阅的主题
#define   TEST_MESSAGE  "test_message"  //发送测试消息



enum TopicType 
{
  TopicType1 = 1, 
  TopicType3 = 3, 
  TopicType5 = 5 
};


enum QoS 
{ 
  QOS0 = 0, 
  QOS1, 
  QOS2 
};

enum MQTT_Connect
{
  Connect_OK = 0,
  Connect_NOK,
  Connect_NOTACK
};

//数据交互结构体
typedef struct __MQTTMessage
{
    uint32_t qos;
    uint8_t retained;
    uint8_t dup;
    uint16_t id;
	  uint8_t type;
    void *payload;
    int32_t payloadlen;
}MQTTMessage;

//用户接收消息结构体
typedef struct __MQTT_MSG
{
	  uint8_t  msgqos;                 //消息质量
		uint8_t  msg[MSG_MAX_LEN];       //消息
	  uint32_t msglenth;               //消息长度
	  uint8_t  topic[MSG_TOPIC_LEN];   //主题    
	  uint16_t packetid;               //消息ID
	  uint8_t  valid;                  //标明消息是否有效
}MQTT_USER_MSG;

//发送消息结构体
typedef struct
{
    int8_t topic[MSG_TOPIC_LEN];
    int8_t qos;
    int8_t retained;

    uint8_t msg[MSG_MAX_LEN];
    uint8_t msglen;
} mqtt_recv_msg_t, *p_mqtt_recv_msg_t, mqtt_send_msg_t, *p_mqtt_send_msg_t;


void mqtt_thread( void *pvParameters);

/************************************************************************
** 函数名称: my_mqtt_send_pingreq								
** 函数功能: 发送MQTT心跳包
** 入口参数: 无
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注: 
************************************************************************/
int32_t MQTT_PingReq(int32_t sock);

/************************************************************************
** 函数名称: MQTT_Connect								
** 函数功能: 登录服务器
** 入口参数: int32_t sock:网络描述符
** 出口参数: Connect_OK:登陆成功 其他:登陆失败
** 备    注: 
************************************************************************/
uint8_t MQTT_Connect(void);

/************************************************************************
** 函数名称: MQTTSubscribe								
** 函数功能: 订阅消息
** 入口参数: int32_t sock：套接字
**           int8_t *topic：主题
**           enum QoS pos：消息质量
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注: 
************************************************************************/
int32_t MQTTSubscribe(int32_t sock,char *topic,enum QoS pos);

/************************************************************************
** 函数名称: UserMsgCtl						
** 函数功能: 用户消息处理函数
** 入口参数: MQTT_USER_MSG  *msg：消息结构体指针
** 出口参数: 无
** 备    注: 
************************************************************************/
void UserMsgCtl(MQTT_USER_MSG  *msg);

/************************************************************************
** 函数名称: GetNextPackID						
** 函数功能: 产生下一个数据包ID
** 入口参数: 无
** 出口参数: uint16_t packetid:产生的ID
** 备    注: 
************************************************************************/
uint16_t GetNextPackID(void);

/************************************************************************
** 函数名称: mqtt_msg_publish						
** 函数功能: 用户推送消息
** 入口参数: MQTT_USER_MSG  *msg：消息结构体指针
** 出口参数: >=0:发送成功 <0:发送失败
** 备    注: 
************************************************************************/
//int32_t MQTTMsgPublish(int32_t sock, char *topic, int8_t qos, int8_t retained,uint8_t* msg,uint32_t msg_len);
int32_t MQTTMsgPublish(int32_t sock, char *topic, int8_t qos, uint8_t* msg,uint16_t msg_len);
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
int32_t ReadPacketTimeout(int32_t sock,uint8_t *buf,int32_t buflen,uint32_t timeout);

/************************************************************************
** 函数名称: mqtt_pktype_ctl						
** 函数功能: 根据包类型进行处理
** 入口参数: uint8_t packtype:包类型
** 出口参数: 无
** 备    注: 
************************************************************************/
void mqtt_pktype_ctl(uint8_t packtype,uint8_t *buf,uint32_t buflen);

/************************************************************************
** 函数名称: WaitForPacket					
** 函数功能: 等待特定的数据包
** 入口参数: int32_t sock:网络描述符
**           uint8_t packettype:包类型
**           uint8_t times:等待次数
** 出口参数: >=0:等到了特定的包 <0:没有等到特定的包
** 备    注: 
************************************************************************/
int32_t WaitForPacket(int32_t sock,uint8_t packettype,uint8_t times);


void mqtt_thread_init(void);

#endif



