# ART-Pi Ap6212a驱动开源参考工程

本仓库基于野火h750PRO开发板提供的AP6181开源驱动移植，用于WICED移植参考.
修改主要集中于wifi_lwip_client工程和WiFi_SDK目录上，其他工程可以按照同样的方法修改移植。

# 野火WiFi模块AP6181-H750-PRO配套代码

### **[wifi-ap6181-h750-code](https://github.com/Embdefire/wifi-ap6181-h750-code)**包含了配套野火h750PRO开发板的wifi-ap6181示例程序。

程序配套代码目录如下：

| 序号 | 清单                                                         | 说明                                         |
| ---- | ------------------------------------------------------------ | -------------------------------------------- |
| 0    | [WiFi_SDK](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/WiFi_SDK) | 官方SDK                                      |
| 1    | [wifi_lwip_AP](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_AP) | 热点（AP）当有热点链接时会自动分配IP，并打印 |
| 2    | [wifi_lwip_client](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_client) | netcoon 客户端                               |
| 3    | [wifi_lwip_client_socket](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_client_socket) | socket 客户端                                |
| 4    | [wifi_lwip_dhcp](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_dhcp) | DHCP                                         |
| 5    | [wifi_lwip_dns](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_dns) | DNS                                          |
| 6    | [wifi_lwip_http](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_http) | 网页像是图片                                 |
| 7    | [wifi_lwip_http_led](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_http_led) | 网页控制开发板led                            |
| 8    | [wifi_lwip_iperf_all](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_iperf_all) | wifi测速                                     |
| 9    | [wifi_lwip_mqtt_ali_dht11](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_mqtt_ali_dht11) | 接入阿里云                                   |
| 10   | [wifi_lwip_mqtt_baidu_dht11_drive](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_mqtt_baidu_dht11_drive) | 接入百度云                                   |
| 11   | [wifi_lwip_onenet_mqtt_dht11](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_onenet_mqtt_dht11) | 接入OneNET                                   |
| 12   | [wifi_lwip_scan](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_scan) | 扫描周围wifi信息，并打印                     |
| 13   | [wifi_lwip_tcpecho](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_tcpecho) | netconn tcp                                  |
| 14   | [wifi_lwip_tcpecho_socket-ov2640](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_tcpecho_socket-ov2640) | wifi 网络摄像头（ov2640）                    |
| 15   | [wifi_lwip_tcpecho_socket-ov5640](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_tcpecho_socket-ov5640) | wifi 网络摄像头（ov5640）                    |
| 16   | [wifi_lwip_tcpecho_socket](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_tcpecho_socket) | socket tcp                                   |
| 17   | [wifi_lwip_tcpecho_socket_AP](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_tcpecho_socket_AP) | 开发板作为热点（服务器）                     |
| 18   | [wifi_lwip_udpecho](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_udpecho) | netconn udp                                  |
| 19   | [wifi_lwip_udpecho_socket](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_udpecho_socket) | socket udp                                   |
| 20   | [wifi_lwip_webclient](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/wifi_lwip_webclient) | 获取野火官网网页并打印                       |
| 21   | [网络调试助手-手机版（安卓）](https://github.com/Embdefire/wifi-ap6181-h750-code/tree/master/网络调试助手-手机版（安卓）) | 网络调试助手（手机版）                       |

### 使用野火开发板需要配置环境

（1）需要将开发板的天线连接好；

（2）将开发板的跳帽J78、J66安装好；

（3）需要将自己的电脑与开发板处于同一局域网下（部分例程需要连接外网）；

（4）以上三条是所有例程必须要配置的硬件环境，每个例程都有相应的必读说明，必读说明更加详细；

（5）例程是基于lwip 、 freertos 的，相关问题可以参考野火的书籍；

## 关于本项目

本项目通过git开源：

- github仓库地址： https://github.com/Embdefire/wifi-ap6181-h750-code 
- gitee仓库地址：https://gitee.com/wildfireteam/wifi-ap6181-h750-code
