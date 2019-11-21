#ifndef __DHT11_H_
#define __DHT11_H_

#include "stm32h7xx.h"

#define DHT11_HIGH  GPIO_PIN_SET
#define DHT11_LOW   GPIO_PIN_RESET


#define Bit_RESET   0
#define Bit_SET     1
/*---------------------------------------*/
#define DHT11_PORT                  GPIOA
#define DHT11_PIN                   GPIO_PIN_12            
#define DHT11_GPIO_CLK_ENABLE()     __GPIOA_CLK_ENABLE()


#define  DHT11_DATA_IN()	          HAL_GPIO_ReadPin(DHT11_PORT, DHT11_PIN)
#define  DHT11_DATA_OUT(value)	    HAL_GPIO_WritePin(DHT11_PORT,DHT11_PIN,value)

typedef struct
{
	uint8_t  humi_int;		//湿度的整数部分
	uint8_t  humi_deci;	 	//湿度的小数部分
	uint8_t  temp_int;	 	//温度的整数部分
	uint8_t  temp_deci;	 	//温度的小数部分
	uint8_t  check_sum;	 	//校验和
}DHT11_Data_TypeDef;

void DHT11_GPIO_Config(void);

uint8_t Read_DHT11(DHT11_Data_TypeDef *DHT11_Data);


#endif //__DHT11_H_

