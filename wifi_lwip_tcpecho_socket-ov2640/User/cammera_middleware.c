#include "cammera_middleware.h"

#include "camera_data_queue.h"
#include "wifi_base_config.h"
#include "stm32h7xx.h"
#include "stm32h7xx_hal.h"
uint32_t frame_counter = 0;
uint32_t 	XferSize = 0;

void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));
  assert_param(IS_FUNCTIONAL_STATE(NewState));

  if (NewState != DISABLE)
  {
    /* Enable the selected DMAy Streamx by setting EN bit */
    DMAy_Streamx->CR |= (uint32_t)DMA_SxCR_EN;
  }
  else
  {
    /* Disable the selected DMAy Streamx by clearing EN bit */
    DMAy_Streamx->CR &= ~(uint32_t)DMA_SxCR_EN;
  }
}


void DCMI_JPEGCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
 
  if (NewState != DISABLE)
  {
    /* Enable the DCMI JPEG format */
    DCMI->CR |= (uint32_t)DCMI_CR_JPEG;
  }
  else
  {
    /* Disable the DCMI JPEG format */
    DCMI->CR &= ~(uint32_t)DCMI_CR_JPEG;
  }
}




FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx)
{
  FunctionalState state = DISABLE;

  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));

  if ((DMAy_Streamx->CR & (uint32_t)DMA_SxCR_EN) != 0)
  {
    /* The selected DMAy Streamx EN bit is set (DMA is still transferring) */
    state = ENABLE;
  }
  else
  {
    /* The selected DMAy Streamx EN bit is cleared (DMA is disabled and 
        all transfers are complete) */
    state = DISABLE;
  }
  return state;
}


/**
  * @brief  Returns the number of remaining data units in the current DMAy Streamx transfer.
  * @param  DMAy_Streamx: where y can be 1 or 2 to select the DMA and x can be 0
  *          to 7 to select the DMA Stream.
  * @retval The number of remaining data units in the current DMAy Streamx transfer.
  */
uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx)
{
  /* Check the parameters */
  assert_param(IS_DMA_ALL_PERIPH(DMAy_Streamx));

  /* Return the number of remaining data units for DMAy Streamx */
  return ((uint16_t)(DMAy_Streamx->NDTR));
}

/**
  * @brief  Enables or disables the DCMI Capture.
  * @param  NewState: new state of the DCMI capture. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DCMI_CaptureCmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
  if (NewState != DISABLE)
  {
    /* Enable the DCMI Capture */
    DCMI->CR |= (uint32_t)DCMI_CR_CAPTURE;
  }
  else
  {
    /* Disable the DCMI Capture */
    DCMI->CR &= ~(uint32_t)DCMI_CR_CAPTURE;
  }
}
/**
  * @brief  Enables or disables the DCMI interface.
  * @param  NewState: new state of the DCMI interface. 
  *          This parameter can be: ENABLE or DISABLE.
  * @retval None
  */
void DCMI_Cmd(FunctionalState NewState)
{
  /* Check the parameters */
  assert_param(IS_FUNCTIONAL_STATE(NewState));
  
  if (NewState != DISABLE)
  {
    /* Enable the DCMI by setting ENABLE bit */
    DCMI->CR |= (uint32_t)DCMI_CR_ENABLE;
  }
  else
  {
    /* Disable the DCMI by clearing ENABLE bit */
    DCMI->CR &= ~(uint32_t)DCMI_CR_ENABLE;
  }
}



extern DCMI_HandleTypeDef DCMI_Handle;
extern DMA_HandleTypeDef DMA_Handle_dcmi;


void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi)
{
	DCMI_IRQHandler_Funtion();

   //重新使能帧中断
  __HAL_DCMI_ENABLE_IT(&DCMI_Handle,DCMI_IT_FRAME);
}

void HAL_DCMI_VsyncEventCallback(DCMI_HandleTypeDef *hdcmi)
{
		DCMI_IRQHandler_Funtion();

		//重新使能帧中断
		__HAL_DCMI_ENABLE_IT(&DCMI_Handle,DCMI_FLAG_VSYNCRI);
}

void DCMI_Start(void)
{
	
	camera_data *data_p;
	
	/*获取写缓冲区指针，准备写入新数据*/
	data_p = cbWrite(&cam_circular_buff);
	
	if (data_p != NULL)	//若缓冲队列未满，开始传输
	{		
		HAL_DCMI_Stop(&DCMI_Handle) ;
		/*配置DMA传输*/
		HAL_DCMI_Start_DMA(&DCMI_Handle, DCMI_MODE_CONTINUOUS, (uint32_t )data_p->head, CAMERA_QUEUE_DATA_LEN);

		DMA_Cmd(DMA2_Stream1, ENABLE);			//重新传输	
	}

}


void DCMI_Stop(void)
{	
	camera_data *data_p;
	
	/*关闭dma*/
	DMA_Cmd(DMA2_Stream1,DISABLE);
	while(DMA_GetCmdStatus(DMA2_Stream1) != DISABLE){}

	/*获取正在操作的写指针*/	
	data_p = cbWriteUsing(&cam_circular_buff);

	/*计算dma传输的数据个数，用于减少jpeg搜索文件尾的时间*/	
	if (data_p != NULL)	
	{
		data_p->img_dma_len =0; //复位	
		XferSize = DCMI_Handle.XferSize;//HAL库里面获取 长度
		data_p->img_dma_len = (XferSize - DMA_GetCurrDataCounter(DMA2_Stream1))*4; //最后一个包 __HAL_DMA_GET_COUNTER			

	}
	
	/*写入缓冲区完毕*/
	cbWriteFinish(&cam_circular_buff);
}



/*帧中断实现*/
void DCMI_IRQHandler_Funtion(void)
{

	frame_counter ++;
	//1.停止DCMI传输
	DCMI_Stop();
	//2.根据缓冲区使用情况决定是否开启dma
	DCMI_Start();
	
}
