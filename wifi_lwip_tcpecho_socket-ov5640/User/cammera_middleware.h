
#ifndef CAMMERA_MIDDLEWARE_H
#define CAMMERA_MIDDLEWARE_H
#include "./camera/bsp_ov5640.h"
void DCMI_JPEGCmd(FunctionalState NewState);


 
 //uint32_t img_real_len=0;
 
extern uint32_t  XferTransferNumber;
extern uint32_t 	XferCount ;
extern uint32_t 	XferSize ;
extern uint32_t 	pBuffPtr ;
extern uint8_t 	DCMI_State;
extern uint32_t	DMA2_Stream1_State;
extern void DMA_Cmd(DMA_Stream_TypeDef* DMAy_Streamx, FunctionalState NewState)    ;
extern void DCMI_JPEGCmd(FunctionalState NewState)                                 ;
extern FunctionalState DMA_GetCmdStatus(DMA_Stream_TypeDef* DMAy_Streamx)          ;
extern uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx)           ;
extern void DCMI_CaptureCmd(FunctionalState NewState)                              ;

extern void DCMI_Start(void)	                                                      ;
extern void DCMI_Stop(void)                                                        ;
extern void DCMI_Cmd(FunctionalState NewState);
extern void DCMI_IRQHandler_Funtion(void);

#endif 

