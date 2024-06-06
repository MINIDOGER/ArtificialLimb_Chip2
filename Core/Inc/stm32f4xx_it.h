/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.h
  * @brief   This file contains the headers of the interrupt handlers.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
 ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
#define MAX_RX_LEN (256U) // 一次性可以接受的数据字节长度，你可以自己定义。U是Unsigned的意思。
#define MAX_TX_LEN (512U) // 一次性可以发送的数据字节长度，你可以自己定义。
/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */
 extern uint8_t *p_IsOK1;
 extern uint8_t *p_IsToReceive1;
 extern uint8_t *p_IsOK3;
 extern uint8_t *p_IsToReceive3;
 extern uint8_t *p_IsOK4;
 extern uint8_t *p_IsToReceive4;
 extern uint8_t *p_IsOK5;
 extern uint8_t *p_IsToReceive5;
 extern uint8_t *p_IsOK6;
 extern uint8_t *p_IsToReceive6;
/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void DMA1_Stream0_IRQHandler(void);
void DMA1_Stream1_IRQHandler(void);
void DMA1_Stream2_IRQHandler(void);
void DMA1_Stream6_IRQHandler(void);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void DMA2_Stream1_IRQHandler(void);
void DMA2_Stream2_IRQHandler(void);
void USART6_IRQHandler(void);
/* USER CODE BEGIN EFP */
void DMA_USART1_Tx_Data(uint8_t *buffer, uint16_t size);//数组发送串口数据
void DMA_usart1_printf(char *format, ...);//仿制printf发送串口数据
void DMA_USART2_Tx_Data(uint8_t *buffer, uint16_t size);//数组发送串口数据
void DMA_usart2_printf(char *format, ...);//仿制printf发送串口数据
void USART1_TX_Wait(void);//发送等待函数
void USART2_TX_Wait(void);//发送等待函数
void MPU6050ModDataBufDMA(struct Data *AllData, int Joint);
void FootDataBufDMA(struct DataUnionBuf *AllData);
void low_pass_filter_init(void);
float low_pass_filter(float value, int Joint);
/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F4xx_IT_H */
