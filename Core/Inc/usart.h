/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.h
  * @brief   This file contains all the function prototypes for
  *          the usart.c file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#ifndef __USART_H__
#define __USART_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
#define USART_REC_LEN           200     /* 定义最大接收字节数 200 */
#define RXBUFFERSIZE            200     /* DMA接收缓存大小 */
#define TX_BUFFER_SIZE          10000    /* 发送环形缓冲区大小 */
#define MAX_DMA_TRANSFER        1024     /* 单次DMA最大传输量 */

  typedef struct {
    uint8_t buffer[TX_BUFFER_SIZE];     /* 数据缓冲区 */
    volatile uint16_t head;             /* 写指针 */
    volatile uint16_t tail;             /* 读指针 */
    volatile uint16_t count;            /* 当前数据量 */
    volatile uint8_t tx_busy;           /* DMA发送忙标志 */
  } ring_buffer_t;

  extern UART_HandleTypeDef huart1;
  extern DMA_HandleTypeDef hdma_usart1_tx;
  extern DMA_HandleTypeDef hdma_usart1_rx;
  extern uint8_t  g_usart_rx_buf[USART_REC_LEN];
  extern uint16_t g_usart_rx_sta;
  extern uint8_t  g_rx_buffer[RXBUFFERSIZE];
  extern ring_buffer_t tx_ring_buffer;
/* USER CODE END Private defines */

void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
  uint16_t uart_get_rx_status(void);
  void uart_clear_rx_status(void);
  uint8_t* uart_get_rx_buffer(void);
  uint16_t uart_get_rx_data_length(void);
  uint16_t ring_buffer_write(const uint8_t *data, uint16_t len);
  void start_dma_transmission(void);
  uint16_t get_buffer_free_space(void);
  uint16_t get_buffer_data_count(void);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __USART_H__ */

