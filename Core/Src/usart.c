/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
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
/* Includes ------------------------------------------------------------------*/
#include "usart.h"
#include <string.h>

/* USER CODE BEGIN 0 */
uint8_t  g_usart_rx_buf[USART_REC_LEN];
uint16_t g_usart_rx_sta = 0;
uint8_t  g_rx_buffer[RXBUFFERSIZE];  /* DMA接收缓冲区 */

/* 🎯 新增：DMA接收管理变量 */
volatile uint16_t g_rx_write_pos = 0;     /* 当前写入位置 */
volatile uint16_t g_rx_last_pos = 0;      /* 上次处理位置 */
volatile uint8_t  g_rx_frame_ready = 0;   /* 接收帧就绪标志 */

/* 发送环形缓冲区 */
ring_buffer_t tx_ring_buffer = {0};
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clocks
  */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Channel3;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1_TX Init */
    hdma_usart1_tx.Instance = DMA1_Channel4;
    hdma_usart1_tx.Init.Request = DMA_REQUEST_USART1_TX;
    hdma_usart1_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
    hdma_usart1_tx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_tx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_tx.Init.Mode = DMA_NORMAL;
    hdma_usart1_tx.Init.Priority = DMA_PRIORITY_HIGH;
    if (HAL_DMA_Init(&hdma_usart1_tx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmatx,hdma_usart1_tx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PB6     ------> USART1_TX
    PB7     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_6|GPIO_PIN_7);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);
    HAL_DMA_DeInit(uartHandle->hdmatx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint16_t get_buffer_free_space(void)
{
  return TX_BUFFER_SIZE - tx_ring_buffer.count;
}

uint16_t get_buffer_data_count(void)
{
  return tx_ring_buffer.count;
}

uint16_t ring_buffer_write(const uint8_t *data, uint16_t len)
{
  uint16_t written = 0;

  /* 禁用中断保证原子操作 */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  for (uint16_t i = 0; i < len; i++) {
    if (tx_ring_buffer.count < TX_BUFFER_SIZE) {
      tx_ring_buffer.buffer[tx_ring_buffer.head] = data[i];
      tx_ring_buffer.head = (tx_ring_buffer.head + 1) % TX_BUFFER_SIZE;
      tx_ring_buffer.count++;
      written++;
    } else {
      /* 缓冲区满了，丢弃剩余数据 */
      break;
    }
  }

  /* 恢复中断状态 */
  __set_PRIMASK(primask);

  return written;
}

void start_dma_transmission(void)
{
  /* 如果DMA忙或没有数据，直接返回 */
  if (tx_ring_buffer.tx_busy || tx_ring_buffer.count == 0) {
    return;
  }

  /* 计算本次发送的数据量 */
  uint16_t send_len = tx_ring_buffer.count;
  if (send_len > MAX_DMA_TRANSFER) {
    send_len = MAX_DMA_TRANSFER;  /* 限制单次发送量 */
  }

  /* 处理环形缓冲区的回绕情况 */
  uint16_t linear_len = TX_BUFFER_SIZE - tx_ring_buffer.tail;
  if (send_len > linear_len) {
    send_len = linear_len;  /* 只发送到缓冲区末尾 */
  }

  /* 标记DMA忙 */
  tx_ring_buffer.tx_busy = 1;

  /* 启动DMA发送 */
  if (HAL_UART_Transmit_DMA(&huart1,
                            &tx_ring_buffer.buffer[tx_ring_buffer.tail],
                            send_len) != HAL_OK) {
    /* DMA启动失败，清除忙标志 */
    tx_ring_buffer.tx_busy = 0;
                            }
}

void process_rx_data(void)
{
  /* 获取当前DMA写入位置 */
  uint16_t current_pos = RXBUFFERSIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

  if (current_pos != g_rx_last_pos) {
    /* 有新数据到达 */
    if (current_pos > g_rx_last_pos) {
      /* 数据连续，直接处理 */
      uint16_t data_len = current_pos - g_rx_last_pos;

      /* 检查是否有完整帧 */
      for (uint16_t i = g_rx_last_pos; i < current_pos; i++) {
        if (g_rx_buffer[i] == 0x0D) {
          /* 找到帧尾标志 */
          if (i + 1 < current_pos && g_rx_buffer[i + 1] == 0x0A) {
            /* 完整的帧结束 */
            uint16_t frame_len = i - g_rx_last_pos;
            if (frame_len < USART_REC_LEN) {
              memcpy(g_usart_rx_buf, &g_rx_buffer[g_rx_last_pos], frame_len);
              g_usart_rx_sta = 0x8000 | frame_len;  /* 标记接收完成 */
            }
            g_rx_last_pos = i + 2;  /* 跳过 \r\n */
            break;
          }
        }
      }

      if ((g_usart_rx_sta & 0x8000) == 0) {
        /* 没有完整帧，更新位置 */
        g_rx_last_pos = current_pos;
      }
    } else {
      /* 缓冲区回绕，先处理到末尾的数据 */
      /* 这里简化处理，直接更新位置 */
      g_rx_last_pos = current_pos;
    }
  }
}

uint16_t uart_get_rx_data_length(void)
{
  uint16_t current_pos = RXBUFFERSIZE - __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);

  if (current_pos >= g_rx_last_pos) {
    return current_pos - g_rx_last_pos;
  } else {
    return (RXBUFFERSIZE - g_rx_last_pos) + current_pos;
  }
}

int _write(int file, char *ptr, int len)
{
  /* 🎯 关键：直接写入环形缓冲区，不等待任何东西！ */
  uint16_t written = ring_buffer_write((uint8_t*)ptr, len);

  /* 尝试启动DMA发送（如果空闲的话） */
  start_dma_transmission();

  /* 立即返回，绝不阻塞！ */
  return written;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    /* 获取刚才发送的数据量 */
    uint16_t sent_len = huart->TxXferSize;

    /* 原子更新环形缓冲区指针 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    tx_ring_buffer.tail = (tx_ring_buffer.tail + sent_len) % TX_BUFFER_SIZE;
    tx_ring_buffer.count -= sent_len;
    tx_ring_buffer.tx_busy = 0;  /* 清除忙标志 */

    __set_PRIMASK(primask);

    /* 🚀 关键：如果还有数据，立即启动下一次发送 */
    start_dma_transmission();
  }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    /* 处理前半部分接收的数据 */
    process_rx_data();
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) {
    /* 处理后半部分接收的数据 */
    process_rx_data();
  }
}

uint16_t uart_get_rx_status(void)
{
  /* 🎯 实时处理DMA接收数据 */
  process_rx_data();
  return g_usart_rx_sta;
}

void uart_clear_rx_status(void)
{
  g_usart_rx_sta = 0;
}

uint8_t* uart_get_rx_buffer(void)
{
  return g_usart_rx_buf;
}
/* USER CODE END 1 */
