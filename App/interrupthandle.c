/**
  ******************************************************************************
  * @file           : interrupthandle.c
  * @brief          : 中断处理函数实现
  *
  *  外设中断的第二层处理逻辑
  ******************************************************************************
  */

#include "interrupthandle.h"
#include "comm_manager.h"
#include "usart.h"
#include "tim.h"
#include "fdcan.h"
#include "bsp_zdt.h"


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }

  if (htim == &htim2)
  {
    /* code */
  }

  if (htim == &htim3)
  {
    /* code */
  }
}

uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &huart1)
  {

    Comm_Depack(Size);
    // uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
    // HAL_UART_Transmit(&huart1, data, sizeof(data), 0);
    // osDelay(10);
    // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, dma_rx_buf, 256);
  }

  if (huart == &huart7)
  {

    Comm_Depack(Size);
    // HAL_UART_Transmit_DMA(&huart7, data, sizeof(data));
    // osDelay(10);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, dma_rx_buf, 256);
  }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1) // 替换为你实际使用的串口
    {
        // 1. 获取错误代码
        uint32_t error_code = huart->ErrorCode;
        
        // 2. 清除错误标志位 (根据不同的芯片系列，宏定义可能略有不同，如 __HAL_UART_CLEAR_OREFLAG)
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        
        // 3. 终止当前的错误传输状态
        HAL_UART_AbortReceive_IT(huart); 
        // 如果用的是 DMA，也可以用 HAL_UART_AbortReceive(huart);
        
        // 4. 重新开启你的 Idle 接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_rx_buf, 256); 
    }

  if (huart->Instance == UART7) // 替换为你实际使用的串口
    {
        // 1. 获取错误代码
        uint32_t error_code = huart->ErrorCode;
        
        // 2. 清除错误标志位 (根据不同的芯片系列，宏定义可能略有不同，如 __HAL_UART_CLEAR_OREFLAG)
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        
        // 3. 终止当前的错误传输状态
        HAL_UART_AbortReceive_IT(huart); 
        // 如果用的是 DMA，也可以用 HAL_UART_AbortReceive(huart);
        
        // 4. 重新开启你的 Idle 接收
        HAL_UARTEx_ReceiveToIdle_DMA(huart, dma_rx_buf, 256); 
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t               rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;

    ZDT_OnRxMessage(rx_header.Identifier, rx_data, rx_header.DataLength);
}