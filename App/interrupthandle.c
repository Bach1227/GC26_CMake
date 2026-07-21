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
#include "config.h"
#include "usart.h"
#include "tim.h"
#include "fdcan.h"
#include "bsp_zdt.h"
#include "gimbal.h"
#include "statemachine.h"
#include <string.h>

#define START_BUTTON_DEBOUNCE_MS 50U

/* == WitGyro UART10 DMA 接收 (需在 RAM_D1, H7 DMA 无法访问 DTCM) == */
__attribute__((section(".RAM_D1"))) uint8_t gyro_rx_buf[256];

void Gyro_UART_Start(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&GYRO_UART_HANDLE, gyro_rx_buf, sizeof(gyro_rx_buf));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  static uint32_t last_button_tick = 0U;
  uint32_t now;

  if (GPIO_Pin != GPIO_PIN_15) {
    return;
  }


  now = HAL_GetTick();
  if (last_button_tick != 0U &&
      (uint32_t)(now - last_button_tick) < START_BUTTON_DEBOUNCE_MS) {
    return;
  }

  last_button_tick = now;
  (void)SM_SendEventFromISR(EVENT_START);
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart == &COMM_UART_HANDLE)
  {
    Comm_OnUartRx(Size);
  }

  if (huart == &GYRO_UART_HANDLE)
  {
      WitGyro_Parse(gyro_rx_buf, Size);
      HAL_UARTEx_ReceiveToIdle_DMA(&GYRO_UART_HANDLE, gyro_rx_buf, sizeof(gyro_rx_buf));
  }

}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) 
{
  if (huart->Instance == COMM_UART_INSTANCE) // 替换为你实际使用的串口
    {
        Comm_OnUartError();
    }

  if (huart->Instance == GYRO_UART_INSTANCE)
    {
        __HAL_UART_CLEAR_FLAG(huart, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);
        HAL_UART_AbortReceive_IT(huart);
        HAL_UARTEx_ReceiveToIdle_DMA(huart, gyro_rx_buf, sizeof(gyro_rx_buf));
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t               rx_data[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        return;

    if (hfdcan == &hfdcan1) {
        ZDT_OnRxMessage(rx_header.Identifier, rx_data, rx_header.DataLength);
    }

    if (hfdcan == &hfdcan2) {
        FDCAN_RxFrame_t frame;
        frame.ID   = rx_header.Identifier;
        frame.DLC  = rx_header.DataLength;
        memcpy(frame.data, rx_data, 8);
        Gimbal_OnCanRx(hfdcan, &frame);
    }
}
