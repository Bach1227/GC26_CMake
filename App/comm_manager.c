/**
  ******************************************************************************
  * @file           : comm_manager.c
  * @brief          : DMA+IDLE 接收 → RingBuffer 零拷贝 → 协议解析
  *
  *  DMA 循环写入 dma_rx_buf, ISR 更新 write_index,
  *  Comm_Task 调用 RingBuffer_Parse → Protocol_ParseBuffer 零拷贝解析.
  ******************************************************************************
  */

#include "comm_manager.h"
#include "usart.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "DataStructure.h"
#include "protocol.h"

/* ====================================================================== */
/*  常量                                                                  */
/* ====================================================================== */

#define DMA_RX_BUF_SIZE    256u

/* ====================================================================== */
/*  静态变量                                                              */
/* ====================================================================== */

__attribute__((section(".RAM_D1"))) uint8_t       dma_rx_buf[256];
static RingBuffer_t  uart_rb;
static TaskHandle_t  comm_task_handle = NULL;
static volatile uint8_t uart_rx_error_flag = 0;   /* ISR → Task 错误通知 */
ProtocolFrame_t frame;

/* 发送缓冲区 */
static uint8_t tx_frame_buf[PROTOCOL_MIN_FRAME_LEN + PROTOCOL_MAX_PAYLOAD];

extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart7;

void Comm_Task(void *argument);
TaskHandle_t Comm_TaskHandle;

/* ====================================================================== */
/*  Comm_Init                                                             */
/* ====================================================================== */

void Comm_Init(void)
{
    // RingBuffer_Init(&uart_rb, dma_rx_buf, DMA_RX_BUF_SIZE);
    // HAL_UART_Receive_DMA(&huart7, dma_rx_buf, DMA_RX_BUF_SIZE);
    // __HAL_UART_ENABLE_IT(&huart7, UART_IT_IDLE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart7);
    // 2. 清除可能残留的溢出、噪声等错误标志位
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF | UART_CLEAR_NEF | UART_CLEAR_PEF | UART_CLEAR_FEF);

    // 3. 延时等待电平彻底稳定（见下文说明）
    // HAL_Delay(50);

    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, dma_rx_buf, 256);
}

void Comm_RestartRx(void)
{
    /* 中止当前接收, 清理 DMA + UART 内部状态机 */
    HAL_UART_AbortReceive(&huart7);

    /* 清所有 UART 错误标志，否则会立即再次触发错误中断 */
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_PEF);
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_FEF);
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_NEF);
    __HAL_UART_CLEAR_FLAG(&huart7, UART_CLEAR_OREF);

    /* 重置环形缓冲区，丢弃错误前可能损坏的数据 */
    RingBuffer_Init(&uart_rb, dma_rx_buf, DMA_RX_BUF_SIZE);

    /* 重启 DMA + IDLE 接收 */
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, dma_rx_buf, DMA_RX_BUF_SIZE);
}

/**
 * ISR 上下文调用: 仅设置错误标志并通知 Comm_Task, 不在此处重启 DMA
 */
void Comm_OnUartError(void)
{
    uart_rx_error_flag = 1;

    if (comm_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(comm_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void Comm_InitTask(void)
{
    // Comm_Init();
    xTaskCreate(Comm_Task, "comm_task", 256, NULL, osPriorityAboveNormal1, &Comm_TaskHandle);
}

/* ====================================================================== */
/*  Comm_OnUartIdle (ISR 上下文)                                           */
/* ====================================================================== */

void Comm_OnUartIdle(uint16_t received_size)
{
    uart_rb.write_index = received_size % DMA_RX_BUF_SIZE;

    if (comm_task_handle != NULL) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        vTaskNotifyGiveFromISR(comm_task_handle, &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/* ====================================================================== */
/*  Comm_Task (FreeRTOS 线程)                                             */
/* ====================================================================== */

void Comm_Task(void *argument)
{
    (void)argument;
    comm_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        // while (RingBuffer_Parse(&uart_rb, Protocol_ParseBuffer) > 0);

        // ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // if (uart_rx_error_flag) {
        //     uart_rx_error_flag = 0;
        //     Comm_RestartRx();
        // }
        // uint8_t data[4] = { 0x01, 0x02, 0x03, 0x04 };
        // HAL_UART_Transmit(&huart7, data, sizeof(data), 10);
        osDelay(10);
    }
}

/* ====================================================================== */
/*  发送函数                                                              */
/* ====================================================================== */

uint16_t Comm_SendFrame(const uint8_t *data, uint16_t len)
{
    if (data == NULL || len == 0) return 0;

    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart7, (uint8_t *)data,
                                                  len, HAL_MAX_DELAY);
    return (status == HAL_OK) ? len : 0;
}

uint16_t Comm_SendCmd(uint8_t seq, uint8_t cmd,
                      const uint8_t *payload, uint16_t len)
{
    uint16_t frame_len = Protocol_BuildCmd(seq, cmd, payload, len,
                                           tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendAckAck(uint8_t seq, uint8_t received, int16_t ack_seq)
{
    AckAck_t ack = { .received = received, .sequence = ack_seq };
    uint8_t payload[3];
    uint16_t plen = PackAckAck(&ack, payload, sizeof(payload));

    uint16_t frame_len = Protocol_BuildAck(seq, ACK_ACK_ACK, payload, plen,
                                           tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendAckEvent(uint8_t seq, uint8_t finished)
{
    AckEvent_t evt = { .finished = finished };
    uint8_t payload[1];
    uint16_t plen = PackAckEvent(&evt, payload, sizeof(payload));

    uint16_t frame_len = Protocol_BuildAck(seq, ACK_ACK_EVENT, payload, plen,
                                           tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendPosition(uint8_t seq, uint8_t type, int16_t x, int16_t y)
{
    Position_t pos = { .x = x, .y = y };
    uint8_t payload[4];
    uint16_t plen = PackPosition(&pos, payload, sizeof(payload));

    uint16_t frame_len;
    if (type == MSG_TYPE_EVENT) {
        frame_len = Protocol_BuildEvent(seq, payload, plen,
                                        tx_frame_buf, sizeof(tx_frame_buf));
    } else {
        frame_len = Protocol_BuildStatus(seq, payload, plen,
                                         tx_frame_buf, sizeof(tx_frame_buf));
    }
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendHeartbeat(uint8_t seq)
{
    uint16_t frame_len = Protocol_BuildHeartbeat(seq,
                                                  tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_SendError(uint8_t seq, uint8_t error_code)
{
    uint16_t frame_len = Protocol_BuildError(seq, error_code,
                                              tx_frame_buf, sizeof(tx_frame_buf));
    return Comm_SendFrame(tx_frame_buf, frame_len);
}

uint16_t Comm_Depack(uint16_t total_len)
{
    uint16_t offset = 0;

    if (total_len < PROTOCOL_MIN_FRAME_LEN) return 0;

    while (offset + PROTOCOL_MIN_FRAME_LEN <= total_len) {

        /* ---- 1. 找 0xAA ---- */
        if (dma_rx_buf[offset] != PROTOCOL_FRAME_HEADER_0) {
            offset++;
            continue;
        }

        /* ---- 2. 0x55 ---- */
        if (offset + 1 >= total_len) break;
        if (dma_rx_buf[offset + 1] != PROTOCOL_FRAME_HEADER_1) {
            offset++;
            continue;
        }

        /* ---- 3. 固定头: AA 55 | SEQ | TYPE | CMD | LEN = 6 bytes ---- */
        if (offset + 6 > total_len) break;

        uint8_t  seq         = dma_rx_buf[offset + 2];
        uint8_t  type        = dma_rx_buf[offset + 3];
        uint8_t  cmd         = dma_rx_buf[offset + 4];
        uint8_t  payload_len = dma_rx_buf[offset + 5];

        if (payload_len > PROTOCOL_MAX_PAYLOAD) {
            offset++;
            continue;
        }

        uint16_t frame_len = PROTOCOL_MIN_FRAME_LEN + payload_len;

        /* ---- 4. 检查帧是否完整 ---- */
        if (offset + frame_len > total_len) break;

        /* ---- 5. 校验和: SUM(SEQ + TYPE + CMD + LEN + PAYLOAD) 截断 8-bit ---- */
        uint8_t checksum = (uint8_t)(PROTOCOL_FRAME_HEADER_0 + PROTOCOL_FRAME_HEADER_1 + seq + type + cmd + payload_len);
        for (uint16_t i = 0; i < payload_len; i++) {
            checksum += dma_rx_buf[offset + 6 + i];
        }

        uint8_t checksum_received = dma_rx_buf[offset + frame_len - 1];

        /* ---- 6. 校验通过 → 零拷贝指向前一步推进 offset ---- */
        if (checksum == checksum_received) {
            const uint8_t *payload_ptr = &dma_rx_buf[offset + 6];

            frame.seq         = seq;
            frame.type        = type;
            frame.cmd         = cmd;
            frame.len         = payload_len;
            frame.checksum    = checksum;
            frame.payload_ptr = payload_ptr;
            Protocol_UnpackPayload(&frame);

            offset += frame_len;
        } else {
            offset++;   /* 校验失败, 跳过 0xAA 继续搜下一帧 */
        }
    }

    return offset;
}