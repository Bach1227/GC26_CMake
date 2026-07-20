#include "comm_manager.h"

#include "cmsis_os2.h"
#include "config.h"
#include "dma.h"
#include "FreeRTOS.h"
#include "protocol.h"
#include "stream_buffer.h"
#include "task.h"
#include "usart.h"

#define DMA_RX_BUF_SIZE          256u
#define COMM_STREAM_BUFFER_SIZE  512u
#define COMM_TASK_READ_SIZE       64u
#define COMM_TX_TIMEOUT_MS         20u

__attribute__((section(".RAM_D1"))) uint8_t dma_rx_buf[DMA_RX_BUF_SIZE];

volatile uint32_t comm_uart_rx_event_count = 0u;
volatile uint32_t comm_uart_rx_byte_count = 0u;
volatile uint32_t comm_uart_test_tx_count = 0u;
volatile uint32_t comm_uart_test_tx_error_count = 0u;

static StreamBufferHandle_t uart_rx_stream = NULL;
static volatile uint8_t uart_rx_error_flag = 0u;
static volatile uint32_t uart_rx_dropped_bytes = 0u;
static volatile HAL_StatusTypeDef uart_rx_start_status = HAL_OK;

TaskHandle_t Comm_TaskHandle = NULL;

static void transmit_protocol_frame(const uint8_t *data, uint16_t length)
{
    if (data == NULL || length == 0u) {
        return;
    }

    /*
     * Protocol's echo buffer is only valid during this callback, so use the
     * synchronous HAL API and finish copying/transmitting before returning.
     */
    (void)HAL_UART_Transmit(&COMM_UART_HANDLE, (uint8_t *)data, length,
                            COMM_TX_TIMEOUT_MS);
}

static HAL_StatusTypeDef start_uart_rx(void)
{
    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(&COMM_UART_HANDLE,
                                     dma_rx_buf, DMA_RX_BUF_SIZE);

    if (status == HAL_OK && COMM_UART_HANDLE.hdmarx != NULL) {
        __HAL_DMA_DISABLE_IT(COMM_UART_HANDLE.hdmarx, DMA_IT_HT);
    }
    uart_rx_start_status = status;
    return status;
}

void Comm_Init(void)
{
    __HAL_UART_CLEAR_IDLEFLAG(&COMM_UART_HANDLE);
    __HAL_UART_CLEAR_FLAG(&COMM_UART_HANDLE,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);
    Protocol_SetTransmitCallback(transmit_protocol_frame);
    Protocol_Reset();
}

static void restart_uart_rx(void)
{
    (void)HAL_UART_AbortReceive(&COMM_UART_HANDLE);
    __HAL_UART_CLEAR_FLAG(&COMM_UART_HANDLE,
                          UART_CLEAR_OREF | UART_CLEAR_NEF |
                          UART_CLEAR_PEF | UART_CLEAR_FEF);

    if (uart_rx_stream != NULL) {
        (void)xStreamBufferReset(uart_rx_stream);
    }
    Protocol_Reset();
    (void)start_uart_rx();
}

void Comm_OnUartError(void)
{
    uart_rx_error_flag = 1u;
}

void Comm_OnUartRx(uint16_t received_size)
{
    BaseType_t task_woken = pdFALSE;

    ++comm_uart_rx_event_count;

    if (received_size > DMA_RX_BUF_SIZE) {
        received_size = DMA_RX_BUF_SIZE;
    }
    comm_uart_rx_byte_count += received_size;

    if (uart_rx_stream != NULL && received_size > 0u) {
        size_t sent = xStreamBufferSendFromISR(uart_rx_stream,
                                               dma_rx_buf,
                                               received_size,
                                               &task_woken);
        if (sent < received_size) {
            uart_rx_dropped_bytes += (uint32_t)(received_size - sent);
        }
    } else {
        uart_rx_dropped_bytes += received_size;
    }

    (void)start_uart_rx();
    portYIELD_FROM_ISR(task_woken);
}

static void Comm_Task(void *argument)
{
    uint8_t bytes[COMM_TASK_READ_SIZE];
#if CONFIG_COMM_UART_LOOPBACK_TEST
    static const uint8_t loopback_test_frame[] = {
        PROTOCOL_HEADER_0,
        PROTOCOL_HEADER_1,
        0x7Fu, /* Reserved test command: parsed but deliberately not echoed. */
        0x00u,
        0x7Fu,
    };
    TickType_t last_test_tx = xTaskGetTickCount();
#endif

    (void)argument;
    (void)start_uart_rx();

    for (;;) {
        if (uart_rx_error_flag != 0u) {
            uart_rx_error_flag = 0u;
            restart_uart_rx();
        }

        size_t received = xStreamBufferReceive(
            uart_rx_stream, bytes, sizeof(bytes), pdMS_TO_TICKS(20u));
        if (received > 0u) {
            Protocol_ProcessBytes(bytes, (uint16_t)received);
        }

#if CONFIG_COMM_UART_LOOPBACK_TEST
        TickType_t now = xTaskGetTickCount();
        if ((TickType_t)(now - last_test_tx) >=
            pdMS_TO_TICKS(CONFIG_COMM_UART_LOOPBACK_TEST_PERIOD_MS)) {
            HAL_StatusTypeDef status = HAL_UART_Transmit(
                &COMM_UART_HANDLE,
                (uint8_t *)loopback_test_frame,
                sizeof(loopback_test_frame),
                COMM_TX_TIMEOUT_MS);
            if (status == HAL_OK) {
                ++comm_uart_test_tx_count;
            } else {
                ++comm_uart_test_tx_error_count;
            }
            last_test_tx = now;
        }
#endif
    }
}

void Comm_InitTask(void)
{
    uart_rx_stream = xStreamBufferCreate(COMM_STREAM_BUFFER_SIZE, 1u);
    if (uart_rx_stream == NULL) {
        uart_rx_start_status = HAL_ERROR;
        return;
    }

    if (xTaskCreate(Comm_Task, "comm_task", 256, NULL,
                    osPriorityAboveNormal1, &Comm_TaskHandle) != pdPASS) {
        vStreamBufferDelete(uart_rx_stream);
        uart_rx_stream = NULL;
        uart_rx_start_status = HAL_ERROR;
        return;
    }

}
