#ifndef __COMM_MANAGER_H__
#define __COMM_MANAGER_H__

#include <stdint.h>

/* Communication UART used by the upper-computer protocol. */
#define COMM_UART_HANDLE    huart7
#define COMM_UART_INSTANCE  UART7

#ifdef __cplusplus
extern "C" {
#endif

extern uint8_t dma_rx_buf[256];
extern volatile uint32_t comm_uart_rx_event_count;
extern volatile uint32_t comm_uart_rx_byte_count;
extern volatile uint32_t comm_uart_test_tx_count;
extern volatile uint32_t comm_uart_test_tx_error_count;

/** Clear UART flags before the RTOS objects are created. */
void Comm_Init(void);

/** Create the byte stream/task; the task starts Receive-to-IDLE DMA. */
void Comm_InitTask(void);

/** UART Receive-to-IDLE callback entry; must be called from ISR context. */
void Comm_OnUartRx(uint16_t received_size);

/** UART error callback entry; must be called from ISR context. */
void Comm_OnUartError(void);

#ifdef __cplusplus
}
#endif

#endif /* __COMM_MANAGER_H__ */
