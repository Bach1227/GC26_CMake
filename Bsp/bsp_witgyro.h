#ifndef __BSP_WITGYRO_H__
#define __BSP_WITGYRO_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include "DataStructure.h"
#include "usart.h"

/* ======================== WIT协议常量 ======================== */

/* 陀螺仪串口宏 — 修改这里切换 */
#define GYRO_UART_HANDLE    huart10
#define GYRO_UART_INSTANCE  USART10
#define WITGYRO_PACKET_SIZE   11    /* 一帧固定11字节 */
#define WITGYRO_HEADER        0x55  /* 数据帧包头 */
#define WITGYRO_TYPE_ANGLE    0x53  /* 角度帧类型 */
#define WITGYRO_CMD_HEADER1   0xFF  /* 命令帧包头1 */
#define WITGYRO_CMD_HEADER2   0xAA  /* 命令帧包头2 */

/* ======================== 设备句柄 ======================== */
typedef struct {
    float pitch;
    float roll;
    float yaw;                  /* 相对软件零点的 Z 轴角度 */
    float yaw_raw;              /* 传感器原始 Z 轴角度 */
    float yaw_zero;             /* 软件零点 */

    uint32_t frame_count;
    uint32_t zero_generation;
    uint8_t  data_ready;
    uint8_t  yaw_zero_valid;
} WitGyro_HandleTypeDef;

/* ======================== 函数声明 ======================== */

void WitGyro_Init(void);
uint8_t WitGyro_Parse(const uint8_t *buf, uint16_t length);
uint8_t WitGyro_GetAngle(float *pitch, float *roll, float *yaw);
uint32_t WitGyro_GetZeroGeneration(void);
void WitGyro_ZeroYaw(void);
void WitGyro_ZeroAll(void);
void WitGyro_ResetRef(void);

#ifdef __cplusplus
}
#endif

#endif /* __BSP_WITGYRO_H__ */
