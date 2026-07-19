#include "bsp_zdt.h"
#include "config.h"
#include "bsp_can.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"
#include "usart.h"

#define ZDT_UART_TX_TIMEOUT_MS       20U
#define ZDT_UART_COMMAND_TIMEOUT_MS  20U
#define ZDT_UART_STOP_RETRIES         3U

static FDCAN_HandleTypeDef   *zdt_hfdcan   = &hfdcan1;
static ZDT_MotorStatus_t      zdt_motors[ZDT_MAX_MOTORS];
static uint8_t                zdt_motor_cnt = 0;

/* ---- helpers ---- */

#if !CONFIG_STEPPER_USE_UART1
static HAL_StatusTypeDef ZDT_SendFrame(uint8_t addr, uint8_t pkt,
                                        uint8_t *data, uint32_t len)
{
    if (zdt_hfdcan == NULL) return HAL_ERROR;
    return CAN_Transmit_EXT(zdt_hfdcan, ZDT_EXT_ID(addr, pkt), data, len);
}
#endif

#if CONFIG_STEPPER_USE_UART1
static HAL_StatusTypeDef ZDT_UART_Transmit(const uint8_t *data, uint16_t len)
{
    return HAL_UART_Transmit(&huart1, (uint8_t *)data, len,
                             ZDT_UART_TX_TIMEOUT_MS);
}

static HAL_StatusTypeDef ZDT_UART_SendEnable(uint8_t addr, bool state)
{
    uint8_t frame[6] = {addr, ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE,
                        (uint8_t)state, ZDT_SYNC_IMMEDIATE, ZDT_CHECKSUM};
    return ZDT_UART_Transmit(frame, sizeof(frame));
}

static HAL_StatusTypeDef ZDT_UART_SendVelocity(
                                      const ZDT_VelocityCommand_t *command,
                                      uint8_t sync_flag)
{
    uint8_t frame[8] = {
        command->addr, ZDT_FC_VELOCITY, command->dir,
        (uint8_t)(command->speed_rpm >> 8),
        (uint8_t)(command->speed_rpm & 0xFFU),
        command->accel, sync_flag, ZDT_CHECKSUM
    };
    return ZDT_UART_Transmit(frame, sizeof(frame));
}

static HAL_StatusTypeDef ZDT_UART_SendPosition(uint8_t addr, uint8_t dir,
                                                uint16_t speed_rpm,
                                                uint8_t accel,
                                                uint32_t pulses,
                                                uint8_t rel_abs,
                                                uint8_t sync_flag)
{
    uint8_t frame[13] = {
        addr, ZDT_FC_POSITION, dir,
        (uint8_t)(speed_rpm >> 8), (uint8_t)(speed_rpm & 0xFFU), accel,
        (uint8_t)(pulses >> 24), (uint8_t)(pulses >> 16),
        (uint8_t)(pulses >> 8), (uint8_t)(pulses & 0xFFU),
        rel_abs, sync_flag, ZDT_CHECKSUM
    };
    return ZDT_UART_Transmit(frame, sizeof(frame));
}

static HAL_StatusTypeDef ZDT_UART_SendStop(uint8_t addr)
{
    uint8_t frame[5] = {addr, ZDT_FC_STOP, ZDT_SUBCODE_STOP,
                        ZDT_SYNC_IMMEDIATE, ZDT_CHECKSUM};
    return ZDT_UART_Transmit(frame, sizeof(frame));
}

static HAL_StatusTypeDef ZDT_UART_SendSync(void)
{
    uint8_t frame[4] = {ZDT_ADDR_BROADCAST, ZDT_FC_SYNC, 0x66U,
                        ZDT_CHECKSUM};
    return ZDT_UART_Transmit(frame, sizeof(frame));
}
#endif

static int ZDT_FindSlot(uint8_t addr)
{
    for (int i = 0; i < ZDT_MAX_MOTORS; i++)
    {
        if (zdt_motors[i].addr == addr) return i;
    }
    if (zdt_motor_cnt < ZDT_MAX_MOTORS)
    {
        int slot = zdt_motor_cnt++;
        memset(&zdt_motors[slot], 0, sizeof(ZDT_MotorStatus_t));
        zdt_motors[slot].addr = addr;
        zdt_motors[slot].move_done = false;
        return slot;
    }
    return -1;
}

/* ---- init ---- */

void ZDT_Init(FDCAN_HandleTypeDef *hfdcan)
{
    zdt_hfdcan = hfdcan;
    zdt_motor_cnt = 0;
    memset(zdt_motors, 0, sizeof(zdt_motors));
}

ZDT_MotorStatus_t* ZDT_GetStatus(uint8_t addr)
{
    int slot = ZDT_FindSlot(addr);
    return (slot >= 0) ? &zdt_motors[slot] : NULL;
}

void ZDT_OnRxMessage(uint32_t ext_id, const uint8_t *data, uint8_t dlc)
{
    if (dlc < 1) return;
    uint8_t addr = (uint8_t)(ext_id >> 8);
    uint8_t func = data[0];

    ZDT_MotorStatus_t *st = ZDT_GetStatus(addr);
    if (st == NULL) return;

    st->last_rx_tick = HAL_GetTick();

    switch (func) {
    case ZDT_FC_READ_POS:
        if (dlc >= 6) {
            st->position = ((int32_t)data[1] << 24) |
                           ((int32_t)data[2] << 16) |
                           ((int32_t)data[3] << 8)  |
                            (int32_t)data[4];
        }
        break;

    case ZDT_FC_READ_STATUS:
        if (dlc >= 2) {
            st->status_flags = data[1];
        }
        break;

    default:
        break;
    }

    /* 到位回传: data = [地址, FD, 9F, 校验]
     * 地址已在 ext_id 中, data[0] 回显地址, data[1]=FD, data[2]=9F */
    if (dlc >= 3 &&
        data[0] == addr &&
        data[1] == ZDT_FC_POSITION &&
        data[2] == 0x9F)
    {
        st->move_done = true;
    }
}

/* ---- 到位等待 / 标志清除 ---- */

bool ZDT_WaitMoveDone(uint8_t addr, uint32_t timeout_ms)
{
    ZDT_MotorStatus_t *st = ZDT_GetStatus(addr);
    if (st == NULL) return false;

    uint32_t start = HAL_GetTick();
    while (!st->move_done)
    {
        if (HAL_GetTick() - start >= timeout_ms)
            return false;           /* 超时 */
        osDelay(1);                 /* 让出 CPU */
    }

    st->move_done = false;          /* 自动清除, 供下一轮使用 */
    return true;
}

void ZDT_ClearMoveDone(uint8_t addr)
{
    ZDT_MotorStatus_t *st = ZDT_GetStatus(addr);
    if (st) st->move_done = false;
}

/* ---- commands ---- */
/* 协议格式: addr仅通过CAN扩展ID传递, data[0]=func, ...params..., 最后字节=checksum(0x6B) */

HAL_StatusTypeDef ZDT_Enable(uint8_t addr)
{
#if CONFIG_STEPPER_USE_UART1
    return ZDT_UART_SendEnable(addr, true);
#else
    uint8_t buf[] = { ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE, 0x01, 0x00 };
    buf[3] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 4);
#endif
}

HAL_StatusTypeDef ZDT_Disable(uint8_t addr)
{
#if CONFIG_STEPPER_USE_UART1
    HAL_StatusTypeDef status = HAL_BUSY;
    for (uint8_t retry = 0U; retry < ZDT_UART_STOP_RETRIES; ++retry) {
        status = ZDT_UART_SendEnable(addr, false);
        if (status == HAL_OK) return HAL_OK;
    }
    return status;
#else
    uint8_t buf[] = { ZDT_FC_ENABLE_CTL, ZDT_SUBCODE_ENABLE, 0x00, 0x00 };
    buf[3] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 4);
#endif
}

HAL_StatusTypeDef ZDT_Stop(uint8_t addr)
{
#if CONFIG_STEPPER_USE_UART1
    return ZDT_StopBatch(&addr, 1U, ZDT_UART_COMMAND_TIMEOUT_MS);
#else
    uint8_t buf[] = { ZDT_FC_STOP, ZDT_SUBCODE_STOP, 0x00 };
    buf[2] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 3);
#endif
}

HAL_StatusTypeDef ZDT_SyncTrigger(void)
{
#if CONFIG_STEPPER_USE_UART1
    return ZDT_UART_SendSync();
#else
    uint8_t buf[] = { ZDT_FC_SYNC, 0x66, 0x00 };
    buf[2] = ZDT_CHECKSUM;
    return ZDT_SendFrame(ZDT_ADDR_BROADCAST, 0, buf, 3);
#endif
}

HAL_StatusTypeDef ZDT_ReadPosition(uint8_t addr)
{
#if CONFIG_STEPPER_USE_UART1
    uint8_t frame[3] = {addr, ZDT_FC_READ_POS, ZDT_CHECKSUM};
    return ZDT_UART_Transmit(frame, sizeof(frame));
#else
    uint8_t buf[] = { ZDT_FC_READ_POS, 0x00 };
    buf[1] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 2);
#endif
}

HAL_StatusTypeDef ZDT_ReadStatus(uint8_t addr)
{
#if CONFIG_STEPPER_USE_UART1
    uint8_t frame[3] = {addr, 0x3AU, ZDT_CHECKSUM};
    return ZDT_UART_Transmit(frame, sizeof(frame));
#else
    uint8_t buf[] = { ZDT_FC_READ_STATUS, 0x00 };
    buf[1] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 2);
#endif
}

HAL_StatusTypeDef ZDT_SetVelocity(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   uint8_t sync_flag)
{
#if CONFIG_STEPPER_USE_UART1
    ZDT_VelocityCommand_t command = {addr, dir, speed_rpm, accel};

    return ZDT_UART_SendVelocity(&command, sync_flag);
#else
    uint8_t buf[7] = {
        ZDT_FC_VELOCITY, dir,
        (uint8_t)(speed_rpm >> 8),
        (uint8_t)(speed_rpm & 0xFF),
        accel, sync_flag,
    };
    buf[6] = ZDT_CHECKSUM;
    return ZDT_SendFrame(addr, 0, buf, 7);
#endif
}

HAL_StatusTypeDef ZDT_SetPosition(uint8_t addr, uint8_t dir,
                                   uint16_t speed_rpm, uint8_t accel,
                                   int32_t pulses, uint8_t rel_abs,
                                   uint8_t sync_flag)
{
#if CONFIG_STEPPER_USE_UART1
    if (pulses < 0) pulses = -pulses;
    return ZDT_UART_SendPosition(addr, dir, speed_rpm, accel,
                                 (uint32_t)pulses, rel_abs, sync_flag);
#else
    uint8_t pkt0[7] = {
        ZDT_FC_POSITION, dir,
        (uint8_t)(speed_rpm >> 8),
        (uint8_t)(speed_rpm & 0xFF),
        accel,
        (uint8_t)(pulses >> 24),
        (uint8_t)(pulses >> 16),
    };

    uint8_t pkt1[6] = {
        ZDT_FC_POSITION,
        (uint8_t)(pulses >> 8),
        (uint8_t)(pulses & 0xFF),
        rel_abs, sync_flag,
    };
    pkt1[5] = ZDT_CHECKSUM;

    if (ZDT_SendFrame(addr, 0, pkt0, 7) != HAL_OK) return HAL_ERROR;
    return ZDT_SendFrame(addr, 1, pkt1, 6);
#endif
}

HAL_StatusTypeDef ZDT_SetVelocitySyncBatch(
                                   const ZDT_VelocityCommand_t *commands,
                                   uint8_t count, uint32_t timeout_ms)
{
    if (commands == NULL || count == 0U || count > ZDT_MAX_MOTORS) {
        return HAL_ERROR;
    }

#if CONFIG_STEPPER_USE_UART1
    HAL_StatusTypeDef status = HAL_OK;

    (void)timeout_ms;
    for (uint8_t i = 0U; i < count; ++i) {
        status = ZDT_UART_SendVelocity(&commands[i], ZDT_SYNC_WAIT);
        if (status != HAL_OK) break;
        osDelay(CONFIG_STEPPER_UART_FRAME_GAP_MS);
    }
    if (status == HAL_OK) {
        status = ZDT_UART_SendSync();
    }
    return status;
#else
    for (uint8_t i = 0U; i < count; ++i) {
        HAL_StatusTypeDef status = ZDT_SetVelocity(commands[i].addr,
                                                   commands[i].dir,
                                                   commands[i].speed_rpm,
                                                   commands[i].accel,
                                                   ZDT_SYNC_WAIT);
        if (status != HAL_OK) return status;
    }
    return ZDT_SyncTrigger();
#endif
}

HAL_StatusTypeDef ZDT_StopBatch(const uint8_t *addresses, uint8_t count,
                                uint32_t timeout_ms)
{
    if (addresses == NULL || count == 0U || count > ZDT_MAX_MOTORS) {
        return HAL_ERROR;
    }

#if CONFIG_STEPPER_USE_UART1
    HAL_StatusTypeDef result = HAL_BUSY;

    (void)timeout_ms;
    for (uint8_t retry = 0U; retry < ZDT_UART_STOP_RETRIES; ++retry) {
        result = HAL_OK;
        for (uint8_t i = 0U; i < count; ++i) {
            HAL_StatusTypeDef status = ZDT_UART_SendStop(addresses[i]);
            if (status != HAL_OK && result == HAL_OK) result = status;
            osDelay(CONFIG_STEPPER_UART_FRAME_GAP_MS);
        }
        if (result == HAL_OK) return HAL_OK;
    }
    return result;
#else
    HAL_StatusTypeDef result = HAL_OK;
    (void)timeout_ms;
    for (uint8_t i = 0U; i < count; ++i) {
        HAL_StatusTypeDef status = ZDT_Stop(addresses[i]);
        if (status != HAL_OK && result == HAL_OK) result = status;
    }
    return result;
#endif
}
