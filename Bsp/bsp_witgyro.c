#include "bsp_witgyro.h"
#include "cmsis_os2.h"

WitGyro_HandleTypeDef g_wit;

/* ======================== 内部辅助 ======================== */

/* 将2字节拼接为有符号16位(小端: 低字节在前) */
static int16_t bytes_to_int16(uint8_t lo, uint8_t hi)
{
    return (int16_t)(((uint16_t)hi << 8) | lo);
}

/* 计算校验和: 除包头0x55外所有字节的累加和取低8位 */
static uint8_t calc_checksum(const uint8_t *buf)
{
    uint16_t sum = 0;
    for (uint8_t i = 1; i < WITGYRO_PACKET_SIZE - 1; i++) {
        sum += buf[i];
    }
    return (uint8_t)(sum & 0xFF);
}

/* 发送原始字节数组 */
static void send_bytes(const uint8_t *data, uint8_t len)
{
    HAL_UART_Transmit(&GYRO_UART_HANDLE, data, len, 10);
}

/* ======================== 初始化 ======================== */

/**
 * @brief  初始化陀螺仪句柄
 * @note   只绑定串口并清零状态，不启动接收(接收方式由调用者决定)
 */
void WitGyro_Init(void)
{
    g_wit.pitch        = 0.0f;
    g_wit.roll         = 0.0f;
    g_wit.yaw          = 0.0f;
    g_wit.yaw_raw      = 0.0f;
    g_wit.yaw_zero     = 0.0f;
    g_wit.frame_count  = 0;
    g_wit.zero_generation = 0;
    g_wit.data_ready   = 0;
    g_wit.yaw_zero_valid = 0;
}

/* ======================== 协议解析(核心) ======================== */

/**
 * @brief  解析缓冲区中的完整帧（扫描 0x55 帧头）
 * @note   while 循环搜索并解析所有完整帧，尾部残缺字节由调用者丢弃。
 *         对 100Hz 流式推送而言，丢残缺字节可接受。
 * @retval 1=至少解析到一帧, 0=无完整帧
 */
uint8_t WitGyro_Parse(const uint8_t *buf, uint16_t length)
{
    uint8_t found = 0;
    uint16_t i = 0;

    while (i + WITGYRO_PACKET_SIZE <= length) {
        if (buf[i] != WITGYRO_HEADER) {
            i++;
            continue;
        }

        /* 角度帧 0x53 */
        if (buf[i + 1] != WITGYRO_TYPE_ANGLE) {
            i += WITGYRO_PACKET_SIZE;
            continue;
        }
        /* 校验和: 帧头0x55 + 后续9字节 = 10字节累加取低8位 */
        uint16_t sum = 0;
        for (uint8_t j = 0; j < WITGYRO_PACKET_SIZE - 1; j++) {
            sum += buf[i + j];
        }
        if ((uint8_t)(sum & 0xFF) != buf[i + WITGYRO_PACKET_SIZE - 1]) {
            i++;
            continue;
        }

        int16_t raw_pitch  = (int16_t)(((uint16_t)buf[i + 3] << 8) | buf[i + 2]);
        int16_t raw_roll = (int16_t)(((uint16_t)buf[i + 5] << 8) | buf[i + 4]);
        int16_t raw_yaw   = (int16_t)(((uint16_t)buf[i + 7] << 8) | buf[i + 6]);

        g_wit.roll    = (float)raw_roll  * 180.0f / 32768.0f;
        g_wit.pitch   = (float)raw_pitch * 180.0f / 32768.0f;
        g_wit.yaw_raw = (float)raw_yaw   * 180.0f / 32768.0f;

        /* 第一帧通过校验的 Z 轴角度作为软件零点 */
        if (g_wit.yaw_zero_valid == 0) {
            g_wit.yaw_zero = g_wit.yaw_raw;
            g_wit.yaw_zero_valid = 1;
            g_wit.zero_generation++;
        }

        g_wit.yaw = g_wit.yaw_raw - g_wit.yaw_zero;
        if (g_wit.yaw >= 180.0f) {
            g_wit.yaw -= 360.0f;
        } else if (g_wit.yaw < -180.0f) {
            g_wit.yaw += 360.0f;
        }
        g_wit.frame_count++;
        g_wit.data_ready = 1;
        found = 1;

        i += WITGYRO_PACKET_SIZE;
    }

    return found;
}

/* ======================== 读取角度 ======================== */

/**
 * @brief  获取最新的角度值
 * @retval 1=有新数据, 0=数据未被更新过
 */
uint8_t WitGyro_GetAngle(float *pitch, float *roll, float *yaw)
{
    if (g_wit.data_ready == 0) return 0;

    *pitch = g_wit.pitch;
    *roll  = g_wit.roll;
    *yaw   = g_wit.yaw;

    g_wit.data_ready = 0;
    return 1;
}

uint32_t WitGyro_GetZeroGeneration(void)
{
    return g_wit.zero_generation;
}

/* ======================== 角度参考命令 ======================== */

/**
 * @brief  角度参考(XY轴相对归零)
 * @note   以当前姿态为参考，Roll/Pitch 归零，Yaw 不变。
 *         用 osDelay 代替 HAL_Delay，空闲时让出 CPU。
 * @warning 包含延时，只能在 FreeRTOS 任务中调用。
 */
void WitGyro_ResetRef(void)
{
    uint8_t unlock[]  = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t ang_ref[] = {0xFF, 0xAA, 0x01, 0x08, 0xB2};
    uint8_t save[]    = {0xFF, 0xAA, 0x00, 0x00, 0xA9};

    send_bytes(unlock, 5);
    osDelay(200);
    send_bytes(ang_ref, 5);
    osDelay(3000);
    send_bytes(save, 5);
}

/* ======================== 软件置零 ======================== */

/**
 * @brief  将当前 Z 轴角度记录为软件零点
 * @note   不向传感器写寄存器；若尚未收到有效帧，则下一帧自动成为零点。
 */
void WitGyro_ZeroYaw(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    if (g_wit.frame_count > 0) {
        g_wit.yaw_zero = g_wit.yaw_raw;
        g_wit.yaw_zero_valid = 1;
        g_wit.zero_generation++;
        g_wit.yaw = 0.0f;
        g_wit.data_ready = 1;
    } else {
        /* 还没有有效帧：下一帧自动成为零点 */
        g_wit.yaw_zero_valid = 0;
    }

    if (primask == 0U) {
        __enable_irq();
    }
}

/**
 * @brief  全部角度归零(加速度校准)
 * @note   发送解锁→加速度校准→停止→保存 四步流程。
 *         模块需保持水平静置，完成后Roll/Pitch/Yaw全部归零。
 * @warning 此函数包含忙等延时，不能在中断中调用。
 */
void WitGyro_ZeroAll(void)
{
    /* 步骤1: 解锁寄存器写保护 */
    uint8_t unlock[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    send_bytes(unlock, 5);
    HAL_Delay(100);

    /* 步骤2: 进入加速度校准 */
    uint8_t cal_start[] = {0xFF, 0xAA, 0x01, 0x01, 0xAB};
    send_bytes(cal_start, 5);
    HAL_Delay(3000);  /* 等待3秒完成校准 */

    /* 步骤3: 退出校准 */
    uint8_t cal_stop[] = {0xFF, 0xAA, 0x01, 0x00, 0xAA};
    send_bytes(cal_stop, 5);
    HAL_Delay(100);

    /* 步骤4: 保存到Flash */
    uint8_t save[] = {0xFF, 0xAA, 0x00, 0x00, 0xA9};
    send_bytes(save, 5);
}
