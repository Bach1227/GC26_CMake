#ifndef __GIMBAL_H__
#define __GIMBAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32h7xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#include "bsp_can.h"
#include "statemachine.h"   /* Event_t */

/* ====================================================================== */
/*  API                                                                    */
/* ====================================================================== */

/** 初始化云台电机, 由 Gimbal_Task 内部调用 */
void Gimbal_Init(void);

/** CAN 中断回调 — 收到 DM 反馈帧时调用, 解析角度/速度/扭矩 */
void Gimbal_OnCanRx(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxFrame_t *frame);

/** 设置目标角度 (°) */
void Gimbal_SetAngle(float angle_deg);

/** 获取当前角度 (°), 用作陀螺仪桩 */
float Gimbal_GetAngle(void);

/** 云台角度闭环任务 (1kHz PID) */
void Gimbal_Task(void *argument);

/** 创建云台任务线程 (PID + Cmd) */
void Gimbal_InitTask(void);

/* ====================================================================== */
/*  机械臂命令队列 (解耦状态机)                                           */
/* ====================================================================== */

/** 复合命令类型 */
typedef enum {
    GIMBAL_CMD_FETCH_RAW,     /* 完整取料序列 (3次颜色等待+夹取+放车身) */
    GIMBAL_CMD_PLACE_ROUGH,   /* 完整粗加工放置序列 (取→放→取回) */
    GIMBAL_CMD_PLACE_TEMP,    /* 完整暂存区放置序列 (取→放, 不回取) */
    GIMBAL_CMD_STACK_TEMP,    /* 完整暂存区码垛序列 */
} GimbalCmdType_t;

/** 命令结构体 */
typedef struct {
    GimbalCmdType_t type;
    Event_t         completion_event; /* 完成后发的 SM 事件 */
} GimbalCmd_t;

/** 初始化命令任务 (创建队列 + xTaskCreate) */
void Gimbal_CmdInit(void);

/** 非阻塞入队, 返回 0 成功 / -1 失败 */
int  Gimbal_SendCmd(const GimbalCmd_t *cmd);

/** 命令任务入口 */
void Gimbal_CmdTask(void *argument);

/* ====================================================================== */
/*  ZDT 直线动作 (伸长 / 升降)                                             */
/* ====================================================================== */

/** 伸长/收缩 (pulses > 0 = 伸出, < 0 = 缩回) */
void Gimbal_Extend(int32_t pulses);

/** 升降（pulses > 0 上升，< 0 下降）；车体/底面行程由调用方选择配置宏。 */
void Gimbal_Lift(int32_t pulses);

/** 初始化夹爪 PWM (启动 TIM1 通道 3 输出) */
void Gimbal_GripperInit(void);

/** 夹爪 PWM（TIM1 为 1 us/计数，pulse 等于高电平脉宽 us） */
void Gimbal_Gripper(uint32_t pulse);

#ifdef __cplusplus
}
#endif

#endif /* __GIMBAL_H__ */
