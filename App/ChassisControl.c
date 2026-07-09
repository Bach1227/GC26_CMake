#include "ChassisControl.h"
#include "bsp_zdt.h"
#include "bsp_can.h"
#include "gimbal.h"
#include "MoveControl.h"
#include "cmsis_os2.h"
#include <math.h>
#include <string.h>
#include "PID.h"
#include "queue.h"
#include "timers.h"
#include <stdlib.h>

/* ====================================================================== */
/*  常量                                                                  */
/* ====================================================================== */

#define WHEEL_RADIUS     0.075f
#define PULSES_PER_REV   2000
#define POS_SPEED_RPM    200
#define POS_ACCEL        128

/* 脉冲 / 米: PPR / (2πR) */
#define PULSE_PER_M      ((float)PULSES_PER_REV / (2.0f * 3.14159265f * WHEEL_RADIUS))


/* ====================================================================== */
/*  移动指令队列                                                          */
/* ====================================================================== */

#define CHASSIS_QUEUE_LEN   8

static QueueHandle_t chassis_queue = NULL;

/* ====================================================================== */
/*  工具: 轮位移(m) → ZDT_SetPosition                                    */
/* ====================================================================== */

static void wheel_position(uint8_t id, float disp_m)
{
    int32_t pulses = (int32_t)(disp_m * PULSE_PER_M);
    if (pulses == 0) return;

    uint8_t dir  = (pulses > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint16_t rpm = (uint16_t)((pulses > 0) ? pulses : -pulses);
    if (rpm > POS_SPEED_RPM) rpm = POS_SPEED_RPM;

    if (pulses < 0) pulses = -pulses;

    ZDT_SetPosition(id, dir, rpm, POS_ACCEL, pulses,
                    ZDT_POS_RELATIVE, ZDT_SYNC_WAIT);
}

/* ====================================================================== */
/*  Chassis_OnCarMove                                                      */
/*  上位机方向+距离 → 车体位移 → 逆运动学 → 四轮脉冲 → ZDT_SetPosition    */
/* ====================================================================== */

void Chassis_OnCarMove(const CarMove_t *cmd)
{
    if (cmd == NULL) return;

    /* 方向角(rad) → 车体位移 */
    float dx = cosf(cmd->direction) * cmd->distance;
    float dy = sinf(cmd->direction) * cmd->distance;
    float dt = cmd->direction;          /* 旋转角 = 朝向 */

    /* 逆运动学: 车体位移 → 四轮线位移 (位移量直接代入速度结构体, 线性变换等价) */
    ChassisSpeed_t ik_in  = {dx, dy, dt};
    WheelSpeed_t   ik_out;
    Kinematics_Inverse(&ik_in, &ik_out);

    /* 四轮定位 */
    wheel_position(1, ik_out.v1);
    osDelay(1);
    wheel_position(2, ik_out.v2);
    osDelay(1);
    wheel_position(3, ik_out.v3);
    osDelay(1);
    wheel_position(4, ik_out.v4);
    osDelay(1);

    ZDT_SyncTrigger();
    osDelay(1);
}

/* ====================================================================== */
/*  全局变量                                                              */
/* ====================================================================== */

TaskHandle_t ChassisTaskHandle;

/* ====================================================================== */
/*  Chassis_SendMoveCmd — 下发移动指令 (非阻塞)                            */
/* ====================================================================== */

static volatile uint32_t dbg_cmd_sent = 0;   /* 调试: Chassis_SendMoveCmd 累计入队 */
static volatile uint32_t dbg_cmd_recv = 0;   /* 调试: Chassis_Task 累计收到 */

int Chassis_SendMoveCmd(int16_t x, int16_t y, Event_t completion_event)
{
    if (chassis_queue == NULL) return -1;
    ChassisMoveCmd_t cmd = { x, y, 0, completion_event };
    if (xQueueSend(chassis_queue, &cmd, 0) == pdPASS) {
        dbg_cmd_sent++;
        return 0;
    }
    return -1;
}

int Chassis_SendRotateCmd(float degrees, Event_t completion_event)
{
    if (chassis_queue == NULL) return -1;
    /* degree → 百分之一弧度 (内部存储用, 陀螺仪反馈用 rad) */
    int16_t centirad = (int16_t)(degrees * 3.14159265f / 180.0f * 100.0f);
    ChassisMoveCmd_t cmd = { 0, 0, centirad, completion_event };
    if (xQueueSend(chassis_queue, &cmd, 0) == pdPASS) {
        dbg_cmd_sent++;
        return 0;
    }
    return -1;
}

/* ====================================================================== */
/*  Chassis_NotifyMoveComplete — 通知底盘当前移动已完成 (ISR 安全)         */
/* ====================================================================== */

void Chassis_NotifyMoveComplete(void)
{
    if (ChassisTaskHandle == NULL) return;

    BaseType_t taskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(ChassisTaskHandle, &taskWoken);
    portYIELD_FROM_ISR(taskWoken);
}

/* ====================================================================== */
/*  陀螺仪闭环旋转                                                        */
/* ====================================================================== */

/* --- 陀螺仪接口 (TODO: 替换为真实 IMU 驱动) --- */
static float gyro_get_angle(void)
{
    return Gimbal_GetAngle();   /* 暂用云台电机角度 */
}

/* --- 旋转 PID (使用 PID.h 组件) --- */
static PID_Param_float rotate_pid;
static bool            rotate_active = false;

#define ROTATE_PID_Kp       0.05f        /* 角度° → 速度, 已按°缩放 */
#define ROTATE_PID_Ki       0.0f
#define ROTATE_PID_Kd       0.002f       /* 微分阻尼 */
#define ROTATE_SPEED_LIMIT  2.0f         /* 最大旋转速度 rad/s */
#define ROTATE_ANGLE_DEAD   3.0f         /* °, 到位死区 */

/* --- 驱动四轮旋转 (差分) --- */
static void drive_rotation(float speed_rad_s)
{
    /* 旋转: 前+右 正转, 左+后 反转 (麦克纳姆轮旋转模式) */
    /* speed_rad_s → ZDT 转速, 按经验比例换算 */
    int16_t rpm = (int16_t)(speed_rad_s * 30.0f);
    if (rpm > 150) rpm = 150;
    if (rpm < -150) rpm = -150;

    ZDT_SetVelocity(1, rpm > 0 ? ZDT_DIR_CW : ZDT_DIR_CCW,
                    abs(rpm), 20, ZDT_SYNC_IMMEDIATE);
    ZDT_SetVelocity(2, rpm > 0 ? ZDT_DIR_CCW : ZDT_DIR_CW,
                    abs(rpm), 20, ZDT_SYNC_IMMEDIATE);
    ZDT_SetVelocity(3, rpm > 0 ? ZDT_DIR_CCW : ZDT_DIR_CW,
                    abs(rpm), 20, ZDT_SYNC_IMMEDIATE);
    ZDT_SetVelocity(4, rpm > 0 ? ZDT_DIR_CW : ZDT_DIR_CCW,
                    abs(rpm), 20, ZDT_SYNC_IMMEDIATE);
    ZDT_SyncTrigger();
}

/* ====================================================================== */
/*  软件定时器: 轮询 ZDT 到位 / 陀螺仪旋转到位                            */
/* ====================================================================== */

static TimerHandle_t  move_check_timer = NULL;
static uint32_t      move_start_tick = 0;

#define MOVE_TIMEOUT_MS  2000   /* 2s 超时 */

static void MoveCheckTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    /* ---- 旋转模式 (陀螺仪闭环) ---- */
    if (rotate_active)
    {
        float current = gyro_get_angle();
        float speed = PID_Update_float(&rotate_pid, current);

        /* 死区: 误差在死区内视为到位 */
        if (fabsf(rotate_pid.ErrorNow) < ROTATE_ANGLE_DEAD)
        {
            ZDT_Stop(1); ZDT_Stop(2); ZDT_Stop(3); ZDT_Stop(4);
            rotate_active = false;
            PID_ClearUp_float(&rotate_pid);
            xTimerStop(xTimer, 0);
            xTaskNotifyGive(ChassisTaskHandle);
            return;
        }

        if (speed >  ROTATE_SPEED_LIMIT) speed =  ROTATE_SPEED_LIMIT;
        if (speed < -ROTATE_SPEED_LIMIT) speed = -ROTATE_SPEED_LIMIT;

        drive_rotation(speed);
        return;
    }

    /* ---- 平移模式 (ZDT 到位回传) ---- */
    /* 超时检查 */
    if (HAL_GetTick() - move_start_tick >= MOVE_TIMEOUT_MS)
    {
        xTimerStop(xTimer, 0);
        xTaskNotifyGive(ChassisTaskHandle);
        return;
    }

    for (uint8_t i = 1; i <= 4; i++)
    {
        ZDT_MotorStatus_t *st = ZDT_GetStatus(i);
        if (st == NULL || !st->move_done)
            return;                     /* 还有电机没到位 */
    }

    /* 四轮全部到位 → 清标志, 停止定时器, 通知任务 */
    for (uint8_t i = 1; i <= 4; i++)
    {
        ZDT_MotorStatus_t *st = ZDT_GetStatus(i);
        if (st) st->move_done = false;
    }

    xTimerStop(xTimer, 0);
    xTaskNotifyGive(ChassisTaskHandle);
}

/* ====================================================================== */
/*  Chassis_Task                                                          */
/*  阻塞读取移动指令队列 → 执行 → 等待电机到位 → 取下一条                 */
/* ====================================================================== */

void Chassis_Task(void *argument)
{
    (void)argument;

    /* 启动 FDCAN1 发送 (不过滤接收) */
    HAL_FDCAN_Start(&hfdcan1);

    ZDT_Enable(1);
    ZDT_Enable(2);
    ZDT_Enable(3);
    ZDT_Enable(4);

    ChassisTaskHandle = xTaskGetCurrentTaskHandle();

    /* 创建到位检测定时器 (10ms 周期) */
    move_check_timer = xTimerCreate("move_chk", pdMS_TO_TICKS(10), pdTRUE,
                                     NULL, MoveCheckTimerCallback);

    ChassisMoveCmd_t cmd;

    for (;;)
    {
        /* 1) 阻塞等待移动指令 */
        if (xQueueReceive(chassis_queue, &cmd, portMAX_DELAY) != pdPASS)
            continue;
        dbg_cmd_recv++;

        /* 判断指令类型 */
        if (cmd.x == 0 && cmd.y == 0 && cmd.rotation != 0)
        {
            /* === 纯旋转: 陀螺仪闭环 (PID float) === */
            PID_Set_Kparam_float(&rotate_pid, ROTATE_PID_Kp,
                                  ROTATE_PID_Ki, ROTATE_PID_Kd);
            PID_Set_Target_float(&rotate_pid, (float)cmd.rotation / 100.0f);
            PID_Set_OutputLimit_float(&rotate_pid, ROTATE_SPEED_LIMIT);
            PID_ClearUp_float(&rotate_pid);
            rotate_active = true;

            move_start_tick = HAL_GetTick();
            xTimerStart(move_check_timer, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

            rotate_active = false;
        }
        else
        {
            /* === 平移 (含平移+旋转) === */
            float dx = (float)cmd.x / 1000.0f;
            float dy = (float)cmd.y / 1000.0f;
            float dt = (float)cmd.rotation / 100.0f;

            ChassisSpeed_t ik_in  = {dx, dy, dt};
            WheelSpeed_t   ik_out;
            Kinematics_Inverse(&ik_in, &ik_out);

            wheel_position(1, ik_out.v1); osDelay(1);
            wheel_position(2, ik_out.v2); osDelay(1);
            wheel_position(3, ik_out.v3); osDelay(1);
            wheel_position(4, ik_out.v4); osDelay(1);

            ZDT_SyncTrigger();

            move_start_tick = HAL_GetTick();
            xTimerStart(move_check_timer, 0);
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }

        /* 到位 → 通知状态机 (如果有事件) */
        if (cmd.completion_event != EVENT_NONE) {
            SM_SendEvent(cmd.completion_event);
        }
        /* 取下一条指令 */
    }
}

void Chassis_TaskInit(void)
{
    /* 队列必须在调度器启动前创建 */
    chassis_queue = xQueueCreate(CHASSIS_QUEUE_LEN, sizeof(ChassisMoveCmd_t));
    xTaskCreate(Chassis_Task, "chassitask", 256, NULL, osPriorityAboveNormal1, &ChassisTaskHandle);
}

/* ====================================================================== */
/* ====================================================================== */
/*  Chassis_Stop / Chassis_Enable                                         */
/* ====================================================================== */

void Chassis_Stop(void)
{
    ZDT_Stop(1); ZDT_Stop(2); ZDT_Stop(3); ZDT_Stop(4);
}

void Chassis_Enable(bool en)
{
    if (en) {
        ZDT_Enable(1); ZDT_Enable(2); ZDT_Enable(3); ZDT_Enable(4);
    } else {
        ZDT_Disable(1); ZDT_Disable(2); ZDT_Disable(3); ZDT_Disable(4);
    }
}
