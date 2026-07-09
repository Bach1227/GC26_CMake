#include "gimbal.h"
#include "bsp_dm.h"
#include "bsp_zdt.h"
#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include "PID.h"
#include "protocol.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "tim.h"

/* 共享变量 (statemachine.c 中定义) */
extern volatile bool g_color_pending;
extern volatile ColorConfirm_t g_color_result;
extern uint8_t seq[3];
extern const uint8_t expected_color[4];

/* ====================================================================== */
/*  配置                                                                  */
/* ====================================================================== */

#define MOTOR_TX_ID         0x001u     /* 发送命令 ID */
#define MOTOR_RX_ID         0x000u     /* 接收反馈 ID */
#define ANGLE_TARGET_DEFAULT 0.0f       /* 默认目标 180° */

/* PID (角度° → 速度) */
#define PID_Kp              0.02f
#define PID_Ki              0.001f
#define PID_Kd              0.0f
#define SPEED_LIMIT         6.0f          /* 输出上限 */
#define INTEGRAL_RANGE      30.0f         /* ° */
#define INTEGRAL_LIMIT      180.0f        /* ° */
#define DEAD_ZONE           1.0f          /* ° 到位死区 */
#define CTRL_PERIOD_MS      1

/* MIT 速度模式 */
#define MIT_KD              1.2f

/* ====================================================================== */
/*  ZDT 直线动作配置                                                      */
/* ====================================================================== */

#define ZDT_ID_EXTEND       5u          /* 伸长电机 CAN 地址 */
#define ZDT_ID_LIFT         6u          /* 升降电机 CAN 地址 */
#define ZDT_SPEED_RPM       200         /* ZDT 运行速度 */
#define ZDT_ACCEL           128         /* ZDT 加速度 */

/* ====================================================================== */
/*  内部变量                                                              */
/* ====================================================================== */

DM_MotorControlBlock   g_motor  = {0};
static bool                   g_inited = false;

PID_Param_float g_pid;
float           g_target_angle = ANGLE_TARGET_DEFAULT;
bool            g_pid_inited   = false;

/* ====================================================================== */
/*  Gimbal_Init                                                           */
/* ====================================================================== */

void Gimbal_Init(void)
{

    memset(&g_motor, 0, sizeof(g_motor));

    DM_Motor_MaxConfigInit(&g_motor.MaxConfig,
                           3.14f,    /* Pmax */
                           30.0f,    /* Vmax */
                           10.0f,    /* Tmax */
                           500.0f,   /* Kpmax */
                           5.0f);    /* Kdmax */

    g_inited = true;
}

/* ====================================================================== */
/*  Gimbal_OnCanRx — CAN 回调                                            */
/* ====================================================================== */

void Gimbal_OnCanRx(FDCAN_HandleTypeDef *hfdcan, FDCAN_RxFrame_t *frame)
{
    if (!g_inited || frame == NULL) return;

    if ((frame->ID & 0xFFFF) == MOTOR_RX_ID)
    {
        DM_J4310_MIT_Parse(&g_motor, frame->data);
    }
}

/* ====================================================================== */
/*  Gimbal_SetAngle / Gimbal_GetAngle                                     */
/* ====================================================================== */

void Gimbal_SetAngle(float angle_deg)
{
    g_target_angle = angle_deg;
    if (g_pid_inited)
        PID_Set_Target_float(&g_pid, g_target_angle);
}

float Gimbal_GetAngle(void)
{
    return g_motor.Status.position_angle;
}

/* ====================================================================== */
/*  Gimbal_Task — 1kHz 角度闭环 PID                                       */
/* ====================================================================== */

void Gimbal_Task(void *argument)
{
    (void)argument;

    /* 启动 CAN2 + 配置过滤器 (只收 0x000) */
    FDCAN_FilterTypeDef filter = {
        .IdType       = FDCAN_STANDARD_ID,
        .FilterIndex  = 0,
        .FilterType   = FDCAN_FILTER_RANGE,
        .FilterConfig = FDCAN_FILTER_TO_RXFIFO0,
        .FilterID1    = 0x000,
        .FilterID2    = 0x000,
    };
    HAL_FDCAN_ConfigFilter(&hfdcan2, &filter);
    HAL_FDCAN_Start(&hfdcan2);
    HAL_FDCAN_ActivateNotification(&hfdcan2, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    Gimbal_Init();

    PID_Set_Kparam_float(&g_pid, PID_Kp, PID_Ki, PID_Kd);
    PID_Set_Target_float(&g_pid, g_target_angle);
    PID_Set_OutputLimit_float(&g_pid, SPEED_LIMIT);
    PID_Set_ErrorDeadZone_float(&g_pid, DEAD_ZONE);
    PID_Set_Integral_float(&g_pid, INTEGRAL_RANGE, INTEGRAL_LIMIT);
    PID_ClearUp_float(&g_pid);
    g_pid_inited = true;

    DM_Motor_Enable(&hfdcan2, MOTOR_TX_ID);
    osDelay(100);
    DM_Motor_SetZero(&hfdcan2, MOTOR_TX_ID);
    osDelay(50);

    /* 使能 ZDT 直线动作电机 (伸长/升降) */
    ZDT_Enable(ZDT_ID_EXTEND);
    ZDT_Enable(ZDT_ID_LIFT);

    DM_Motor_MIT_Struct mit = {
        .position_angle = 0.0f,
        .velocity_rad_s = 0.0f,
        .kp             = 0.0f,
        .kd             = MIT_KD,
        .torque_Nm      = 0.0f,
    };

    for (;;)
    {
        float current = g_motor.Status.position_angle;
        float target  = g_target_angle;

        /* 归一化: 让 target 到 current 的误差在 ±180° 内 */
        float error = target - current;
        if (error > 180.0f)  target -= 360.0f;
        if (error < -180.0f) target += 360.0f;

        PID_Set_Target_float(&g_pid, target);
        float speed = PID_Update_float(&g_pid, current);

        mit.velocity_rad_s = speed;
        DM_J4310_MIT_Send(&hfdcan2, MOTOR_TX_ID, &mit);

        osDelay(CTRL_PERIOD_MS);
    }
}

/* ====================================================================== */
/*  ZDT 直线动作                                                         */
/* ====================================================================== */

static void zdt_move(uint8_t id, int32_t pulses)
{
    if (pulses == 0) return;

    uint8_t  dir  = (pulses > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    uint16_t rpm = (uint16_t)((pulses > 0) ? pulses : -pulses);
    if (rpm > ZDT_SPEED_RPM) rpm = ZDT_SPEED_RPM;
    if (pulses < 0) pulses = -pulses;

    ZDT_SetPosition(id, dir, rpm, ZDT_ACCEL, pulses,
                    ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
}

void Gimbal_Extend(int32_t pulses)
{
    zdt_move(ZDT_ID_EXTEND, pulses);
}

void Gimbal_Lift(int32_t pulses)
{
    zdt_move(ZDT_ID_LIFT, pulses);
}

/* ====================================================================== */
/*  PWM 夹爪 (TIM1 CH3)                                                  */
/* ====================================================================== */

void Gimbal_GripperInit(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);  /* 初始闭合 */
}

void Gimbal_Gripper(uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}

/* ====================================================================== */
/*  复合命令 — 取料序列 (内部直接调 Gimbal_* API)                        */
/* ====================================================================== */

#define FETCH_PICKUP_EXTEND  500
#define FETCH_PLACE_EXTEND   300
#define FETCH_EXTEND_DIFF    (FETCH_PICKUP_EXTEND - FETCH_PLACE_EXTEND)

static float fetch_car_angle(uint8_t pos)
{
    float deg;
    if (pos == 1)      deg = 180.0f;
    else if (pos == 2) deg = 220.0f;
    else               deg = 260.0f;
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

static void ExecFetchRaw(Event_t done_event)
{
    Gimbal_Extend(FETCH_PICKUP_EXTEND);
    osDelay(100);

    for (int i = 0; i < 3; i++)
    {
        g_color_pending = true;
        while (g_color_pending) osDelay(10);

        if (g_color_result.color == expected_color[seq[i]])
        {
            Gimbal_Lift(-200); osDelay(100);
            Gimbal_Gripper(30000); osDelay(200);
            Gimbal_Lift(200); osDelay(100);
        }

        Gimbal_Extend(-FETCH_EXTEND_DIFF); osDelay(100);
        float angle = fetch_car_angle(seq[i]);
        Gimbal_SetAngle(angle);
        while (fabsf(Gimbal_GetAngle() - angle) > 1.0f) osDelay(10);

        Gimbal_Lift(-200); osDelay(100);
        Gimbal_Gripper(0); osDelay(200);
        Gimbal_Lift(200); osDelay(100);

        Gimbal_Extend(FETCH_EXTEND_DIFF); osDelay(100);
        Gimbal_SetAngle(0.0f);
        while (fabsf(Gimbal_GetAngle()) > 1.0f) osDelay(10);
    }

    Gimbal_Extend(-FETCH_PICKUP_EXTEND);
    osDelay(100);

    SM_SendEvent(done_event);
}

/* ====================================================================== */
/*  Gimbal_CmdTask — 队列驱动机械臂命令执行                               */
/* ====================================================================== */

static QueueHandle_t gimbal_cmd_queue = NULL;

void Gimbal_CmdTask(void *argument)
{
    (void)argument;
    GimbalCmd_t cmd;

    for (;;)
    {
        xQueueReceive(gimbal_cmd_queue, &cmd, portMAX_DELAY);

        switch (cmd.type) {
        case GIMBAL_CMD_FETCH_RAW:
            ExecFetchRaw(cmd.completion_event);
            break;
        case GIMBAL_CMD_PLACE_ROUGH:
        case GIMBAL_CMD_PLACE_TEMP:
        case GIMBAL_CMD_STACK_TEMP:
            /* TODO: 后续补全 */
            SM_SendEvent(cmd.completion_event);
            break;
        }
    }
}

void Gimbal_CmdInit(void)
{
    gimbal_cmd_queue = xQueueCreate(16, sizeof(GimbalCmd_t));
    xTaskCreate(Gimbal_CmdTask, "gimbal_cmd", 256, NULL, osPriorityAboveNormal, NULL);
}

int Gimbal_SendCmd(const GimbalCmd_t *cmd)
{
    if (gimbal_cmd_queue == NULL || cmd == NULL) return -1;
    return (xQueueSend(gimbal_cmd_queue, cmd, 0) == pdPASS) ? 0 : -1;
}

/* ====================================================================== */
/*  Gimbal_InitTask                                                       */
/* ====================================================================== */

void Gimbal_InitTask(void)
{
    xTaskCreate(Gimbal_Task, "gimbal", 256, NULL, osPriorityAboveNormal, NULL);
    Gimbal_CmdInit();
}
