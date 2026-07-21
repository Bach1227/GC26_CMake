#include "gimbal.h"
#include "config.h"
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
extern uint8_t seq[2][3];

/* ====================================================================== */
/*  配置                                                                  */
/* ====================================================================== */

#define MOTOR_TX_ID         0x001u     /* 发送命令 ID */
#define MOTOR_RX_ID         0x000u     /* 接收反馈 ID */
#define ANGLE_TARGET_DEFAULT CONFIG_CAR_MATERIAL_POS_1_DEG      /* 默认目标 180° */

/* PID (角度° → 速度) */
#define PID_Kp              0.028f
#define PID_Ki              0.0010f
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
/* ====================================================================== */
/*  内部变量                                                              */
/* ====================================================================== */

DM_MotorControlBlock   g_motor  = {0};
static bool                   g_inited = false;

PID_Param_float g_pid;
float           g_target_angle = ANGLE_TARGET_DEFAULT;
bool            g_pid_inited   = false;

/* 将电机的 ±180° 周期反馈解绕为相对当前电机零点的连续角度。 */
static volatile float   g_continuous_angle = 0.0f;
static volatile float   g_prev_wrapped_angle = 0.0f;
static volatile uint8_t g_angle_tracking_valid = 0;

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

        float wrapped = g_motor.Status.position_angle;
        if (!g_angle_tracking_valid)
        {
            g_prev_wrapped_angle = wrapped;
            g_continuous_angle = wrapped;
            g_angle_tracking_valid = 1;
        }
        else
        {
            float delta = wrapped - g_prev_wrapped_angle;
            if (delta > 180.0f)       delta -= 360.0f;
            else if (delta < -180.0f) delta += 360.0f;

            g_continuous_angle += delta;
            g_prev_wrapped_angle = wrapped;
        }
    }
}

/* ====================================================================== */
/*  Gimbal_SetAngle / Gimbal_GetAngle                                     */
/* ====================================================================== */

static float clamp_gimbal_angle(float angle_deg)
{
#if CONFIG_GIMBAL_SINGLE_TURN_ENABLE
    if (angle_deg < CONFIG_GIMBAL_TURN_MIN_DEG)
        return CONFIG_GIMBAL_TURN_MIN_DEG;
    if (angle_deg > CONFIG_GIMBAL_TURN_MAX_DEG)
        return CONFIG_GIMBAL_TURN_MAX_DEG;
#endif
    return angle_deg;
}

void Gimbal_SetAngle(float angle_deg)
{
    g_target_angle = clamp_gimbal_angle(angle_deg);
    if (g_pid_inited)
        PID_Set_Target_float(&g_pid, g_target_angle);
}

float Gimbal_GetAngle(void)
{
    return g_continuous_angle;
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

#if CONFIG_DM_RESET_ZERO_ON_BOOT
    DM_Motor_SetZero(&hfdcan2, MOTOR_TX_ID);
    osDelay(50);

    /* 设零后的下一帧作为连续角度跟踪起点。 */
    g_continuous_angle = 0.0f;
    g_angle_tracking_valid = 0;
#endif

    DM_Motor_MIT_Struct mit = {
        .position_angle = 0.0f,
        .velocity_rad_s = 0.0f,
        .kp             = 0.0f,
        .kd             = MIT_KD,
        .torque_Nm      = 0.0f,
    };

    for (;;)
    {
#if CONFIG_DM_SLOW_ROTATE_TEST
        /* MIT 速度模式: kp=0，由 kd 对速度误差提供阻尼/转矩。 */
        mit.velocity_rad_s = CONFIG_DM_SLOW_ROTATE_SPEED;
#else
        float current = g_continuous_angle;
        float target  = g_target_angle;

        /* 连续角度直接闭环，不允许周期角度折返后累计出第二圈。 */
        PID_Set_Target_float(&g_pid, target);
        float speed = PID_Update_float(&g_pid, current);

        mit.velocity_rad_s = speed;
#endif

#if CONFIG_GIMBAL_SINGLE_TURN_ENABLE
        /* 即使目标或测试速度异常，也禁止累计角度驶入第二圈。 */
        float limit_angle = g_continuous_angle;
        if ((limit_angle <= CONFIG_GIMBAL_TURN_MIN_DEG && mit.velocity_rad_s < 0.0f) ||
            (limit_angle >= CONFIG_GIMBAL_TURN_MAX_DEG && mit.velocity_rad_s > 0.0f))
        {
            mit.velocity_rad_s = 0.0f;
        }
#endif
        DM_J4310_MIT_Send(&hfdcan2, MOTOR_TX_ID, &mit);
        // DM_J4310_MIT_Send(&hfdcan2, MOTOR_TX_ID, &mit);

        osDelay(CTRL_PERIOD_MS);
    }
}

/* ====================================================================== */
/*  ZDT 直线动作                                                         */
/* ====================================================================== */

static void zdt_move(uint8_t id, int32_t pulses,
                     uint16_t speed_rpm, uint8_t accel)
{
    if (pulses == 0) return;

    uint8_t dir = (pulses > 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    if (pulses < 0) pulses = -pulses;

    ZDT_SetPosition(id, dir, speed_rpm, accel, pulses,
                    ZDT_POS_RELATIVE, ZDT_SYNC_IMMEDIATE);
}

void Gimbal_Extend(int32_t pulses)
{
    zdt_move(ZDT_ID_EXTEND, pulses,
             CONFIG_STEPPER_EXTEND_SPEED_RPM,
             CONFIG_STEPPER_EXTEND_ACCEL);
}

void Gimbal_Lift(int32_t pulses)
{
    zdt_move(ZDT_ID_LIFT, pulses,
             CONFIG_STEPPER_LIFT_SPEED_RPM,
             CONFIG_STEPPER_LIFT_ACCEL);
}

/* ====================================================================== */
/*  PWM 夹爪 (TIM1 CH3)                                                  */
/* ====================================================================== */

void Gimbal_GripperInit(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3,
                          CONFIG_GRIPPER_CLOSE_PULSE_US);  /* 初始闭合 */
}

void Gimbal_Gripper(uint32_t pulse)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, pulse);
}

/* ====================================================================== */
/*  复合命令 — 取料序列 (内部直接调 Gimbal_* API)                        */
/* ====================================================================== */

#define FETCH_PICKUP_EXTEND  1500
#define FETCH_PLACE_EXTEND   300
#define FETCH_EXTEND_DIFF    (FETCH_PICKUP_EXTEND - FETCH_PLACE_EXTEND)

static float fetch_car_angle(uint8_t pos)
{
    float deg;
    if (pos == 1)      deg = CONFIG_CAR_MATERIAL_POS_1_DEG;
    else if (pos == 2) deg = CONFIG_CAR_MATERIAL_POS_2_DEG;
    else               deg = CONFIG_CAR_MATERIAL_POS_3_DEG;
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

static void ExecFetchRaw(Event_t done_event)
{
    // ZDT_SetVelocity(ZDT_ID_LIFT, ZDT_DIR_CW,
    //                 CONFIG_STEPPER_LIFT_SPEED_RPM,
    //                 CONFIG_STEPPER_LIFT_ACCEL,
    //                 ZDT_SYNC_IMMEDIATE);

    float folded_angle = CONFIG_MAP_MATERIAL_POS_2_DEG;
    Gimbal_SetAngle(folded_angle);
    osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);
    Gimbal_Extend(FETCH_PICKUP_EXTEND);
    osDelay(100);
    Gimbal_Gripper(CONFIG_GRIPPER_OPEN_PULSE_US);
    osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);
    for (int i = 0; i < 3; i++)
    {
        // if (wait_expected_color(seq[0][i]))
        // {
            Gimbal_Lift(-CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES);
            osDelay(CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS);
            Gimbal_Gripper(CONFIG_GRIPPER_CLOSE_PULSE_US);
            osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);
            Gimbal_Lift(CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES);
            osDelay(CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS);
        // }

        Gimbal_Extend(-FETCH_EXTEND_DIFF);
        osDelay(100);

        float angle = fetch_car_angle(seq[0][i]);
        Gimbal_SetAngle(angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Gripper(CONFIG_GRIPPER_OPEN_PULSE_US);
        osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);

        Gimbal_Extend(FETCH_EXTEND_DIFF);
        osDelay(100);

        float folded_angle = CONFIG_MAP_MATERIAL_POS_2_DEG;
        Gimbal_SetAngle(folded_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);
    }

    Gimbal_Extend(-FETCH_PICKUP_EXTEND);
    osDelay(100);



    if (done_event != EVENT_NONE) {
        SM_SendEvent(done_event);
    }
}

static void ExecVisionPickup(Event_t done_event)
{
    uint8_t color = g_vision_feedback.color;
    if (color < 1U || color > 3U) {
        return;
    }

    if (CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES != 0) {
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS);
    }

    Gimbal_Gripper(CONFIG_GRIPPER_CLOSE_PULSE_US);
    osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);

    if (CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES != 0) {
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS);
    }

    float car_target = fetch_car_angle(color);
    Gimbal_SetAngle(car_target);
    osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

    if (CONFIG_GIMBAL_LIFT_CAR_PULSES != 0) {
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
    }

    Gimbal_Gripper(CONFIG_GRIPPER_OPEN_PULSE_US);
    osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);

    if (CONFIG_GIMBAL_LIFT_CAR_PULSES != 0) {
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
    }

    Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);

    if (done_event != EVENT_NONE) {
        SM_SendEvent(done_event);
    }
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
        case GIMBAL_CMD_VISION_PICKUP:
            ExecVisionPickup(cmd.completion_event);
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
