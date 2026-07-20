#include "ChassisControl.h"
#include "config.h"
#include "bsp_zdt.h"
#include "bsp_witgyro.h"
#include "cmsis_os2.h"
#include "PID.h"
#include "queue.h"
#include "timers.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>

/* ====================================================================== */
/*  Command queue and control state                                       */
/* ====================================================================== */

#define CHASSIS_QUEUE_LEN       8U
#define CHASSIS_STOP_TIMEOUT_MS 20U

#define ROTATE_PID_KP           0.05f
#define ROTATE_PID_KI           0.0f
#define ROTATE_PID_KD           0.002f
#define ROTATE_SPEED_LIMIT      2.0f
#define ROTATE_ANGLE_DEAD       3.0f
#define ROTATE_RPM_PER_RAD_S    30.0f

static QueueHandle_t chassis_queue = NULL;
static TimerHandle_t control_timer = NULL;

TaskHandle_t ChassisTaskHandle = NULL;

static volatile bool motion_active = false;
static volatile bool motion_abort_requested = false;

static PID_Param_float rotate_pid;
static PID_Param_float heading_pid;

static volatile uint32_t dbg_cmd_sent = 0;
static volatile uint32_t dbg_cmd_recv = 0;

/* ====================================================================== */
/*  Gyroscope continuous-yaw helper                                       */
/* ====================================================================== */

typedef struct {
    float angle;
    uint32_t last_update_tick;
    uint32_t zero_generation;
    bool valid;
} GyroSample_t;

static bool gyro_get_sample(GyroSample_t *sample)
{
    float pitch;
    float roll;
    float yaw;
    static float cached = 0.0f;
    static float prev_raw = 0.0f;
    static float unwrap_offset = 0.0f;
    static uint32_t last_zero_generation = 0;
    static uint32_t last_update_tick = 0;
    static bool valid = false;

    if (sample == NULL) {
        return false;
    }

    if (WitGyro_GetAngle(&pitch, &roll, &yaw))
    {
        uint32_t zero_generation = WitGyro_GetZeroGeneration();

        if (!valid || zero_generation != last_zero_generation) {
            prev_raw = yaw;
            unwrap_offset = 0.0f;
            cached = yaw;
            last_zero_generation = zero_generation;
        } else {
            float delta = yaw - prev_raw;
            if (delta > 180.0f) {
                unwrap_offset -= 360.0f;
            } else if (delta < -180.0f) {
                unwrap_offset += 360.0f;
            }
            prev_raw = yaw;
            cached = yaw + unwrap_offset;
        }

        last_update_tick = HAL_GetTick();
        valid = true;
    }

    sample->angle = cached;
    sample->last_update_tick = last_update_tick;
    sample->zero_generation = last_zero_generation;
    sample->valid = valid;
    return valid;
}

/* ====================================================================== */
/*  Signed wheel-speed output                                             */
/* ====================================================================== */

static int16_t clamp_motor_rpm(int32_t rpm)
{
    if (rpm > CONFIG_CHASSIS_MOTOR_RPM_LIMIT) {
        return CONFIG_CHASSIS_MOTOR_RPM_LIMIT;
    }
    if (rpm < -CONFIG_CHASSIS_MOTOR_RPM_LIMIT) {
        return -CONFIG_CHASSIS_MOTOR_RPM_LIMIT;
    }
    return (int16_t)rpm;
}

static ZDT_VelocityCommand_t make_wheel_velocity(uint8_t id,
                                                  int16_t signed_rpm)
{
    ZDT_VelocityCommand_t command;

    command.addr = id;
    command.dir = (signed_rpm >= 0) ? ZDT_DIR_CW : ZDT_DIR_CCW;
    command.speed_rpm = (uint16_t)((signed_rpm >= 0)
                      ? signed_rpm : -(int32_t)signed_rpm);
    command.accel = CONFIG_STEPPER_CHASSIS_ACCEL;
    return command;
}

static void drive_wheel_rpm(float rpm_1, float rpm_2,
                            float rpm_3, float rpm_4)
{
    int16_t motor_1 = clamp_motor_rpm((int32_t)lroundf(
                         rpm_1 * CONFIG_CHASSIS_MOTOR_1_POLARITY));
    int16_t motor_2 = clamp_motor_rpm((int32_t)lroundf(
                         rpm_2 * CONFIG_CHASSIS_MOTOR_2_POLARITY));
    int16_t motor_3 = clamp_motor_rpm((int32_t)lroundf(
                         rpm_3 * CONFIG_CHASSIS_MOTOR_3_POLARITY));
    int16_t motor_4 = clamp_motor_rpm((int32_t)lroundf(
                         rpm_4 * CONFIG_CHASSIS_MOTOR_4_POLARITY));
    ZDT_VelocityCommand_t commands[4] = {
        make_wheel_velocity(1U, motor_1),
        make_wheel_velocity(2U, motor_2),
        make_wheel_velocity(3U, motor_3),
        make_wheel_velocity(4U, motor_4)
    };

    /* 周期控制不等待 UART：总线忙时保留上一周期轮速。 */
    (void)ZDT_SetVelocitySyncBatch(commands, 4U, 0U);
}

static void drive_translation(float rpm_x, float rpm_y, float yaw_rpm)
{
    yaw_rpm *= CONFIG_CHASSIS_YAW_OUTPUT_DIRECTION;

    drive_wheel_rpm(rpm_y + yaw_rpm,
                    rpm_x - yaw_rpm,
                    rpm_y - yaw_rpm,
                    rpm_x + yaw_rpm);
}

static void drive_rotation(float speed_rad_s)
{
    float yaw_rpm = speed_rad_s * ROTATE_RPM_PER_RAD_S
                  * CONFIG_CHASSIS_YAW_OUTPUT_DIRECTION;

    drive_wheel_rpm(yaw_rpm, -yaw_rpm, -yaw_rpm, yaw_rpm);
}

static void stop_all_motors(void)
{
    static const uint8_t addresses[4] = {1U, 2U, 3U, 4U};
    (void)ZDT_StopBatch(addresses, 4U, CHASSIS_STOP_TIMEOUT_MS);
}

/* ====================================================================== */
/*  Timer and command helpers                                             */
/* ====================================================================== */

static void ControlTimerCallback(TimerHandle_t xTimer)
{
    (void)xTimer;

    if (ChassisTaskHandle != NULL) {
        xTaskNotifyGive(ChassisTaskHandle);
    }
}

static void drain_control_notifications(void)
{
    while (ulTaskNotifyTake(pdTRUE, 0) != 0U) {
    }
}

static uint32_t calculate_translation(uint32_t *duration_ms,
                                      float *rpm_x, float *rpm_y,
                                      int16_t x_mm, int16_t y_mm)
{
    float x_time_s;
    float y_time_s;
    float duration_s;

    if (duration_ms == NULL || rpm_x == NULL || rpm_y == NULL) {
        return 0U;
    }
    if (x_mm == 0 && y_mm == 0) {
        *duration_ms = 0U;
        *rpm_x = 0.0f;
        *rpm_y = 0.0f;
        return 0U;
    }

    x_time_s = fabsf((float)x_mm) / CONFIG_CHASSIS_X_SPEED_MM_S;
    y_time_s = fabsf((float)y_mm) / CONFIG_CHASSIS_Y_SPEED_MM_S;
    duration_s = fmaxf(x_time_s, y_time_s);

    if (duration_s < ((float)CONFIG_CHASSIS_MIN_MOVE_MS / 1000.0f)) {
        duration_s = (float)CONFIG_CHASSIS_MIN_MOVE_MS / 1000.0f;
    }

    *duration_ms = (uint32_t)ceilf(duration_s * 1000.0f);
    *rpm_x = CONFIG_CHASSIS_X_DIRECTION * CONFIG_CHASSIS_TRANSLATE_RPM
           * ((float)x_mm / (CONFIG_CHASSIS_X_SPEED_MM_S * duration_s));
    *rpm_y = CONFIG_CHASSIS_Y_DIRECTION * CONFIG_CHASSIS_TRANSLATE_RPM
           * ((float)y_mm / (CONFIG_CHASSIS_Y_SPEED_MM_S * duration_s));
    return *duration_ms;
}

/* ====================================================================== */
/*  Motion execution in ChassisTask context                               */
/* ====================================================================== */

static bool execute_translation(const ChassisMoveCmd_t *cmd)
{
    uint32_t duration_ms;
    uint32_t start_tick;
    uint32_t heading_zero_generation = 0;
    float rpm_x;
    float rpm_y;
    bool heading_locked = false;
    bool completed = false;

    if (cmd == NULL) {
        return false;
    }
    if (calculate_translation(&duration_ms, &rpm_x, &rpm_y,
                              cmd->x, cmd->y) == 0U) {
        return true;
    }

    PID_Set_Kparam_float(&heading_pid,
                         CONFIG_CHASSIS_HEADING_KP,
                         CONFIG_CHASSIS_HEADING_KI,
                         CONFIG_CHASSIS_HEADING_KD);
    PID_Set_ErrorDeadZone_float(&heading_pid,
                                CONFIG_CHASSIS_HEADING_DEAD_DEG);
    PID_Set_OutputLimit_float(&heading_pid,
                              CONFIG_CHASSIS_YAW_RPM_LIMIT);
    PID_ClearUp_float(&heading_pid);

    drain_control_notifications();
    motion_abort_requested = false;
    motion_active = true;
    start_tick = HAL_GetTick();
    xTimerStart(control_timer, 0);

    for (;;)
    {
        uint32_t now = HAL_GetTick();
        float yaw_rpm = 0.0f;
        GyroSample_t gyro;

        if (motion_abort_requested) {
            break;
        }
        if ((uint32_t)(now - start_tick) >= duration_ms) {
            completed = true;
            break;
        }

        if (gyro_get_sample(&gyro) &&
            (uint32_t)(now - gyro.last_update_tick)
                <= CONFIG_CHASSIS_GYRO_TIMEOUT_MS)
        {
            if (!heading_locked ||
                gyro.zero_generation != heading_zero_generation)
            {
                PID_Set_Target_float(&heading_pid, gyro.angle);
                PID_ClearUp_float(&heading_pid);
                heading_pid.ValueLast = gyro.angle;
                heading_zero_generation = gyro.zero_generation;
                heading_locked = true;
            }

            yaw_rpm = PID_Update_float(&heading_pid, gyro.angle);
            heading_pid.ValueLast = gyro.angle;
        }
        else
        {
            /* Gyro stale: finish this command with timed open-loop motion. */
            heading_locked = false;
            PID_ClearUp_float(&heading_pid);
        }

        drive_translation(rpm_x, rpm_y, yaw_rpm);
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    xTimerStop(control_timer, 0);
    stop_all_motors();
    osDelay(CONFIG_CHASSIS_STOP_SETTLE_MS);
    PID_ClearUp_float(&heading_pid);
    motion_active = false;
    motion_abort_requested = false;
    drain_control_notifications();
    return completed;
}

static bool execute_rotation(const ChassisMoveCmd_t *cmd)
{
    bool completed = false;

    if (cmd == NULL) {
        return false;
    }

    PID_Set_Kparam_float(&rotate_pid,
                         ROTATE_PID_KP, ROTATE_PID_KI, ROTATE_PID_KD);
    PID_Set_Target_float(&rotate_pid, (float)cmd->rotation / 100.0f);
    PID_Set_OutputLimit_float(&rotate_pid, ROTATE_SPEED_LIMIT);
    PID_ClearUp_float(&rotate_pid);

    drain_control_notifications();
    motion_abort_requested = false;
    motion_active = true;
    xTimerStart(control_timer, 0);

    for (;;)
    {
        GyroSample_t gyro;

        if (motion_abort_requested) {
            break;
        }

        if (gyro_get_sample(&gyro))
        {
            float speed = PID_Update_float(&rotate_pid, gyro.angle);
            rotate_pid.ValueLast = gyro.angle;

            if (fabsf(rotate_pid.ErrorNow) < ROTATE_ANGLE_DEAD) {
                completed = true;
                break;
            }

            drive_rotation(speed);
        }

        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    }

    xTimerStop(control_timer, 0);
    stop_all_motors();
    osDelay(CONFIG_CHASSIS_STOP_SETTLE_MS);
    PID_ClearUp_float(&rotate_pid);
    motion_active = false;
    motion_abort_requested = false;
    drain_control_notifications();
    return completed;
}

/* ====================================================================== */
/*  Public command API                                                    */
/* ====================================================================== */

int Chassis_SendMoveCmd(int16_t x, int16_t y, Event_t completion_event)
{
    ChassisMoveCmd_t cmd = {x, y, 0, completion_event};

    if (chassis_queue == NULL) {
        return -1;
    }
    if (xQueueSend(chassis_queue, &cmd, 0) == pdPASS) {
        dbg_cmd_sent++;
        return 0;
    }
    return -1;
}

int Chassis_SendRotateCmd(float degrees, Event_t completion_event)
{
    int16_t centidegree = (int16_t)(degrees * 100.0f);
    ChassisMoveCmd_t cmd = {0, 0, centidegree, completion_event};

    if (chassis_queue == NULL) {
        return -1;
    }
    if (xQueueSend(chassis_queue, &cmd, 0) == pdPASS) {
        dbg_cmd_sent++;
        return 0;
    }
    return -1;
}

void Chassis_Task(void *argument)
{
    ChassisMoveCmd_t cmd;

    (void)argument;
    ChassisTaskHandle = xTaskGetCurrentTaskHandle();

    control_timer = xTimerCreate("chassis_ctl",
                                 pdMS_TO_TICKS(CONFIG_CHASSIS_CONTROL_PERIOD_MS),
                                 pdTRUE, NULL, ControlTimerCallback);
    if (control_timer == NULL) {
        vTaskDelete(NULL);
        return;
    }

    /* 调试安全：任务开始处理状态机命令前，先停止全部底盘电机。 */
    stop_all_motors();
    osDelay(CONFIG_CHASSIS_STOP_SETTLE_MS);

    for (;;)
    {
        bool completed;

        if (xQueueReceive(chassis_queue, &cmd, portMAX_DELAY) != pdPASS) {
            continue;
        }
        dbg_cmd_recv++;

        if (cmd.x == 0 && cmd.y == 0 && cmd.rotation != 0) {
            completed = execute_rotation(&cmd);
        } else {
            completed = execute_translation(&cmd);
        }

        if (completed && cmd.completion_event != EVENT_NONE) {
            SM_SendEvent(cmd.completion_event);
        }
    }
}

void Chassis_TaskInit(void)
{
    chassis_queue = xQueueCreate(CHASSIS_QUEUE_LEN, sizeof(ChassisMoveCmd_t));
    if (chassis_queue == NULL) {
        return;
    }

    xTaskCreate(Chassis_Task, "chassitask", 256, NULL,
                osPriorityAboveNormal1, &ChassisTaskHandle);
}

void Chassis_Stop(void)
{
    if (motion_active) {
        motion_abort_requested = true;
        if (ChassisTaskHandle != NULL) {
            xTaskNotifyGive(ChassisTaskHandle);
        }
        return;
    }

    stop_all_motors();
}

void Chassis_Enable(bool en)
{
    if (en) {
        ZDT_Enable(1);
        ZDT_Enable(2);
        ZDT_Enable(3);
        ZDT_Enable(4);
    } else {
        Chassis_Stop();
        ZDT_Disable(1);
        ZDT_Disable(2);
        ZDT_Disable(3);
        ZDT_Disable(4);
    }
}
