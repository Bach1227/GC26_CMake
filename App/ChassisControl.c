#include "ChassisControl.h"
#include "config.h"
#include "bsp_zdt.h"
#include "bsp_witgyro.h"
#include "cmsis_os2.h"
#if CONFIG_USE_GIMBAL && CONFIG_VISION_ADJUST_ONLY && CONFIG_ENABLE_MATERIAL_PLACEMENT
#include "gimbal.h"
#endif
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
volatile bool g_chassis_adjust_heading_active = false;
volatile float g_chassis_adjust_heading_deg = 0.0f;
volatile ChassisVisionAxis_t g_chassis_adjust_axis =
    CHASSIS_VISION_AXIS_IDLE;
static volatile uint32_t adjust_heading_zero_generation = 0U;

static PID_Param_float rotate_pid;
static PID_Param_float heading_pid;
static PID_Param_float vision_x_pid;
static PID_Param_float vision_y_pid;

static volatile int8_t vision_offset_x = 0;
static volatile int8_t vision_offset_y = 0;
static volatile uint32_t vision_feedback_tick = 0U;
static volatile uint32_t vision_feedback_sequence = 0U;
#if CONFIG_USE_GIMBAL && CONFIG_VISION_ADJUST_ONLY && CONFIG_ENABLE_MATERIAL_PLACEMENT
static volatile bool vision_pickup_requested = false;
#endif

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

#if CONFIG_USE_GIMBAL && CONFIG_VISION_ADJUST_ONLY && CONFIG_ENABLE_MATERIAL_PLACEMENT
static void request_vision_pickup_once(void)
{
    bool should_request = false;

    taskENTER_CRITICAL();
    if (!vision_pickup_requested) {
        vision_pickup_requested = true;
        should_request = true;
    }
    taskEXIT_CRITICAL();

    if (should_request) {
        const GimbalCmd_t cmd = {
            .type = GIMBAL_CMD_VISION_PICKUP,
            .completion_event = EVENT_NONE,
        };
        (void)Gimbal_SendCmd(&cmd);
    }
}
#endif

static bool execute_vision_adjust(void)
{
    TickType_t last_wake = xTaskGetTickCount();
    ChassisVisionAxis_t axis = CHASSIS_VISION_AXIS_X;
    uint32_t last_feedback_sequence = 0U;
    uint8_t stable_frames = 0U;
    bool motors_running = false;
    bool completed = false;

    PID_Set_Kparam_float(&vision_x_pid,
                         CONFIG_VISION_ADJUST_KP_X, 0.0f, 0.0f);
    PID_Set_Kparam_float(&vision_y_pid,
                         CONFIG_VISION_ADJUST_KP_Y, 0.0f, 0.0f);
    PID_Set_Target_float(&vision_x_pid, 0.0f);
    PID_Set_Target_float(&vision_y_pid, 0.0f);
    PID_Set_ErrorDeadZone_float(&vision_x_pid,
                                CONFIG_VISION_ADJUST_DEADZONE);
    PID_Set_ErrorDeadZone_float(&vision_y_pid,
                                CONFIG_VISION_ADJUST_DEADZONE);
    PID_Set_OutputLimit_float(&vision_x_pid,
                              CONFIG_VISION_ADJUST_RPM_LIMIT);
    PID_Set_OutputLimit_float(&vision_y_pid,
                              CONFIG_VISION_ADJUST_RPM_LIMIT);
    PID_ClearUp_float(&vision_x_pid);
    PID_ClearUp_float(&vision_y_pid);

    PID_Set_Kparam_float(&heading_pid,
                         CONFIG_CHASSIS_HEADING_KP,
                         CONFIG_CHASSIS_HEADING_KI,
                         CONFIG_CHASSIS_HEADING_KD);
    PID_Set_Target_float(&heading_pid, g_chassis_adjust_heading_deg);
    PID_Set_ErrorDeadZone_float(&heading_pid,
                                CONFIG_CHASSIS_HEADING_DEAD_DEG);
    PID_Set_OutputLimit_float(&heading_pid,
                              CONFIG_VISION_ADJUST_YAW_RPM_LIMIT);
    PID_ClearUp_float(&heading_pid);

    motion_abort_requested = false;
    motion_active = true;

    while (g_chassis_adjust_heading_active &&
           !motion_abort_requested) {
        uint32_t now = HAL_GetTick();
        uint32_t feedback_tick;
        uint32_t feedback_sequence;
        int8_t offset_x;
        int8_t offset_y;
        GyroSample_t gyro;

        taskENTER_CRITICAL();
        offset_x = vision_offset_x;
        offset_y = vision_offset_y;
        feedback_tick = vision_feedback_tick;
        feedback_sequence = vision_feedback_sequence;
        taskEXIT_CRITICAL();

        if (feedback_tick != 0U &&
            (uint32_t)(now - feedback_tick)
                <= CONFIG_VISION_ADJUST_TIMEOUT_MS &&
            gyro_get_sample(&gyro) &&
            (uint32_t)(now - gyro.last_update_tick)
                <= CONFIG_CHASSIS_GYRO_TIMEOUT_MS) {
            float rpm_x = 0.0f;
            float rpm_y = 0.0f;
            float yaw_rpm;

            if (feedback_sequence != last_feedback_sequence) {
                last_feedback_sequence = feedback_sequence;

                float active_offset =
                    (axis == CHASSIS_VISION_AXIS_X)
                    ? (float)offset_x
                    : (float)offset_y;
                bool error_in_threshold =
                    (fabsf(active_offset)
                     <= CONFIG_VISION_ADJUST_DEADZONE);

                if (error_in_threshold) {
                    if (stable_frames < UINT8_MAX) {
                        ++stable_frames;
                    }
                } else {
                    stable_frames = 0U;
                }

                if (axis == CHASSIS_VISION_AXIS_X) {
                    if (stable_frames
                        >= CONFIG_VISION_ADJUST_X_STABLE_FRAMES) {
                        axis = CHASSIS_VISION_AXIS_Y;
                        stable_frames = 0U;
                        PID_ClearUp_float(&vision_x_pid);
                        PID_ClearUp_float(&vision_y_pid);
                        g_chassis_adjust_axis = axis;
                    }
                } else if (stable_frames
                           >= CONFIG_VISION_ADJUST_Y_STABLE_FRAMES) {
                    completed = true;
                    taskENTER_CRITICAL();
                    g_chassis_adjust_heading_active = false;
                    g_chassis_adjust_axis =
                        CHASSIS_VISION_AXIS_IDLE;
                    taskEXIT_CRITICAL();
                    break;
                }
            }

            if (axis == CHASSIS_VISION_AXIS_X) {
                rpm_x = -CONFIG_CHASSIS_X_DIRECTION
                      * PID_Update_float(&vision_x_pid, (float)offset_x);
            } else {
                rpm_y = -CONFIG_CHASSIS_Y_DIRECTION
                      * PID_Update_float(&vision_y_pid, (float)offset_y);
            }

            if (gyro.zero_generation != adjust_heading_zero_generation) {
                taskENTER_CRITICAL();
                g_chassis_adjust_heading_deg = gyro.angle;
                adjust_heading_zero_generation = gyro.zero_generation;
                taskEXIT_CRITICAL();
                PID_Set_Target_float(&heading_pid, gyro.angle);
                PID_ClearUp_float(&heading_pid);
            }

            yaw_rpm = PID_Update_float(&heading_pid, gyro.angle);
            heading_pid.ValueLast = gyro.angle;
            drive_translation(rpm_x, rpm_y, yaw_rpm);
            motors_running = true;
        } else if (motors_running) {
            stop_all_motors();
            motors_running = false;
            PID_ClearUp_float(&vision_x_pid);
            PID_ClearUp_float(&vision_y_pid);
            PID_ClearUp_float(&heading_pid);
        }

        vTaskDelayUntil(&last_wake,
                        pdMS_TO_TICKS(CONFIG_VISION_ADJUST_PERIOD_MS));
    }

    stop_all_motors();
    PID_ClearUp_float(&vision_x_pid);
    PID_ClearUp_float(&vision_y_pid);
    PID_ClearUp_float(&heading_pid);
    g_chassis_adjust_axis = CHASSIS_VISION_AXIS_IDLE;
    motion_active = false;
    motion_abort_requested = false;
#if CONFIG_USE_GIMBAL && CONFIG_VISION_ADJUST_ONLY && CONFIG_ENABLE_MATERIAL_PLACEMENT
    if (completed) {
        request_vision_pickup_once();
    }
#endif
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
    ChassisMoveCmd_t cmd = {
        .x = x,
        .y = y,
        .rotation = 0,
        .completion_event = completion_event,
        .run_vision_adjust = false,
    };

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
    ChassisMoveCmd_t cmd = {
        .x = 0,
        .y = 0,
        .rotation = centidegree,
        .completion_event = completion_event,
        .run_vision_adjust = false,
    };

    if (chassis_queue == NULL) {
        return -1;
    }
    if (xQueueSend(chassis_queue, &cmd, 0) == pdPASS) {
        dbg_cmd_sent++;
        return 0;
    }
    return -1;
}

static bool begin_vision_adjust(bool use_current_heading,
                                float requested_heading_deg)
{
    GyroSample_t gyro;
    ChassisMoveCmd_t cmd = {
        .x = 0,
        .y = 0,
        .rotation = 0,
#if CONFIG_VISION_ADJUST_ONLY
        .completion_event = EVENT_NONE,
#else
        .completion_event = EVENT_ADJUST_DONE,
#endif
        .run_vision_adjust = true,
    };

    if (chassis_queue == NULL ||
        !gyro_get_sample(&gyro) ||
        (uint32_t)(HAL_GetTick() - gyro.last_update_tick)
            > CONFIG_CHASSIS_GYRO_TIMEOUT_MS) {
        g_chassis_adjust_heading_active = false;
        return false;
    }

    float heading_target = gyro.angle;
    if (!use_current_heading) {
        heading_target += remainderf(requested_heading_deg - gyro.angle,
                                     360.0f);
    }

    taskENTER_CRITICAL();
    g_chassis_adjust_heading_deg = heading_target;
    adjust_heading_zero_generation = gyro.zero_generation;
    vision_offset_x = 0;
    vision_offset_y = 0;
    vision_feedback_tick = 0U;
    vision_feedback_sequence = 0U;
#if CONFIG_USE_GIMBAL && CONFIG_VISION_ADJUST_ONLY && CONFIG_ENABLE_MATERIAL_PLACEMENT
    vision_pickup_requested = false;
#endif
    g_chassis_adjust_axis = CHASSIS_VISION_AXIS_X;
    g_chassis_adjust_heading_active = true;
    taskEXIT_CRITICAL();

    if (xQueueSend(chassis_queue, &cmd, 0) != pdPASS) {
        g_chassis_adjust_heading_active = false;
        return false;
    }
    return true;
}

bool Chassis_BeginVisionAdjust(void)
{
    return begin_vision_adjust(true, 0.0f);
}

bool Chassis_BeginVisionAdjustAtHeading(float heading_deg)
{
    return begin_vision_adjust(false, heading_deg);
}

void Chassis_UpdateVisionAdjust(int8_t offset_x, int8_t offset_y)
{
    taskENTER_CRITICAL();
    vision_offset_x = offset_x;
    vision_offset_y = offset_y;
    vision_feedback_tick = HAL_GetTick();
    ++vision_feedback_sequence;
    taskEXIT_CRITICAL();
}

void Chassis_EndVisionAdjust(void)
{
    bool was_active;

    taskENTER_CRITICAL();
    was_active = g_chassis_adjust_heading_active;
    g_chassis_adjust_heading_active = false;
    g_chassis_adjust_axis = CHASSIS_VISION_AXIS_IDLE;
    taskEXIT_CRITICAL();

    if (chassis_queue != NULL) {
        (void)xQueueReset(chassis_queue);
    }
    Chassis_Stop();

#if CONFIG_USE_GIMBAL && CONFIG_VISION_ADJUST_ONLY && CONFIG_ENABLE_MATERIAL_PLACEMENT
    if (was_active) {
        request_vision_pickup_once();
    }
#else
    (void)was_active;
#endif
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

        if (cmd.run_vision_adjust) {
            completed = execute_vision_adjust();
        } else if (cmd.x == 0 && cmd.y == 0 && cmd.rotation != 0) {
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
