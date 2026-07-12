#include "statemachine.h"
#include "config.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "ChassisControl.h"

#ifdef USE_GIMBAL
#include "Gimbal.h"
#endif

#include "protocol.h"
#include <math.h>

/* ====================================================================== */
/*  内部变量                                                              */
/* ====================================================================== */

State_t               sm_currentState;
static const StateMachineTable_t *sm_table      = NULL;
static uint32_t              sm_tableSize = 0;

/* ====================================================================== */
/*  事件队列                                                              */
/* ====================================================================== */

#define SM_QUEUE_LENGTH     16

static QueueHandle_t sm_queue = NULL;

uint8_t seq[3] = {2, 3, 1};

/* 颜色校验 (被 protocol.c 和 gimbal.c 引用, 始终定义) */
const uint8_t expected_color[4] = {0, 2, 1, 3};
volatile ColorConfirm_t g_color_result = {0};
volatile bool           g_color_pending = false;

#ifdef USE_GIMBAL

/* 伸长距离 */
#define PICKUP_EXTEND  500
#define PLACE_EXTEND   300
#define EXTEND_DIFF    (PICKUP_EXTEND - PLACE_EXTEND)  /* 200 */
#define CAR_EXTEND      300
#define MAP_EXTEND_1_3  400
#define MAP_EXTEND_2    200

static int32_t map_extend(uint8_t pos) {
    return (pos == 2) ? MAP_EXTEND_2 : MAP_EXTEND_1_3;
}

static void Gripper_Close(void) { Gimbal_Gripper(30000); }
static void Gripper_Open(void)  { Gimbal_Gripper(0); }

static bool wait_expected_color(uint8_t pos)
{
#if CONFIG_SKIP_COLOR_CONFIRM
    (void)pos;
    return true;
#else
    g_color_pending = true;
    while (g_color_pending)
        osDelay(10);
    return g_color_result.color == expected_color[pos];
#endif
}

#endif /* USE_GIMBAL */

/* 前向声明 */
static void SM_Task(void *argument);

static void SM_ChassisMove(int16_t x, int16_t y, Event_t completion_event)
{
#if CONFIG_USE_CHASSIS
    Chassis_SendMoveCmd(x, y, completion_event);
#else
    (void)x;
    (void)y;
    if (completion_event != EVENT_NONE)
        SM_SendEvent(completion_event);
#endif
}

/* ====================================================================== */
/*  动作函数                                                              */
/* ====================================================================== */

static void Action_Nop(void)
{
    SM_SendEvent(EVENT_ADJUST_DONE);
}

/* ---- 初始化与准备 ---- */

static void Action_Start(void)
{
    SM_ChassisMove(50, 50, EVENT_NONE);      /* 中间点 */
    SM_ChassisMove(300, 0, EVENT_ARRIVED);    /* 到 QR 位 */
}

static void Action_ParseQR(void)
{
    /* TODO: 实际扫码解析 */
    SM_SendEvent(EVENT_ACTION_DONE);
}

/* ---- 第一批次 ---- */

#ifdef USE_GIMBAL

/* 车体上的 3 个物料位置角度 */
static float car_angle(uint8_t pos)
{
    float deg;
    if (pos == 1)      deg = CONFIG_CAR_MATERIAL_POS_1_DEG;
    else if (pos == 2) deg = CONFIG_CAR_MATERIAL_POS_2_DEG;
    else               deg = CONFIG_CAR_MATERIAL_POS_3_DEG;
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

/* 地图上对应工位的 3 个物料位置角度 */
static float map_angle(uint8_t pos)
{
    float deg;
    if (pos == 1)      deg = CONFIG_MAP_MATERIAL_POS_1_DEG;
    else if (pos == 2) deg = CONFIG_MAP_MATERIAL_POS_2_DEG;
    else               deg = CONFIG_MAP_MATERIAL_POS_3_DEG;
    return deg;
}

#endif /* USE_GIMBAL */

static void Action_MoveToRaw1(void)
{
    SM_ChassisMove(800, 0, EVENT_ARRIVED);
#ifdef USE_GIMBAL
    /* 前往原料区途中将云台从内收位折到地图中间位。 */
    Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
#endif
}

static void Action_FetchRaw1(void)
{
#ifdef USE_GIMBAL
    GimbalCmd_t cmd = {GIMBAL_CMD_FETCH_RAW, EVENT_ACTION_DONE};
    Gimbal_SendCmd(&cmd);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_MoveToRough1(void)
{
    SM_ChassisMove(-400, 0, EVENT_NONE);        /* 左移到主干道 */
    // Chassis_SendRotateCmd(90.0f, EVENT_NONE);         /* 逆时针转180° (陀螺仪闭环) */
    SM_ChassisMove(0, 1800, EVENT_ARRIVED);      /* 直行到粗加工区 */
}

static void Action_PlaceRough1(void)
{
#ifdef USE_GIMBAL
    /* 从车身取料 → 放到粗加工区 */
    for (int i = 0; i < 3; i++)
    {
        /* 从车身对应角度取料 */
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);

        /* 转到工位对应角度放置 */
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        int32_t m_ext = map_extend(seq[i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }

    /* 取回: 从工位取 → 放回车身 */
    for (int i = 0; i < 3; i++)
    {
        /* 转到工位对应角度取料 */
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        int32_t m_ext = map_extend(seq[i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);

        /* 去车身对应角度放料 */
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);
    }

    Gimbal_SetAngle(0);
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_MoveToTemp1(void)
{
    SM_ChassisMove(1000, 0, EVENT_NONE);
    SM_ChassisMove(0, -1000, EVENT_ARRIVED);
}

static void Action_PlaceTemp1(void)
{
#ifdef USE_GIMBAL
    /* 从车身取料 → 放到暂存区 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        /* 从车身对应角度取料 */
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);

        /* 转到暂存区对应角度放置 */
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        int32_t m_ext = map_extend(seq[i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }
    Gimbal_SetAngle(0.0f);
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

/* ---- 第二批次 ---- */

static void Action_MoveToRaw2(void)
{
    SM_ChassisMove(0, 500, EVENT_NONE);
    SM_ChassisMove(-500, 0, EVENT_ARRIVED);
}

static void Action_FetchRaw2(void)
{
#ifdef USE_GIMBAL
    /* 同第一批: 伸出→等颜色→夹取→转放料位→释放→归零 */
    Gimbal_Extend(PICKUP_EXTEND);
    osDelay(100);

    for (int i = 0; i < 3; i++)
    {
        if (wait_expected_color(seq[i]))
        {
            Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
            osDelay(100);
            Gripper_Close();
            osDelay(200);
            Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
            osDelay(100);
        }

        Gimbal_Extend(-EXTEND_DIFF);
        osDelay(100);
        float angle = car_angle(seq[i]);
        Gimbal_SetAngle(angle);
        while (fabsf(Gimbal_GetAngle() - angle) > 1.0f)
            osDelay(10);

        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);

        Gimbal_Extend(EXTEND_DIFF);
        osDelay(100);
        Gimbal_SetAngle(0.0f);
        while (fabsf(Gimbal_GetAngle()) > 1.0f)
            osDelay(10);
    }

    Gimbal_Extend(-PICKUP_EXTEND);
    osDelay(100);

    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_MoveToRough2(void)
{
    SM_ChassisMove(-400, 0, EVENT_NONE);
    SM_ChassisMove(0, -1800, EVENT_ARRIVED);
}

static void Action_PlaceRough2(void)
{
#ifdef USE_GIMBAL
    /* 从车身取料 → 放到粗加工区 */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);

        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        int32_t m_ext = map_extend(seq[i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }

    /* 取回 */
    for (int i = 0; i < 3; i++)
    {
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        int32_t m_ext = map_extend(seq[i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);

        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);
    }

    Gimbal_SetAngle(0.0f);
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_MoveToTemp2(void)
{
    SM_ChassisMove(1000, 0, EVENT_NONE);
    SM_ChassisMove(0, -1000, EVENT_ARRIVED);
}

static void Action_StackTemp2(void)
{
#ifdef USE_GIMBAL
    /* 从车身取料 → 放到暂存区码垛 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);

        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        int32_t m_ext = map_extend(seq[i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }
    Gimbal_SetAngle(0.0f);
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

/* ---- 收尾 ---- */

static void Action_ReturnStart(void)
{
    SM_ChassisMove(0, 800, EVENT_NONE);
    SM_ChassisMove(-1600, 0, EVENT_ARRIVED);
}

/* ====================================================================== */
/*  默认状态转换表 (线性查表法)                                           */
/* ====================================================================== */

const StateMachineTable_t SM_DefaultTable[] = {
    /* 当前状态            事件                动作                   下一状态 */
    {STATE_IDLE,            EVENT_START,        Action_Start,        STATE_MOVE_TO_QR},
    {STATE_MOVE_TO_QR,      EVENT_ARRIVED,      Action_ParseQR,      STATE_READ_QR},
    {STATE_READ_QR,         EVENT_ACTION_DONE,  Action_MoveToRaw1,   STATE_MOVE_TO_RAW_1},

    {STATE_MOVE_TO_RAW_1,   EVENT_ARRIVED,      Action_Nop,          STATE_ADJUST_RAW_1},
    {STATE_ADJUST_RAW_1,    EVENT_ADJUST_DONE,   Action_FetchRaw1,    STATE_FETCH_RAW_1},
    {STATE_FETCH_RAW_1,     EVENT_ACTION_DONE,  Action_MoveToRough1, STATE_MOVE_TO_ROUGH_1},
    {STATE_MOVE_TO_ROUGH_1, EVENT_ARRIVED,      Action_Nop, STATE_ADJUST_ROUGH_1},
    {STATE_ADJUST_ROUGH_1,  EVENT_ADJUST_DONE,   Action_PlaceRough1,  STATE_PLACE_ROUGH_1},
    {STATE_PLACE_ROUGH_1,   EVENT_ACTION_DONE,  Action_MoveToTemp1,  STATE_MOVE_TO_TEMP_1},
    {STATE_MOVE_TO_TEMP_1,  EVENT_ARRIVED,      Action_Nop,  STATE_ADJUST_TEMP_1},
    {STATE_ADJUST_TEMP_1,   EVENT_ADJUST_DONE,   Action_PlaceTemp1,   STATE_PLACE_TEMP_1},
    {STATE_PLACE_TEMP_1,    EVENT_ACTION_DONE,  Action_MoveToRaw2,   STATE_MOVE_TO_RAW_2},

    {STATE_MOVE_TO_RAW_2,   EVENT_ARRIVED,      Action_Nop,   STATE_ADJUST_RAW_2},
    {STATE_ADJUST_RAW_2,    EVENT_ADJUST_DONE,   Action_FetchRaw2,    STATE_FETCH_RAW_2},
    {STATE_FETCH_RAW_2,     EVENT_ACTION_DONE,  Action_MoveToRough2, STATE_MOVE_TO_ROUGH_2},
    {STATE_MOVE_TO_ROUGH_2, EVENT_ARRIVED,      Action_Nop, STATE_ADJUST_ROUGH_2},
    {STATE_ADJUST_ROUGH_2,  EVENT_ADJUST_DONE,   Action_PlaceRough2,  STATE_PLACE_ROUGH_2},
    {STATE_PLACE_ROUGH_2,   EVENT_ACTION_DONE,  Action_MoveToTemp2,  STATE_MOVE_TO_TEMP_2},
    {STATE_MOVE_TO_TEMP_2,  EVENT_ARRIVED,      Action_Nop,  STATE_ADJUST_TEMP_2},
    {STATE_ADJUST_TEMP_2,   EVENT_ADJUST_DONE,   Action_StackTemp2,   STATE_STACK_TEMP_2},
    {STATE_STACK_TEMP_2,    EVENT_ACTION_DONE,  Action_ReturnStart,  STATE_RETURN_START},

    {STATE_RETURN_START,    EVENT_ARRIVED,      Action_Nop,          STATE_IDLE},
};

const uint32_t SM_DefaultTableSize = sizeof(SM_DefaultTable) / sizeof(SM_DefaultTable[0]);

/* ====================================================================== */
/*  SM_Init                                                               */
/* ====================================================================== */

void SM_Init(void)
{
    sm_table       = SM_DefaultTable;
    sm_tableSize   = SM_DefaultTableSize;
    sm_currentState = STATE_IDLE;

    sm_queue = xQueueCreate(SM_QUEUE_LENGTH, sizeof(Event_t));

    xTaskCreate(SM_Task, "sm_event", 128, NULL, osPriorityNormal, NULL);

    SM_SendEvent(EVENT_START);
}

/* ====================================================================== */
/*  SM_GetState                                                            */
/* ====================================================================== */

State_t SM_GetState(void)
{
    return sm_currentState;
}

/* ====================================================================== */
/*  SM_ProcessEvent — 事件处理引擎 (查表法)                                */
/* ====================================================================== */

void SM_ProcessEvent(Event_t event)
{
    if (sm_table == NULL) return;

    for (uint32_t i = 0; i < sm_tableSize; i++)
    {
        const StateMachineTable_t *entry = &sm_table[i];

        if (entry->currentState == sm_currentState && entry->event == event)
        {
            if (entry->action != NULL)
                entry->action();

            sm_currentState = entry->nextState;
            return;
        }
    }

    /* 无匹配: 忽略 */
}

/* ====================================================================== */
/*  SM_Reset                                                               */
/* ====================================================================== */

void SM_Reset(State_t initialState)
{
    sm_currentState = initialState;
}

/* ====================================================================== */
/*  SM_Task — 事件处理任务                                                */
/* ====================================================================== */

static void SM_Task(void *argument)
{
    (void)argument;
    Event_t event;

    for (;;)
    {
        if (xQueueReceive(sm_queue, &event, portMAX_DELAY) == pdPASS)
        {
            SM_ProcessEvent(event);
        }
    }
}

/* ====================================================================== */
/*  SM_SendEvent / SM_SendEventFromISR                                    */
/* ====================================================================== */

int SM_SendEvent(Event_t event)
{
    if (sm_queue == NULL) return -1;
    return (xQueueSend(sm_queue, &event, 0) == pdPASS) ? 0 : -1;
}

int SM_SendEventFromISR(Event_t event)
{
    if (sm_queue == NULL) return -1;

    BaseType_t taskWoken = pdFALSE;
    int ret = (xQueueSendFromISR(sm_queue, &event, &taskWoken) == pdPASS) ? 0 : -1;

    if (taskWoken == pdTRUE)
        taskYIELD();

    return ret;
}
