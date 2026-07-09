#include "statemachine.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "ChassisControl.h"
#include "Gimbal.h"
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

/* ====================================================================== */
/*  颜色校验                                                              */
/* ====================================================================== */

/* 期望颜色表: 按位置索引, [1]=蓝, [2]=红, [3]=绿 */
const uint8_t expected_color[4] = {0, 2, 1, 3};

/* 共享变量: 由 Protocol_Dispatch 写入, 状态机轮询 */
volatile ColorConfirm_t g_color_result = {0};
volatile bool           g_color_pending = false;

/* ====================================================================== */
/*  伸长距离                                                              */
/* ====================================================================== */

#define PICKUP_EXTEND  500
#define PLACE_EXTEND   300
#define EXTEND_DIFF    (PICKUP_EXTEND - PLACE_EXTEND)  /* 200 */

/* 车身位伸出距离 (3个位置相同) */
#define CAR_EXTEND      300
/* 地图工位伸出距离: 2最短, 1/3相同 */
#define MAP_EXTEND_1_3  400
#define MAP_EXTEND_2    200

/* ====================================================================== */
/*  辅助函数                                                              */
/* ====================================================================== */

static int32_t map_extend(uint8_t pos) {
    return (pos == 2) ? MAP_EXTEND_2 : MAP_EXTEND_1_3;
}

static void Gripper_Close(void) { Gimbal_Gripper(30000); }
static void Gripper_Open(void)  { Gimbal_Gripper(0); }

/* 前向声明 */
static void SM_Task(void *argument);

/* ====================================================================== */
/*  动作函数                                                              */
/* ====================================================================== */

static void Action_Nop(void)
{
}

/* ---- 初始化与准备 ---- */

static void Action_Start(void)
{
    Chassis_SendMoveCmd(50, -50, EVENT_NONE);      /* 中间点 */
    Gimbal_Extend(500);
    Chassis_SendMoveCmd(300, 0, EVENT_ARRIVED);    /* 到 QR 位 */
}

static void Action_ParseQR(void)
{
    /* TODO: 实际扫码解析 */
    SM_SendEvent(EVENT_ACTION_DONE);
}

/* ---- 第一批次 ---- */

/* 车体上的 3 个角度 (1=180°, 2=220°, 3=260°) */
static float car_angle(uint8_t pos)
{
    float deg;
    if (pos == 1)      deg = 180.0f;
    else if (pos == 2) deg = 220.0f;
    else               deg = 260.0f;
    if (deg > 180.0f) deg -= 360.0f;
    return deg;
}

/* 地图上对应工位的 3 个角度 (TODO: 填入实际值) */
static float map_angle(uint8_t pos)
{
    float deg;
    if (pos == 1)      deg = 30.0f;    /* TODO */
    else if (pos == 2) deg = 0.0f;    /* TODO */
    else               deg = -30.0f;    /* TODO */
    return deg;
}

static void Action_MoveToRaw1(void)
{
    Chassis_SendMoveCmd(800, 0, EVENT_ARRIVED);
}

static void Action_FetchRaw1(void)
{
    GimbalCmd_t cmd = {GIMBAL_CMD_FETCH_RAW, EVENT_ACTION_DONE};
    Gimbal_SendCmd(&cmd);
}

static void Action_MoveToRough1(void)
{
    Chassis_SendMoveCmd(-400, 0, EVENT_NONE);        /* 左移到主干道 */
    Chassis_SendRotateCmd(90.0f, EVENT_NONE);         /* 逆时针转180° (陀螺仪闭环) */
    Chassis_SendMoveCmd(0, 1800, EVENT_ARRIVED);      /* 直行到粗加工区 */
}

static void Action_PlaceRough1(void)
{
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);
    }

    Gimbal_SetAngle(0);
    SM_SendEvent(EVENT_ACTION_DONE);
}

static void Action_MoveToTemp1(void)
{
    Chassis_SendMoveCmd(1000, 0, EVENT_NONE);
    Chassis_SendMoveCmd(0, 1000, EVENT_ARRIVED);
}

static void Action_PlaceTemp1(void)
{
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }
    Gimbal_SetAngle(0.0f);
    SM_SendEvent(EVENT_ACTION_DONE);
}

/* ---- 第二批次 ---- */

static void Action_MoveToRaw2(void)
{
    Chassis_SendMoveCmd(0, 500, EVENT_NONE);
    Chassis_SendMoveCmd(-500, 0, EVENT_ARRIVED);
}

static void Action_FetchRaw2(void)
{
    /* 同第一批: 伸出→等颜色→夹取→转放料位→释放→归零 */
    Gimbal_Extend(PICKUP_EXTEND);
    osDelay(100);

    for (int i = 0; i < 3; i++)
    {
        g_color_pending = true;
        while (g_color_pending)
            osDelay(10);

        if (g_color_result.color == expected_color[seq[i]])
        {
            Gimbal_Lift(-200);
            osDelay(100);
            Gripper_Close();
            osDelay(200);
            Gimbal_Lift(200);
            osDelay(100);
        }

        Gimbal_Extend(-EXTEND_DIFF);
        osDelay(100);
        float angle = car_angle(seq[i]);
        Gimbal_SetAngle(angle);
        while (fabsf(Gimbal_GetAngle() - angle) > 1.0f)
            osDelay(10);

        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
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
}

static void Action_MoveToRough2(void)
{
    Chassis_SendMoveCmd(-400, 0, EVENT_NONE);
    Chassis_SendMoveCmd(0, -1800, EVENT_ARRIVED);
}

static void Action_PlaceRough2(void)
{
    /* 从车身取料 → 放到粗加工区 */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(200);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);

        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
        osDelay(100);
        Gimbal_Extend(-CAR_EXTEND);
        osDelay(100);
    }

    Gimbal_SetAngle(0.0f);
    SM_SendEvent(EVENT_ACTION_DONE);
}

static void Action_MoveToTemp2(void)
{
    Chassis_SendMoveCmd(1000, 0, EVENT_NONE);
    Chassis_SendMoveCmd(0, -1000, EVENT_ARRIVED);
}

static void Action_StackTemp2(void)
{
    /* 从车身取料 → 放到暂存区码垛 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        Gimbal_Extend(CAR_EXTEND);
        osDelay(100);
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(200);
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
        Gimbal_Lift(-200);
        osDelay(100);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(200);
        osDelay(100);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }
    Gimbal_SetAngle(0.0f);
    SM_SendEvent(EVENT_ACTION_DONE);
}

/* ---- 收尾 ---- */

static void Action_ReturnStart(void)
{
    Chassis_SendMoveCmd(0, 800, EVENT_NONE);
    Chassis_SendMoveCmd(-1600, 0, EVENT_ARRIVED);
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
