#include "statemachine.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#include "ChassisControl.h"
#include "Gimbal.h"
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

/* 前向声明 */
static void SM_Task(void *argument);

/* ====================================================================== */
/*  动作函数                                                              */
/*                                                                        */
/*  移动动作: Chassis_SendMoveCmd(x, y, EVENT_ARRIVED)                   */
/*           到位后 Chassis_Task 自动发 EVENT_ARRIVED                     */
/*                                                                        */
/*  操作动作: SM_SendEvent(EVENT_ACTION_DONE)                            */
/*           表示机械臂操作 (扫码/抓取/放置) 完成                         */
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

static void Action_MoveToRaw1(void)
{
    Chassis_SendMoveCmd(800, 0, EVENT_ARRIVED);
}

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

static void Action_FetchRaw1(void)
{
    /* 按顺序取料: seq[3] = {2, 3, 1} */
    /* 每次: 0° CW→目标 → 取料 → CCW←0° */
    for (int i = 0; i < 3; i++)
    {
        float angle = car_angle(seq[i]);
        Gimbal_SetAngle(angle);                     /* CW 出去 */
        while (fabsf(Gimbal_GetAngle() - angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);           /* 伸长 */
        // osDelay(100);                 /* 等伸出到位 */
        // Gimbal_Gripper(30000);        /* 夹紧 */
        // osDelay(200);                 /* 等夹紧 */
        // Gimbal_Extend(-500);          /* 缩回 */

        Gimbal_SetAngle(0.0f);                      /* CCW 回零 */
        while (fabsf(Gimbal_GetAngle()) > 1.0f)
            osDelay(10);
    }

    SM_SendEvent(EVENT_ACTION_DONE);
}

static void Action_MoveToRough1(void)
{
    Chassis_SendMoveCmd(-400, 0, EVENT_NONE);        /* 左移到主干道 */
    Chassis_SendRotateCmd(90.0f, EVENT_NONE);         /* 逆时针转180° (陀螺仪闭环) */
    Chassis_SendMoveCmd(0, 1800, EVENT_ARRIVED);      /* 直行到粗加工区 */
}

static void Action_PlaceRough1(void)
{
    /* 按顺序: 从车身取料 → 放到粗加工区 → 取回 */
    for (int i = 0; i < 3; i++)
    {
        /* 从车身对应角度取料 */
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);           /* 伸长 */
        // osDelay(100);
        // Gimbal_Gripper(30000);        /* 夹取 */
        // osDelay(200);
        // Gimbal_Extend(-500);          /* 缩回 */

        /* 转到工位对应角度放置，放完了就从车体取下一个 */
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);
    }

    for (int i = 0; i < 3; i++)
    {

        /* 转到工位对应角度取料 */
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);
        

        // Gimbal_Extend(500);           /* 伸长 */
        // osDelay(100);
        // Gimbal_Gripper(30000);        /* 夹取 */
        // osDelay(200);
        // Gimbal_Extend(-500);          /* 缩回 */

        /* 去车身对应角度放料 */
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);
    }

    //三个都放完后取回
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
    /* 按顺序: 从车身取料 → 放到暂存区 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        /* 从车身对应角度取料 */
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);           /* 伸长 */
        // osDelay(100);
        // Gimbal_Gripper(30000);        /* 夹取 */
        // osDelay(200);
        // Gimbal_Extend(-500);          /* 缩回 */
        // Gimbal_SetAngle(0.0f);

        /* 转到暂存区对应角度放置 */
        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);           /* 伸长到放置位 */
        // osDelay(100);
        // Gimbal_Gripper(0);            /* 松开 */
        // osDelay(200);
        // Gimbal_Extend(-500);          /* 缩回 */
        // Gimbal_SetAngle(0.0f);
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
    /* 同第一批: 按顺序从原料区取料到车身 */
    for (int i = 0; i < 3; i++)
    {
        float angle = car_angle(seq[i]);
        Gimbal_SetAngle(angle);
        while (fabsf(Gimbal_GetAngle() - angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);
        // osDelay(100);
        // Gimbal_Gripper(30000);
        // osDelay(200);
        // Gimbal_Extend(-500);

        Gimbal_SetAngle(0.0f);
        while (fabsf(Gimbal_GetAngle()) > 1.0f)
            osDelay(10);
    }
    SM_SendEvent(EVENT_ACTION_DONE);
}

static void Action_MoveToRough2(void)
{
    Chassis_SendMoveCmd(-400, 0, EVENT_NONE);
    Chassis_SendMoveCmd(0, -1800, EVENT_ARRIVED);
}

static void Action_PlaceRough2(void)
{
    /* 同第一批: 从车身取 → 放到粗加工区 → 取回 */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);
        // osDelay(100);
        // Gimbal_Gripper(30000);
        // osDelay(200);
        // Gimbal_Extend(-500);
        // Gimbal_SetAngle(0.0f);

        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);
        // osDelay(100);
        // Gimbal_Gripper(0);
        // osDelay(200);
        // Gimbal_Extend(-500);
        // Gimbal_SetAngle(0.0f);
    }

    /* TODO: 取回 */
    Gimbal_SetAngle(180.0f);
    while (fabsf(Gimbal_GetAngle() - 180.0f) > 1.0f)
        osDelay(10);
    // Gimbal_Extend(500);
    // osDelay(100);
    // Gimbal_Gripper(30000);
    // osDelay(200);
    // Gimbal_Extend(-500);
    Gimbal_SetAngle(0.0f);
    while (fabsf(Gimbal_GetAngle()) > 1.0f)
        osDelay(10);

    SM_SendEvent(EVENT_ACTION_DONE);
}

static void Action_MoveToTemp2(void)
{
    Chassis_SendMoveCmd(1000, 0, EVENT_NONE);
    Chassis_SendMoveCmd(0, -1000, EVENT_ARRIVED);
}

static void Action_StackTemp2(void)
{
    /* 同第一批暂存: 从车身取 → 放到暂存区码垛 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[i]);
        Gimbal_SetAngle(c_angle);
        while (fabsf(Gimbal_GetAngle() - c_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);
        // osDelay(100);
        // Gimbal_Gripper(30000);
        // osDelay(200);
        // Gimbal_Extend(-500);
        // Gimbal_SetAngle(0.0f);

        float m_angle = map_angle(seq[i]);
        Gimbal_SetAngle(m_angle);
        while (fabsf(Gimbal_GetAngle() - m_angle) > 1.0f)
            osDelay(10);

        // Gimbal_Extend(500);
        // osDelay(100);
        // Gimbal_Gripper(0);
        // osDelay(200);
        // Gimbal_Extend(-500);
        // Gimbal_SetAngle(0.0f);
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
/*                                                                        */
/*  每个状态对应一个 EVENT_ARRIVED (移动) 或 EVENT_ACTION_DONE (操作)     */
/*  引擎根据当前状态查表, 无需内部变量即可线性流转                         */
/* ====================================================================== */

const StateMachineTable_t SM_DefaultTable[] = {
    /* 当前状态            事件                动作                   下一状态 */
    {STATE_IDLE,            EVENT_START,        Action_Start,        STATE_MOVE_TO_QR},
    {STATE_MOVE_TO_QR,      EVENT_ARRIVED,      Action_ParseQR,      STATE_READ_QR},
    {STATE_READ_QR,         EVENT_ACTION_DONE,  Action_MoveToRaw1,   STATE_MOVE_TO_RAW_1},

    {STATE_MOVE_TO_RAW_1,   EVENT_ARRIVED,      Action_FetchRaw1,    STATE_FETCH_RAW_1},
    {STATE_FETCH_RAW_1,     EVENT_ACTION_DONE,  Action_MoveToRough1, STATE_MOVE_TO_ROUGH_1},
    {STATE_MOVE_TO_ROUGH_1, EVENT_ARRIVED,      Action_PlaceRough1,  STATE_PLACE_ROUGH_1},
    {STATE_PLACE_ROUGH_1,   EVENT_ACTION_DONE,  Action_MoveToTemp1,  STATE_MOVE_TO_TEMP_1},
    {STATE_MOVE_TO_TEMP_1,  EVENT_ARRIVED,      Action_PlaceTemp1,   STATE_PLACE_TEMP_1},
    {STATE_PLACE_TEMP_1,    EVENT_ACTION_DONE,  Action_MoveToRaw2,   STATE_MOVE_TO_RAW_2},

    {STATE_MOVE_TO_RAW_2,   EVENT_ARRIVED,      Action_FetchRaw2,    STATE_FETCH_RAW_2},
    {STATE_FETCH_RAW_2,     EVENT_ACTION_DONE,  Action_MoveToRough2, STATE_MOVE_TO_ROUGH_2},
    {STATE_MOVE_TO_ROUGH_2, EVENT_ARRIVED,      Action_PlaceRough2,  STATE_PLACE_ROUGH_2},
    {STATE_PLACE_ROUGH_2,   EVENT_ACTION_DONE,  Action_MoveToTemp2,  STATE_MOVE_TO_TEMP_2},
    {STATE_MOVE_TO_TEMP_2,  EVENT_ARRIVED,      Action_StackTemp2,   STATE_STACK_TEMP_2},
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

    const osThreadAttr_t attr = {
        .name       = "sm_event",
        .stack_size = 256 * 2,
        .priority   = osPriorityNormal,
    };
    osThreadNew(SM_Task, NULL, &attr);

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
