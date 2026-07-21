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

volatile State_t      sm_currentState;
static const StateMachineTable_t *sm_table      = NULL;
static uint32_t              sm_tableSize = 0;

/* ====================================================================== */
/*  事件队列                                                              */
/* ====================================================================== */

#define SM_QUEUE_LENGTH     16

static QueueHandle_t sm_queue = NULL;

uint8_t seq[2][3] = {
    {2u, 3u, 1u},
    {2u, 3u, 1u},
};
static volatile bool sm_material_sequence_ready = false;
static volatile bool sm_material_sequence_event_sent = false;

/* 最近一次识别到的颜色，仅保留用于状态观察。 */
volatile uint8_t g_current_color = 0;

#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT

static int32_t map_extend(uint8_t pos)
{
    if (pos == 1) return CONFIG_GIMBAL_MAP_EXTEND_POS_1_PULSES;
    if (pos == 2) return CONFIG_GIMBAL_MAP_EXTEND_POS_2_PULSES;
    return CONFIG_GIMBAL_MAP_EXTEND_POS_3_PULSES;
}

static void Gripper_Close(void) { Gimbal_Gripper(CONFIG_GRIPPER_CLOSE_PULSE_US); }
static void Gripper_Open(void)  { Gimbal_Gripper(CONFIG_GRIPPER_OPEN_PULSE_US); }

static void wait_material_moved_and_settled(uint32_t *settled_sequence)
{
    uint32_t current_sequence;

    do {
        osDelay(10);
        current_sequence = Protocol_GetMaterialSettledSequence();
    } while (current_sequence == *settled_sequence);

    *settled_sequence = current_sequence;
}

static uint8_t wait_current_material_color(void)
{
    uint8_t color;

    do {
        color = g_vision_feedback.color;
        if (color < PROTOCOL_COLOR_RED || color > PROTOCOL_COLOR_BLUE) {
            osDelay(10);
        }
    } while (color < PROTOCOL_COLOR_RED || color > PROTOCOL_COLOR_BLUE);

    return color;
}

#endif /* USE_GIMBAL && CONFIG_ENABLE_MATERIAL_PLACEMENT */

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

static void SM_ChassisRotate(float degrees, Event_t completion_event)
{
#if CONFIG_USE_CHASSIS
    Chassis_SendRotateCmd(degrees, completion_event);
#else
    (void)degrees;
    if (completion_event != EVENT_NONE)
        SM_SendEvent(completion_event);
#endif
}

/* ====================================================================== */
/*  动作函数                                                              */
/* ====================================================================== */

static void Action_Nop(void)
{
}

static void Action_EnterAdjust(void)
{
#if CONFIG_USE_CHASSIS
    if (sm_currentState == STATE_MOVE_TO_RAW_1 ||
        sm_currentState == STATE_MOVE_TO_RAW_2) {
        (void)Chassis_BeginVisionAdjustAtHeading(
            CONFIG_CHASSIS_RAW_HEADING_DEG);
    } else if (sm_currentState == STATE_MOVE_TO_ROUGH_1 ||
        sm_currentState == STATE_MOVE_TO_ROUGH_2) {
        (void)Chassis_BeginVisionAdjustAtHeading(
            CONFIG_CHASSIS_ROUGH_HEADING_DEG);
    } else if (sm_currentState == STATE_MOVE_TO_TEMP_1 ||
               sm_currentState == STATE_MOVE_TO_TEMP_2) {
        (void)Chassis_BeginVisionAdjustAtHeading(
            CONFIG_CHASSIS_TEMP_HEADING_DEG);
    } else {
        (void)Chassis_BeginVisionAdjust();
    }
#endif
#if CONFIG_SKIP_ADJUST
#if CONFIG_USE_CHASSIS
    Chassis_EndVisionAdjust();
#endif
    SM_SendEvent(EVENT_ADJUST_DONE);
#endif
}

/* ---- 初始化与准备 ---- */

static void Action_Start(void)
{
    /* 临时测试：原有平移动作。 */
    SM_ChassisMove(0, 410, EVENT_NONE);        /* 先沿 Y 轴到中间点 */
    // SM_ChassisMove(500, 0, EVENT_NONE);        /* 再沿 X 轴到中间点 */
    SM_ChassisMove(870, 000, EVENT_ARRIVED);     /* 到 QR 位 */

    /* 航向角闭环测试：相对陀螺仪软件零点转到 +90°。 */
    // Chassis_SendRotateCmd(90.0f, EVENT_NONE);
}

static void Action_ParseQR(void)
{
#if CONFIG_SKIP_QR
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    bool send_ready_event = false;

    taskENTER_CRITICAL();
    if (sm_material_sequence_ready &&
        !sm_material_sequence_event_sent) {
        sm_material_sequence_event_sent = true;
        send_ready_event = true;
    }
    taskEXIT_CRITICAL();

    if (send_ready_event && SM_SendEvent(EVENT_ACTION_DONE) != 0) {
        taskENTER_CRITICAL();
        sm_material_sequence_event_sent = false;
        taskEXIT_CRITICAL();
    }
#endif
}

/* ---- 第一批次 ---- */

#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT

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

#endif /* USE_GIMBAL && CONFIG_ENABLE_MATERIAL_PLACEMENT */

static void Action_MoveToRaw1(void)
{
    taskENTER_CRITICAL();
    sm_material_sequence_ready = false;
    sm_material_sequence_event_sent = false;
    taskEXIT_CRITICAL();

    #if defined(USE_GIMBAL)
        /* 前往原料区途中将云台从内收位折到地图中间位。 */
        Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
    #endif
    SM_ChassisRotate(CONFIG_CHASSIS_RAW_HEADING_DEG, EVENT_NONE);
    SM_ChassisMove(1050, 0, EVENT_ARRIVED);
}

static void FetchRawToCar(void)
{
#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT
    uint32_t settled_sequence = Protocol_GetMaterialSettledSequence();

    Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
    osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);
    Gripper_Open();
    // osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);

    /* Raw/地图位置 2 为伸缩原点，每次进入 Raw 只确认一次动后静止。 */
    wait_material_moved_and_settled(&settled_sequence);

    for (int i = 0; i < 3; i++)
    {
        uint8_t material_color = wait_current_material_color();

        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS);
        Gripper_Close();
        osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_TURNTABLE_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_TURNTABLE_WAIT_MS);

        /* 车体比 Raw 原点更靠内，负位移表示缩短。 */
        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(1000);
        Gimbal_SetAngle(car_angle(material_color));
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Open();
        osDelay(CONFIG_GIMBAL_GRIPPER_WAIT_MS);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);

        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(1000);
        Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);
    }

    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_FetchRaw1(void)
{
    FetchRawToCar();
}

static void Action_MoveToRough1(void)
{
    SM_ChassisMove(-550, 0, EVENT_NONE);        /* 左移到主干道 */
    SM_ChassisRotate(CONFIG_CHASSIS_ROUGH_HEADING_DEG, EVENT_NONE); /* 陀螺仪闭环 */
    SM_ChassisMove(0, -2050, EVENT_ARRIVED);      /* 直行到粗加工区 */
}

static void Action_PlaceRough1(void)
{
#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT
    /* 从车身取料 → 放到粗加工区 */
    for (int i = 0; i < 3; i++)
    {

        /* 从车身对应角度取料 */
        float c_angle = car_angle(seq[0][i]);
        Gimbal_SetAngle(c_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);

        /* 转到工位对应角度放置 */
        float m_angle = map_angle(seq[0][i]);
        Gimbal_SetAngle(m_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        int32_t m_ext = map_extend(seq[0][i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }

    /* 取回: 从工位取 → 放回车身 */
    for (int i = 0; i < 3; i++)
    {
        /* 转到工位对应角度取料 */
        float m_angle = map_angle(seq[0][i]);
        Gimbal_SetAngle(m_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        int32_t m_ext = map_extend(seq[0][i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gimbal_Extend(-m_ext);
        osDelay(100);

        /* 去车身对应角度放料 */
        float c_angle = car_angle(seq[0][i]);
        Gimbal_SetAngle(c_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
    }

    #ifdef USE_GIMBAL
    /* 前往原料区途中将云台从内收位折到地图中间位。 */
    Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
    #endif
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_MoveToTemp1(void)
{
    SM_ChassisMove(-980, 0, EVENT_NONE);
    SM_ChassisRotate(CONFIG_CHASSIS_TEMP_HEADING_DEG, EVENT_NONE);
    SM_ChassisMove(-1100, 0, EVENT_ARRIVED);
}

static void Action_PlaceTemp1(void)
{
#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT
    /* 从车身取料 → 放到暂存区 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        /* 从车身对应角度取料 */
        float c_angle = car_angle(seq[0][i]);
        Gimbal_SetAngle(c_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);

        /* 转到暂存区对应角度放置 */
        float m_angle = map_angle(seq[0][i]);
        Gimbal_SetAngle(m_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        int32_t m_ext = map_extend(seq[0][i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }
#ifdef USE_GIMBAL
    /* 前往原料区途中将云台从内收位折到地图中间位。 */
    Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
#endif
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

/* ---- 第二批次 ---- */

static void Action_MoveToRaw2(void)
{
    SM_ChassisMove(-1050, 0, EVENT_NONE);
    SM_ChassisRotate(CONFIG_CHASSIS_RAW_HEADING_DEG, EVENT_NONE);
    SM_ChassisMove(-650, 0, EVENT_ARRIVED);
}

static void Action_FetchRaw2(void)
{
    FetchRawToCar();
}

static void Action_MoveToRough2(void)
{
    SM_ChassisMove(-550, 0, EVENT_NONE);        /* 左移到主干道 */
    SM_ChassisRotate(CONFIG_CHASSIS_ROUGH_HEADING_DEG, EVENT_NONE); /* 陀螺仪闭环 */
    SM_ChassisMove(0, -2050, EVENT_ARRIVED);      /* 直行到粗加工区 */
}

static void Action_PlaceRough2(void)
{
#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT
    /* 从车身取料 → 放到粗加工区 */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[1][i]);
        Gimbal_SetAngle(c_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);

        float m_angle = map_angle(seq[1][i]);
        Gimbal_SetAngle(m_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        int32_t m_ext = map_extend(seq[1][i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gimbal_Extend(-m_ext);
        osDelay(100);
    }

    /* 取回 */
    for (int i = 0; i < 3; i++)
    {
        float m_angle = map_angle(seq[1][i]);
        Gimbal_SetAngle(m_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        int32_t m_ext = map_extend(seq[1][i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_GROUND_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_GROUND_WAIT_MS);
        Gimbal_Extend(-m_ext);
        osDelay(100);

        float c_angle = car_angle(seq[1][i]);
        Gimbal_SetAngle(c_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
    }

#ifdef USE_GIMBAL
    /* 前往原料区途中将云台从内收位折到地图中间位。 */
    Gimbal_SetAngle(CONFIG_MAP_MATERIAL_POS_2_DEG);
#endif
    SM_SendEvent(EVENT_ACTION_DONE);
#else
    SM_SendEvent(EVENT_ACTION_DONE);
#endif
}

static void Action_MoveToTemp2(void)
{
    SM_ChassisMove(-980, 0, EVENT_NONE);
    SM_ChassisRotate(CONFIG_CHASSIS_TEMP_HEADING_DEG, EVENT_NONE);
    SM_ChassisMove(-1100, 0, EVENT_ARRIVED);
}

static void Action_StackTemp2(void)
{
#if defined(USE_GIMBAL) && CONFIG_ENABLE_MATERIAL_PLACEMENT
    /* 从车身取料 → 放到暂存区码垛 (不回取) */
    for (int i = 0; i < 3; i++)
    {
        float c_angle = car_angle(seq[1][i]);
        Gimbal_SetAngle(c_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        Gimbal_Extend(-CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gripper_Close();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_CAR_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_CAR_WAIT_MS);
        Gimbal_Extend(CONFIG_GIMBAL_CAR_RETRACT_PULSES);
        osDelay(100);

        float m_angle = map_angle(seq[1][i]);
        Gimbal_SetAngle(m_angle);
        osDelay(CONFIG_GIMBAL_ROTATE_WAIT_MS);

        int32_t m_ext = map_extend(seq[1][i]);
        Gimbal_Extend(m_ext);
        osDelay(100);
        Gimbal_Lift(-CONFIG_GIMBAL_LIFT_STACK_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_STACK_WAIT_MS);
        Gripper_Open();
        osDelay(200);
        Gimbal_Lift(CONFIG_GIMBAL_LIFT_STACK_PULSES);
        osDelay(CONFIG_GIMBAL_LIFT_STACK_WAIT_MS);
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
    SM_ChassisMove(50, 0, EVENT_ARRIVED);
    SM_ChassisMove(0, 2300, EVENT_NONE);
    SM_ChassisMove(-1250, 0, EVENT_ARRIVED);
}

/* ====================================================================== */
/*  默认状态转换表 (线性查表法)                                           */
/* ====================================================================== */

const StateMachineTable_t SM_DefaultTable[] = {
    /* 当前状态            事件                动作                   下一状态 */
    {STATE_IDLE,            EVENT_START,        Action_Start,        STATE_MOVE_TO_QR},
    {STATE_MOVE_TO_QR,      EVENT_ARRIVED,      Action_ParseQR,      STATE_READ_QR},
    {STATE_READ_QR,         EVENT_ACTION_DONE,  Action_MoveToRaw1,   STATE_MOVE_TO_RAW_1},

    {STATE_MOVE_TO_RAW_1,   EVENT_ARRIVED,      Action_EnterAdjust,  STATE_ADJUST_RAW_1},
    {STATE_ADJUST_RAW_1,    EVENT_ADJUST_DONE,   Action_FetchRaw1,    STATE_FETCH_RAW_1},
    {STATE_FETCH_RAW_1,     EVENT_ACTION_DONE,  Action_MoveToRough1, STATE_MOVE_TO_ROUGH_1},
    {STATE_MOVE_TO_ROUGH_1, EVENT_ARRIVED,      Action_EnterAdjust, STATE_ADJUST_ROUGH_1},
    {STATE_ADJUST_ROUGH_1,  EVENT_ADJUST_DONE,   Action_PlaceRough1,  STATE_PLACE_ROUGH_1},
    {STATE_PLACE_ROUGH_1,   EVENT_ACTION_DONE,  Action_MoveToTemp1,  STATE_MOVE_TO_TEMP_1},
    {STATE_MOVE_TO_TEMP_1,  EVENT_ARRIVED,      Action_EnterAdjust,  STATE_ADJUST_TEMP_1},
    {STATE_ADJUST_TEMP_1,   EVENT_ADJUST_DONE,   Action_PlaceTemp1,   STATE_PLACE_TEMP_1},
    {STATE_PLACE_TEMP_1,    EVENT_ACTION_DONE,  Action_MoveToRaw2,   STATE_MOVE_TO_RAW_2},

    {STATE_MOVE_TO_RAW_2,   EVENT_ARRIVED,      Action_EnterAdjust,   STATE_ADJUST_RAW_2},
    {STATE_ADJUST_RAW_2,    EVENT_ADJUST_DONE,   Action_FetchRaw2,    STATE_FETCH_RAW_2},
    {STATE_FETCH_RAW_2,     EVENT_ACTION_DONE,  Action_MoveToRough2, STATE_MOVE_TO_ROUGH_2},
    {STATE_MOVE_TO_ROUGH_2, EVENT_ARRIVED,      Action_EnterAdjust, STATE_ADJUST_ROUGH_2},
    {STATE_ADJUST_ROUGH_2,  EVENT_ADJUST_DONE,   Action_PlaceRough2,  STATE_PLACE_ROUGH_2},
    {STATE_PLACE_ROUGH_2,   EVENT_ACTION_DONE,  Action_MoveToTemp2,  STATE_MOVE_TO_TEMP_2},
    {STATE_MOVE_TO_TEMP_2,  EVENT_ARRIVED,      Action_EnterAdjust,  STATE_ADJUST_TEMP_2},
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
    sm_material_sequence_ready = false;
    sm_material_sequence_event_sent = false;

    sm_queue = xQueueCreate(SM_QUEUE_LENGTH, sizeof(Event_t));

    xTaskCreate(SM_Task, "sm_event", 128, NULL, osPriorityNormal, NULL);

    /* 等待 PA15 按钮中断发送 EVENT_START。 */
}

/* ====================================================================== */
/*  SM_GetState                                                            */
/* ====================================================================== */

State_t SM_GetState(void)
{
    return sm_currentState;
}

static bool valid_material_sequence(const uint8_t order[3])
{
    uint8_t seen = 0u;

    if (order == NULL) {
        return false;
    }
    for (uint8_t i = 0u; i < 3u; ++i) {
        if (order[i] < 1u || order[i] > 3u) {
            return false;
        }
        seen |= (uint8_t)(1u << order[i]);
    }
    if (seen != 0x0Eu) {
        return false;
    }

    return true;
}

bool SM_SetMaterialSequences(const uint8_t first[3],
                             const uint8_t second[3])
{
    bool send_ready_event = false;
    bool accepted = false;

    if (!valid_material_sequence(first) ||
        !valid_material_sequence(second)) {
        return false;
    }

    taskENTER_CRITICAL();
    if (sm_currentState == STATE_IDLE ||
        sm_currentState == STATE_MOVE_TO_QR ||
        sm_currentState == STATE_READ_QR) {
        for (uint8_t i = 0u; i < 3u; ++i) {
            seq[0][i] = first[i];
            seq[1][i] = second[i];
        }
        sm_material_sequence_ready = true;
        accepted = true;

        if (sm_currentState == STATE_READ_QR &&
            !sm_material_sequence_event_sent) {
            sm_material_sequence_event_sent = true;
            send_ready_event = true;
        }
    }
    taskEXIT_CRITICAL();

    if (send_ready_event && SM_SendEvent(EVENT_ACTION_DONE) != 0) {
        taskENTER_CRITICAL();
        sm_material_sequence_event_sent = false;
        taskEXIT_CRITICAL();
        return false;
    }

    return accepted;
}

void SM_SetCurrentColor(uint8_t color)
{
    if (color < 1u || color > 3u) {
        return;
    }
    g_current_color = color;
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
    taskENTER_CRITICAL();
    sm_currentState = initialState;
    sm_material_sequence_ready = false;
    sm_material_sequence_event_sent = false;
    taskEXIT_CRITICAL();
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

    portYIELD_FROM_ISR(taskWoken);

    return ret;
}
