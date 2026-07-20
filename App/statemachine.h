#ifndef __STATEMACHINE_H__
#define __STATEMACHINE_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* ====================================================================== */
/*  1. 状态枚举 (线性展开, 无内部变量区分批次)                             */
/* ====================================================================== */

typedef enum {
    /* 初始化与准备 */
    STATE_IDLE,             /* 启停区待机 */
    STATE_MOVE_TO_QR,       /* 前往二维码板 */
    STATE_READ_QR,          /* 扫码并解析任务 */

    /* 第一批次 */
    STATE_MOVE_TO_RAW_1,    /* 前往原料区 (起点: 二维码板) */
    STATE_ADJUST_RAW_1,     /* 原料区视觉微调 */
    STATE_FETCH_RAW_1,      /* 抓取第一批物料 */
    STATE_MOVE_TO_ROUGH_1,  /* 前往粗加工区 */
    STATE_ADJUST_ROUGH_1,   /* 粗加工区视觉微调 */
    STATE_PLACE_ROUGH_1,    /* 放置第一批物料 */
    STATE_MOVE_TO_TEMP_1,   /* 前往暂存区 */
    STATE_ADJUST_TEMP_1,    /* 暂存区视觉微调 */
    STATE_PLACE_TEMP_1,     /* 放置第一批物料 (平放) */

    /* 第二批次 */
    STATE_MOVE_TO_RAW_2,    /* 前往原料区 (起点: 暂存区) */
    STATE_ADJUST_RAW_2,     /* 原料区视觉微调 */
    STATE_FETCH_RAW_2,      /* 抓取第二批物料 */
    STATE_MOVE_TO_ROUGH_2,  /* 前往粗加工区 */
    STATE_ADJUST_ROUGH_2,   /* 粗加工区视觉微调 */
    STATE_PLACE_ROUGH_2,    /* 放置第二批物料 */
    STATE_MOVE_TO_TEMP_2,   /* 前往暂存区 */
    STATE_ADJUST_TEMP_2,    /* 暂存区视觉微调 */
    STATE_STACK_TEMP_2,     /* 放置第二批物料 (码垛) */

    /* 收尾 */
    STATE_RETURN_START,     /* 返回启停区 */
} State_t;

/* ====================================================================== */
/*  2. 事件枚举 (统一抽象)                                                */
/* ====================================================================== */

typedef enum {
    EVENT_NONE = 0,         /* 无事件 (哨兵) */
    EVENT_START,            /* 启动触发 */
    EVENT_ARRIVED,          /* 底盘到位 (所有移动完成) */
    EVENT_ACTION_DONE,      /* 机械臂操作完成 (扫码/抓取/放置/码垛) */
    EVENT_ADJUST_DONE,      /* 上位机视觉微调完成 */
} Event_t;

/* ====================================================================== */
/*  3. 动作函数指针类型                                                    */
/* ====================================================================== */

typedef void (*ActionFunc_t)(void);

/* ====================================================================== */
/*  4. 状态转换表条目结构                                                  */
/* ====================================================================== */

typedef struct {
    State_t      currentState;   /* 当前状态           */
    Event_t      event;          /* 触发事件           */
    ActionFunc_t action;         /* 执行的动作 (可为NULL) */
    State_t      nextState;      /* 下一状态           */
} StateMachineTable_t;

/* ====================================================================== */
/*  5. 默认转换表 + 初始化                                                 */
/* ====================================================================== */

/** 默认状态转换表 (在 statemachine.c 中定义) */
extern const StateMachineTable_t SM_DefaultTable[];

/** 默认转换表条目数 */
extern const uint32_t SM_DefaultTableSize;

/* ====================================================================== */
/*  6. API                                                                 */
/* ====================================================================== */

/** 初始化状态机 (使用默认转换表, 创建队列, 启动任务) */
void SM_Init(void);

/** 获取当前状态 */
State_t SM_GetState(void);

/**
 * 一次保存两轮各三个物料的颜色顺序。
 * 仅在 IDLE / MOVE_TO_QR / READ_QR 接受，成功返回 true。
 */
bool SM_SetMaterialSequences(const uint8_t first[3],
                             const uint8_t second[3]);

/** 保存上位机最近识别到的颜色（1=红，2=绿，3=蓝）。 */
void SM_SetCurrentColor(uint8_t color);

/** 处理事件: 查表执行动作并转换状态 */
void SM_ProcessEvent(Event_t event);

/** 重置状态机到初始状态 */
void SM_Reset(State_t initialState);

/* ====================================================================== */
/*  7. 事件队列                                                            */
/* ====================================================================== */

/** 从任意任务发送事件到队列 (非阻塞) */
int SM_SendEvent(Event_t event);

/** 从中断发送事件到队列 (非阻塞) */
int SM_SendEventFromISR(Event_t event);

#ifdef __cplusplus
}
#endif

#endif /* __STATEMACHINE_H__ */
