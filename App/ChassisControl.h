#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include "bsp_zdt.h"
#include "FreeRTOS.h"
#include "task.h"
#include "statemachine.h"   /* Event_t */

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 队列移动指令 ---- */

typedef struct {
    int16_t x;                  /* X 位移 (mm) */
    int16_t y;                  /* Y 位移 (mm) */
    int16_t rotation;           /* 纯旋转时为 0.01°；平移+旋转时为 0.01 rad */
    Event_t completion_event;   /* 到位后发送的事件; EVENT_NONE = 不发 */
    bool run_vision_adjust;     /* 进入视觉纯 P 闭环 */
} ChassisMoveCmd_t;

typedef enum {
    CHASSIS_VISION_AXIS_IDLE = 0,
    CHASSIS_VISION_AXIS_X,
    CHASSIS_VISION_AXIS_Y,
} ChassisVisionAxis_t;

/** 向底盘队列下发移动指令 (非阻塞), 移动完成后自动发 completion_event */
int  Chassis_SendMoveCmd(int16_t x, int16_t y, Event_t completion_event);

/** 下发原地旋转指令 (非阻塞), 使用陀螺仪闭环 (角度制) */
int  Chassis_SendRotateCmd(float degrees, Event_t completion_event);

/** 进入视觉微调阶段并记录一次当前车头方向。 */
bool Chassis_BeginVisionAdjust(void);

/** 更新视觉闭环使用的最新归一化偏差，不创建移动队列项。 */
void Chassis_UpdateVisionAdjust(int8_t offset_x, int8_t offset_y);

/** 结束视觉微调，停止当前微调并清除尚未执行的微调指令。 */
void Chassis_EndVisionAdjust(void);

extern volatile bool g_chassis_adjust_heading_active;
extern volatile float g_chassis_adjust_heading_deg;
extern volatile ChassisVisionAxis_t g_chassis_adjust_axis;

/* ---- ---- */

void Chassis_Task(void *argument);

void Chassis_Stop(void);

void Chassis_Enable(bool en);

void Chassis_TaskInit(void);


#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_CONTROL_H__ */
