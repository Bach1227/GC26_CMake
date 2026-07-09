#ifndef __CHASSIS_CONTROL_H__
#define __CHASSIS_CONTROL_H__

#include <stdint.h>
#include <stdbool.h>
#include "protocol.h"
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
    int16_t rotation;           /* 旋转 (百分之一弧度, 仅平移+旋转时用) */
    Event_t completion_event;   /* 到位后发送的事件; EVENT_NONE = 不发 */
} ChassisMoveCmd_t;

/** 向底盘队列下发移动指令 (非阻塞), 移动完成后自动发 completion_event */
int  Chassis_SendMoveCmd(int16_t x, int16_t y, Event_t completion_event);

/** 下发原地旋转指令 (非阻塞), 使用陀螺仪闭环 (角度制) */
int  Chassis_SendRotateCmd(float degrees, Event_t completion_event);

/** 通知底盘当前移动已完成 (可从 ISR 调用) */
void Chassis_NotifyMoveComplete(void);

/* ---- ---- */

void Chassis_OnCarMove(const CarMove_t *cmd);

void Chassis_Task(void *argument);

void Chassis_Stop(void);

void Chassis_Enable(bool en);

void Chassis_TaskInit(void);


#ifdef __cplusplus
}
#endif

#endif /* __CHASSIS_CONTROL_H__ */
