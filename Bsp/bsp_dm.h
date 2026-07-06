#ifndef __DM_J4310_MIT_H
#define __DM_J4310_MIT_H

#include "bsp_can.h"   // 沿用你已有的 STD/FD 抽象层
#include "PID.h"         // 沿用你已有的 PID 结构体


/*-------------------- 配置宏 --------------------*/
#define DM_J4310_MIT_NODE_ID    0x01u   /* 默认节点 ID，可运行时修改 */
#define DM_J4310_CAN_BUS        CAN1    /* 改成实际使用的 CAN 外设 */

/* MIT 模式 5 个控制量物理范围（与说明书一致） */
#define P_MAX_RAD       3.14f     
#define V_MAX_RAD_S     30.0f   
#define T_MAX_NM        10.0f   
#define KP_MAX          500.0f
#define KD_MAX          5.0f

#define P_MAX_RAD_3519       12.5   
#define V_MAX_RAD_S_3519     200.0f   
#define T_MAX_NM_3519        10.0f   
#define KP_MAX_3519          500.0f
#define KD_MAX_3519          5.0f

#define DM_Kposition    1  /* 位置环增益 */
#define DM_Kspeed       1.2  /* 速度环增益 */

#define DM_Kposition_3519    0  /* 位置环增益 */
#define DM_Kspeed_3519       0.4  /* 速度环增益 */

typedef struct
{
    float  Pmax;
    float  Vmax;
    float  Tmax;
    float  Kpmax;
    float  Kdmax;
}DM_Motor_MaxConfig;


/*-------------------- 数据结构 --------------------*/
typedef struct {
    float   position_angle;
    float   velocity_rad_s;
    float   kp;
    float   kd;
    float   torque_Nm;
    DM_Motor_MaxConfig maxConfig;
} DM_Motor_MIT_Struct;

typedef struct {
    uint8_t  err;          /* 0=正常，见说明书 err 表 8~E */
    float    position_angle;
    float    velocity_rad_s;
    float    torque_Nm;
    float    temp_MOS_C;
    float    temp_COIL_C;
}DM_Motor_Status;

/* 控制块：状态 + PID + 前馈 */
typedef struct {
    DM_Motor_Status  Status;
    DM_Motor_MIT_Struct       MIT_Struct;
    PID_Param_float          Angle_PID;
    DM_Motor_MaxConfig       MaxConfig;
    // PID_Param_float          speed_PID;
} DM_MotorControlBlock;

/*-------------------- API --------------------*/

//驱动的前置必要部分
void DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t MotorID);
void DM_Motor_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t MotorID);
void DM_Motor_MaxConfigInit(DM_Motor_MaxConfig* MaxConfig, float pmax, float vmax, float tmax, float kpmax, float kdmax);

void DM_J4310_MIT_Send(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, DM_Motor_MIT_Struct* Param);
void DM_J4310_MIT_Parse(DM_MotorControlBlock *h, uint8_t rx_data[8]);
void DM_Motor_MIT_4310_SetSpeed(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float speed_rad_s);
void DM_J4310_MIT_SetPosition(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float position, float speed_rad_s);
float DM_J4310_MIT_Update(DM_MotorControlBlock *h);   /* 返回电流给定（A），需再换算成 0–32A -> 0–32767 */

void DM_Motor_MIT_3519_SetSpeed(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float speed_rad_s);

#define DM_SpeedControl_Mode

#ifdef DM_SpeedControl_Mode

#endif

#endif
