#include "bsp_dm.h"

static float AngleUnify(float angle);
/* 局部函数：float<->定点 互转（与说明书映射一致） */
static int16_t  float_to_int16(float x, float x_min, float x_max, uint8_t bits);
static float    uint16_to_float(int16_t x, float x_min, float x_max, uint8_t bits);
static int16_t  float_to_int12(float x, float x_min, float x_max);
static float    int12_to_float(int16_t x, float x_min, float x_max);

/* 全局句柄，中断里直接喂数据 */

// static Fmax(float)

#define Max(a,b) ((a)>(b)?(a):(b))
#define Min(a,b) ((a)<(b)?(a):(b))

static float Fmax(float a, float b)
{
    if (a > b)
    {
        return a;
    }
    else
    {
        return b;
    }
}

static float Fmin(float a, float b)
{
    if (a > b)
    {
        return b;
    }
    else
    {
        return a;
    }
}

void DM_Motor_Enable(FDCAN_HandleTypeDef *hfdcan, uint16_t MotorID)
{
    uint8_t temp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    CAN_Transmit_STD(hfdcan, MotorID, temp, 8);
}

void DM_Motor_Disable(FDCAN_HandleTypeDef *hfdcan, uint16_t MotorID)
{
    uint8_t temp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    CAN_Transmit_STD(hfdcan, MotorID, temp, 8);
}

void DM_Motor_MaxConfigInit(DM_Motor_MaxConfig* MaxConfig, float pmax, float vmax, float tmax, float kpmax, float kdmax)
{
    MaxConfig->Pmax = pmax;
    MaxConfig->Vmax = vmax;
    MaxConfig->Tmax = tmax;
    MaxConfig->Kpmax = kpmax;
    MaxConfig->Kdmax = kdmax;
}


void DM_Motor_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t MotorID)
{
    uint8_t temp[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    CAN_Transmit_STD(hfdcan, MotorID, temp, 8);
}

void DM_MotorStatus_Update(DM_Motor_Status* Status, uint8_t* data)
{
    float temp = 0;
    Status->err = (data[0] >> 4) & 0x0F;
    int16_t pos_raw = (int16_t)((data[1] << 8) | data[2]);
    int16_t vel_raw = (int16_t)((data[3] << 4) | (data[4] >> 4));
    int16_t tor_raw = (int16_t)(((data[4] & 0x0F) << 8) | data[5]);

    temp = uint16_to_float(pos_raw, -P_MAX_RAD, P_MAX_RAD, 16) / 2.0f;
    Status->position_angle = AngleUnify(temp);
    Status->velocity_rad_s= int12_to_float(vel_raw, -V_MAX_RAD_S, V_MAX_RAD_S);
    Status->torque_Nm    = int12_to_float(tor_raw, -T_MAX_NM, T_MAX_NM);
    Status->temp_MOS_C   = (float)data[6];
    Status->temp_COIL_C  = (float)data[7];
}

/* 打包并发出 MIT 控制帧 */
void DM_J4310_MIT_Send(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, DM_Motor_MIT_Struct* Param)
{
    uint8_t  tx[8];
    int16_t  p_u, v_u, kp_u, kd_u, t_u;

    /* 限幅 */
    float p = fmaxf(-P_MAX_RAD,   fminf(Param->position_angle,  P_MAX_RAD));
    float v = fmaxf(-V_MAX_RAD_S, fminf(Param->velocity_rad_s,V_MAX_RAD_S));
    float kp= fmaxf(0,            fminf(Param->kp,           KP_MAX));
    float kd= fmaxf(0,            fminf(Param->kd,           KD_MAX));
    float t = fmaxf(-T_MAX_NM,    fminf(Param->torque_Nm,    T_MAX_NM));

    /* 转定点 */
    p_u  = float_to_int16(p , -P_MAX_RAD , P_MAX_RAD , 16);
    v_u  = float_to_int12(v , -V_MAX_RAD_S, V_MAX_RAD_S);
    kp_u = float_to_int12(kp, 0, KP_MAX);
    kd_u = float_to_int12(kd, 0, KD_MAX);
    t_u  = float_to_int12(t , -T_MAX_NM, T_MAX_NM);

    /* 位拼接（大端） */
    tx[0] = (p_u >> 8) & 0xFF;
    tx[1] = (p_u >> 0) & 0xFF;
    tx[2] = (v_u >> 4) & 0xFF;
    tx[3] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F);
    tx[4] = (kp_u >> 0) & 0xFF;
    tx[5] = (kd_u >> 4) & 0xFF;
    tx[6] = ((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F);
    tx[7] = (t_u >> 0) & 0xFF;

    CAN_Transmit_STD(hcan, MotorID, tx, 8);
}

void DM_3519_MIT_Send(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, DM_Motor_MIT_Struct* Param)
{
    uint8_t  tx[8];
    int16_t  p_u, v_u, kp_u, kd_u, t_u;

    /* 限幅 */
    float p = fmaxf(-P_MAX_RAD_3519,   fminf(Param->position_angle,  P_MAX_RAD_3519));
    float v = fmaxf(-V_MAX_RAD_S_3519, fminf(Param->velocity_rad_s,V_MAX_RAD_S_3519));
    float kp= fmaxf(0,            fminf(Param->kp,           KP_MAX_3519));
    float kd= fmaxf(0,            fminf(Param->kd,           KD_MAX_3519));
    float t = fmaxf(-T_MAX_NM_3519,    fminf(Param->torque_Nm,    T_MAX_NM_3519));

    /* 转定点 */
    p_u  = float_to_int16(p , -P_MAX_RAD_3519 , P_MAX_RAD_3519 , 16);
    v_u  = float_to_int12(v , -V_MAX_RAD_S_3519, V_MAX_RAD_S_3519);
    kp_u = float_to_int12(kp, 0, KP_MAX_3519);
    kd_u = float_to_int12(kd, 0, KD_MAX_3519);
    t_u  = float_to_int12(t , -T_MAX_NM_3519, T_MAX_NM_3519);

    /* 位拼接（大端） */
    tx[0] = (p_u >> 8) & 0xFF;
    tx[1] = (p_u >> 0) & 0xFF;
    tx[2] = (v_u >> 4) & 0xFF;
    tx[3] = ((v_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F);
    tx[4] = (kp_u >> 0) & 0xFF;
    tx[5] = (kd_u >> 4) & 0xFF;
    tx[6] = ((kd_u & 0x0F) << 4) | ((t_u >> 8) & 0x0F);
    tx[7] = (t_u >> 0) & 0xFF;

    CAN_Transmit_STD(hcan, MotorID, tx, 8);
}


//void DM_Motor_MIT_SetSpeed(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float speed_rad_s)
//{
//    DM_Motor_MIT_Struct Param;
//    Param.position_angle = 0.0f;
//    Param.velocity_rad_s = speed_rad_s;
//    Param.kp = 0;
//    Param.kd = DM_Kspeed;
//    Param.torque_Nm = 0.00;
//    DM_J4310_MIT_Send(hcan, MotorID, &Param);
//}

void DM_J4310_MIT_SetPosition(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float position, float speed_rad_s)
{
    DM_Motor_MIT_Struct Param;
    Param.position_angle = position;
    Param.velocity_rad_s = speed_rad_s;
    Param.kp = DM_Kposition;
    Param.kd = DM_Kspeed;
    Param.torque_Nm = 0.00;
    DM_J4310_MIT_Send(hcan, 0x011, &Param);
}

void DM_J4310_MIT_Parse(DM_MotorControlBlock *h, uint8_t d[8])
{
    h->Status.err = (d[0] >> 4) & 0x0F;
    int16_t pos_raw = (int16_t)((d[1] << 8) | d[2]);
    int16_t vel_raw = (int16_t)((d[3] << 4) | (d[4] >> 4));
    int16_t tor_raw = (int16_t)(((d[4] & 0x0F) << 8) | d[5]);

    h->Status.position_angle = uint16_to_float(pos_raw, -P_MAX_RAD, P_MAX_RAD, 16);
    h->Status.position_angle = AngleUnify(h->Status.position_angle);
    h->Status.velocity_rad_s= int12_to_float(vel_raw, -V_MAX_RAD_S, V_MAX_RAD_S);
    h->Status.torque_Nm    = int12_to_float(tor_raw, -T_MAX_NM, T_MAX_NM);
    h->Status.temp_MOS_C   = (float)d[6];
    h->Status.temp_COIL_C  = (float)d[7];
}

void DM_Motor_MIT_4310_SetSpeed(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float speed_rad_s)
{
    DM_Motor_MIT_Struct Param;
    Param.position_angle = 0.0f;
    Param.velocity_rad_s = speed_rad_s;
    Param.kp = 0;
    Param.kd = DM_Kspeed;
    Param.torque_Nm = 0.00;
    // DM_MIT_Send(hcan, MotorID, &Param);
    DM_J4310_MIT_Send(hcan, MotorID, &Param);
}

void DM_Motor_MIT_3519_SetSpeed(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float speed_rad_s)
{
    DM_Motor_MIT_Struct Param;
    Param.position_angle = 0.0f;
    Param.velocity_rad_s = speed_rad_s;
    Param.kp = 0;
    Param.kd = DM_Kspeed;
    Param.torque_Nm = 0.00;
    // DM_MIT_Send(hcan, MotorID, &Param);
    DM_3519_MIT_Send(hcan, MotorID, &Param);
}

#ifdef DM_SpeedControl_Mode

void DM_Motor_SpeedControl_SetSpeed(FDCAN_HandleTypeDef *hcan, uint16_t MotorID, float speed_rad_s)
{
    // DM_Motor_MIT_Struct Param;
    // Param.position_angle = 0.0f;
    // Param.velocity_rad_s = speed_rad_s;
    // Param.kp = 0;
    // Param.kd = DM_Kspeed;
    // Param.torque_Nm = 0.00;
    // DM_J4310_MIT_Send(&hfdcan1, MotorID, &Param);
}

#endif

static float AngleUnify(float angle)
{
    // 角度范围 [-180, 180]
    if (angle < -3.1415926)
    {
        angle += 6.2831853;
    }
    angle = angle / 3.1415926 * 180.0f;
    return angle;
}

/*-------------------- 定点转换 --------------------*/
static int16_t float_to_int16(float x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    if (offset < 0.0f) offset = 0.0f;
    if (offset > span) offset = span;
    return (int16_t)(offset / span * 65535);
}
static float uint16_to_float(int16_t x, float x_min, float x_max, uint8_t bits)
{
    float span = x_max - x_min;
    return x_min + span * (float)x / ((1 << bits) - 1);
}
static int16_t float_to_int12(float x, float x_min, float x_max)
{
    float span = x_max - x_min;
    float offset = x - x_min;
    if (offset < 0.0f) offset = 0.0f;
    if (offset > span) offset = span;
    return (int16_t)(offset / span * 4095.0f);
}
static float int12_to_float(int16_t x, float x_min, float x_max)
{
    float span = x_max - x_min;
    return x_min + span * (float)x / 4095.0f;
}
