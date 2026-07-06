#ifndef __PID_H
#define __PID_H

#define STM32H7

#ifdef STM32F4
#include "stm32f4xx_hal.h"
#endif // DEBUG

#ifdef STM32F1
#include "stm32f1xx_hal.h"
#endif // DEBUG

#ifdef STM32H7
#include "stm32h7xx_hal.h"
#endif 


#include "math.h"

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    int32_t Target;
    int32_t ValueNow;
    int32_t ValueLast;
    int32_t ErrorNow;
    int32_t ErrorSum;
    int32_t ErrorDeadZone;

    int32_t FeedForeward;

    int32_t IntegralRange;
    int32_t IntegralLimit;

    int32_t Output;
    int32_t Output_Limit;

}PID_Param_int32;

typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Target;
    float ValueNow;
    float ValueLast;
    float ErrorNow;
    float ErrorSum;
    float ErrorDeadZone;

    float FeedForeward;

    float IntegralRange;
    float IntegralLimit;

    float Output;
    float Output_Limit;

}PID_Param_float;

void PID_Set_Kparam_int32(PID_Param_int32* PID_Param, float Kp, float Ki, float Kd);
void PID_Set_Kparam_float(PID_Param_float* PID_Param, float Kp, float Ki, float Kd);
void PID_Set_ErrorDeadZone_int32(PID_Param_int32* PID_Param, int32_t DeadZone);
void PID_Set_ErrorDeadZone_float(PID_Param_float* PID_Param, float DeadZone);
void PID_Set_Integral_int32(PID_Param_int32* PID_Param, int32_t Range, int32_t Limit);
void PID_Set_Integral_float(PID_Param_float* PID_Param, float Range, float Limit);
void PID_Set_Target_int32(PID_Param_int32* PID_Param, int32_t Target);
void PID_Set_Target_float(PID_Param_float* PID_Param, float Target);
void PID_Set_OutputLimit_int32(PID_Param_int32* PID_Param, int32_t Limit);
void PID_Set_OutputLimit_float(PID_Param_float* PID_Param, float Limit);
int32_t PID_Update_int32(PID_Param_int32* PID_Param, int32_t ValueNow);
float PID_Update_float(PID_Param_float* PID_Param, float ValueNow);

void PID_Set_FeedForeward_int32(PID_Param_int32* PID_Param, int32_t Feedforward);
void PID_Set_FeedForeward_float(PID_Param_float* PID_Param, float Feedforward);


void PID_ClearUp_float(PID_Param_float* PID_Param);
void PID_ClearUp_int32(PID_Param_int32* PID_Param);



#endif // !__PID_H
