#include "PID.h"

#define MAX(a,b)((a>b)?a:b)
#define MIN(a,b)((a>b)?b:a)
#define ABS(a)((a>0)?a:-a)


void PID_Set_Kparam_int32(PID_Param_int32* PID_Param, float Kp, float Ki, float Kd)
{
    PID_Param->Kp = Kp;
    PID_Param->Ki = Ki;
    PID_Param->Kd = Kd;
}

void PID_Set_Kparam_float(PID_Param_float* PID_Param, float Kp, float Ki, float Kd)
{
    PID_Param->Kp = Kp;
    PID_Param->Ki = Ki;
    PID_Param->Kd = Kd;
}

void PID_Set_ErrorDeadZone_int32(PID_Param_int32* PID_Param, int32_t DeadZone)
{
    PID_Param->ErrorDeadZone = DeadZone;
}

void PID_Set_ErrorDeadZone_float(PID_Param_float* PID_Param, float DeadZone)
{
    PID_Param->ErrorDeadZone = DeadZone;
}

void PID_Set_Integral_int32(PID_Param_int32* PID_Param, int32_t Range, int32_t Limit)
{
    PID_Param->IntegralRange = Range;
    PID_Param->IntegralLimit = Limit;
}

void PID_Set_Integral_float(PID_Param_float* PID_Param, float Range, float Limit)
{
    PID_Param->IntegralRange = Range;
    PID_Param->IntegralLimit = Limit;
}

void PID_Set_Target_int32(PID_Param_int32* PID_Param, int32_t Target)
{
    PID_Param->Target = Target;
}

void PID_Set_Target_float(PID_Param_float* PID_Param, float Target)
{
    PID_Param->Target = Target;
}

void PID_Set_FeedForeward_int32(PID_Param_int32* PID_Param, int32_t Feedforward)
{
    PID_Param->FeedForeward = Feedforward;
}

void PID_Set_FeedForeward_float(PID_Param_float* PID_Param, float Feedforward)
{
    PID_Param->FeedForeward = Feedforward;
}

void PID_Set_OutputLimit_int32(PID_Param_int32* PID_Param, int32_t Limit)
{
    PID_Param->Output_Limit = Limit;
}

void PID_Set_OutputLimit_float(PID_Param_float* PID_Param, float Limit)
{
    PID_Param->Output_Limit = Limit;
}

int32_t PID_Update_int32(PID_Param_int32* PID_Param, int32_t ValueNow)
{
    PID_Param->ValueNow = ValueNow;
    PID_Param->ErrorNow = PID_Param->Target - PID_Param->ValueNow;

    if (PID_Param->ErrorNow <= PID_Param->ErrorDeadZone && PID_Param->ErrorNow >= -PID_Param->ErrorDeadZone)
    {
        PID_Param->ErrorNow = 0;
    }

    if (PID_Param->IntegralRange != 0)
    {
        if (ABS(PID_Param->ErrorNow) <= PID_Param->IntegralRange)
        {
            PID_Param->ErrorSum += PID_Param->ErrorNow;
        }
    }
    else
    {
        PID_Param->ErrorSum += PID_Param->ErrorNow;
    }
    
    if (PID_Param->IntegralLimit != 0)
    {
        if (PID_Param->ErrorSum >= ABS(PID_Param->IntegralLimit))
        {
            PID_Param->ErrorSum = ABS(PID_Param->IntegralLimit);
        }
        else if (PID_Param->ErrorSum < -ABS(PID_Param->IntegralLimit))
        {
            PID_Param->ErrorSum = -ABS(PID_Param->IntegralLimit);
        }
    }
    
    PID_Param->Output = PID_Param->Kp * PID_Param->ErrorNow
                      + PID_Param->Ki * PID_Param->ErrorSum
                      + PID_Param->Kd * (PID_Param->ValueNow - PID_Param->ValueLast)
                      + PID_Param->FeedForeward;

    
    PID_Param->ValueLast = PID_Param->ValueNow;
                      
    //限制最大值
    if (PID_Param->Output_Limit != 0)
    {
        if (PID_Param->Output > 0)
        {
            PID_Param->Output =  MIN(PID_Param->Output, ABS(PID_Param->Output_Limit));
        }
        else if (PID_Param->Output < 0)
        {
            PID_Param->Output =  MAX(PID_Param->Output, -ABS(PID_Param->Output_Limit));
        }
    }
    return PID_Param->Output;
}

float PID_Update_float(PID_Param_float* PID_Param, float ValueNow)
{
    PID_Param->ValueNow = ValueNow;
    PID_Param->ErrorNow = PID_Param->Target - PID_Param->ValueNow;

    if (PID_Param->ErrorNow <= PID_Param->ErrorDeadZone && PID_Param->ErrorNow >= -PID_Param->ErrorDeadZone)
    {
        PID_Param->ErrorNow = 0;
    }

    if (PID_Param->IntegralRange != 0)
    {
        if (PID_Param->ErrorNow <= PID_Param->IntegralRange && PID_Param->ErrorNow >= -PID_Param->IntegralRange)
        {
            PID_Param->ErrorSum += PID_Param->ErrorNow;
        }
    }
    else
    {
        PID_Param->ErrorSum += PID_Param->ErrorNow;
    }
    
    if (PID_Param->IntegralLimit != 0)
    {
        if (PID_Param->ErrorSum >= fabs(PID_Param->IntegralLimit))
        {
            PID_Param->ErrorSum = fabs(PID_Param->IntegralLimit);
        }
        else if (PID_Param->ErrorSum < -fabs(PID_Param->IntegralLimit))
        {
            PID_Param->ErrorSum = -fabs(PID_Param->IntegralLimit);
        }
    }
    
    PID_Param->Output = PID_Param->Kp * PID_Param->ErrorNow
                      + PID_Param->Ki * PID_Param->ErrorSum
                      + PID_Param->Kd * (PID_Param->ValueNow - PID_Param->ValueLast)
                      + PID_Param->FeedForeward;

    if (PID_Param->Output_Limit != 0)
    {
        if (PID_Param->Output > 0)
        {
            PID_Param->Output =  MIN(PID_Param->Output, ABS(PID_Param->Output_Limit));
        }
        else if (PID_Param->Output < 0)
        {
            PID_Param->Output =  MAX(PID_Param->Output, -ABS(PID_Param->Output_Limit));
        }
    }
    return PID_Param->Output;
}

void PID_ClearUp_float(PID_Param_float* PID_Param)
{
    PID_Param->ValueLast = 0;
    PID_Param->ErrorSum = 0;
    // PID_Param->Target = 0;
}

void PID_ClearUp_int32(PID_Param_int32* PID_Param)
{
    PID_Param->ValueLast = 0;
    PID_Param->ErrorSum = 0;
    // PID_Param->Target = 0;
}
