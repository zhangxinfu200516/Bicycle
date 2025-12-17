#include "pid.h"

/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

void Math_Constrain(float *value, float min, float max)
{
    if (*value < min)
    {
        *value = min;
    }
    else if (*value > max)
    {
        *value = max;
    }
}
/**
 * @brief PID初始化
 *
 * @param __K_P P值
 * @param __K_I I值
 * @param __K_D D值
 * @param __K_F 前馈
 * @param __I_Out_Max 积分限幅
 * @param __Out_Max 输出限幅
 * @param __D_T 时间片长度
 */
void Init(struct PID_Cale * pid,float __K_P, float __K_I, float __K_D,float __I_Output_Max, float __Out_Max,float __D_T)
{
   pid->kp = __K_P;
   pid->ki = __K_I;
   pid->kd = __K_D;
   pid->D_t = __D_T;
   pid->i_out_max = __I_Output_Max;
   pid->out_max = __Out_Max;
}

/**
 * @brief PID调整值
 *
 * @return float 输出值
 */
float TIM_Adjust_PeriodElapsedCallback(struct PID_Cale *pid, float Target, float Now)
{
    // P输出
    float p_out = 0.0f;
    // I输出
    float i_out = 0.0f;
    // D输出
    float d_out = 0.0f;
    // 误差
    float error;
    // 上一次误差
    float last_error;
    // 线性非变速积分
    float speed_ratio = 1.0f;

    error = Target - Now;

    pid->Target = Target;

    pid->Now = Now;
    // 计算p项
    p_out = pid->kp * error;
    // 计算i项
    // 积分限幅
    if (pid->i_out_max != 0.0f)
    {
        Math_Constrain(&pid->Integral_Error, -pid->i_out_max / pid->ki, pid->i_out_max / pid->ki);
    }
    // 没有积分分离
    pid->Integral_Error += speed_ratio * pid->D_t * error;
    i_out = pid->ki * pid->Integral_Error;

    // 计算d项

    // 没有微分先行
    d_out = pid->kd * (error - last_error) / pid->D_t;

    // 计算总共的输出

    pid->Out = p_out + i_out + d_out;
    // 输出限幅
    if (pid->Out != 0.0f)
    {
        Math_Constrain(&pid->Out, -pid->out_max, pid->out_max);
    }

    // 善后工作
    last_error = error;

    return pid->Out;
}
/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/