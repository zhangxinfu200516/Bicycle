#ifndef PID_H
#define PID_H

typedef struct PID_Cale
{
    float kp;
    float ki;
    float kd;
    float i_out_max;
    float out_max;
    float Target;
    float Now;
    float Out;
    float D_t;
    float Integral_Error;
};
extern void Init(struct PID_Cale * pid,float __K_P, float __K_I, float __K_D,float __I_Output_Max, float __Out_Max,float __D_T);
extern float TIM_Adjust_PeriodElapsedCallback(struct PID_Cale *pid, float Target, float Now);


#endif // !PID_H
