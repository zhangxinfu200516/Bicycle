#ifndef PID_H
#define PID_H

typedef struct {
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
} PID_Cale;

extern void PID_Cale_Init(PID_Cale * pid,float __K_P, float __K_I, float __K_D,float __I_Output_Max, float __Out_Max,float __D_T);
extern float PID_Cale_TIM_Adjust_PeriodElapsedCallback(PID_Cale *pid, float Target, float Now);
extern void Math_Constrain(float *value, float min, float max);

#endif // !PID_H
