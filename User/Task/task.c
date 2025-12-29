#include "task.h"
#include "tim.h"
#include "mpu6050.h"
#include "i2c.h"
#include <stdbool.h>
#include "pid.h"
#include "BlueTooth.h"
#include "usart.h"
#ifdef NEW
//用户参数接口区域
bool init_finished = 0;
MPU6050_t mpu6050;
PID_Cale pid_roll;
PID_Cale pid_windmill;
float Servo_angle_output = 90.0f,Wheel_pwm = 0.0f,motor2_pwm = 0.0f;
float Servo_angle_Zero = -0.0f;
float Wheel_Gy_K = 0.1f;
//声明函数区域
void Control_Motor(int16_t Pwm,GPIO_PinState _Enable_STBY_Pin);
void Control_Motor2(int16_t Pwm,GPIO_PinState _Enable_STBY_Pin);

void Task_Init()
{
    //定时器1使能
    HAL_TIM_Base_Start_IT(&htim1);
    //pwm使能
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    //
    Control_Motor(0,GPIO_PIN_RESET);
    //蓝牙模块初始化
    HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
    //MPU6050初始化
    while(MPU6050_Init(&hi2c1))
	{
		HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
		HAL_Delay(499);
	}
    
	HAL_Delay(999); 
    //pid初始化
    PID_Cale_Init(&pid_roll,1.0f,0.8f,0.0f,5.0f,10.0f,0.002f);
    PID_Cale_Init(&pid_windmill,800.0f,0.8f,0.0f,100.0f,10000.0f,0.002f);
    init_finished = 1;
}
void Task_Loop()
{
    // 读取MPU6050数据
    //MPU6050_Read_All(&hi2c1, &mpu6050);
    // HAL_Delay(1);
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    if (huart1.ErrorCode)
    {
        HAL_UART_DMAStop(&huart1); // 停止以重启
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, Bluetooth_RxData, Bluetooth_length);
    }
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //判断程序初始化完成
    if(init_finished == 0)
    {
        return;
    }

    //选择回调函数
    if (htim->Instance == TIM1)
    {
        // 读取MPU6050数据
        MPU6050_Read_All(&hi2c1, &mpu6050);
        // 蓝牙按键数据处理
        Key_Scan();
        //PID闭环控制车体倾斜角度
        static uint8_t mod2 = 0;
        mod2++;
        if(mod2 == 2)
        {
            int16_t set_pwm;
            mod2 = 0;
            //控制风车
            motor2_pwm = PID_Cale_TIM_Adjust_PeriodElapsedCallback(&pid_windmill,Servo_angle_Zero,mpu6050.KalmanAngleX);
            // 计算舵机角度
            Servo_angle_output = 90.0f + PID_Cale_TIM_Adjust_PeriodElapsedCallback(&pid_roll,Servo_angle_Zero,mpu6050.KalmanAngleX) - Wheel_Gy_K * mpu6050.Gx;
            if (PorcessData.Key[10] == Key_Status_PRESSED)
            {
                Servo_angle_output = 90.0f;
                set_pwm = (int16_t)(500.0f + Servo_angle_output / 180.0f * (2500.0f - 500.0f));
            }
            else
            {
                Servo_angle_output = 90.0f;
                motor2_pwm = 0.0f;
            }
            __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, set_pwm);
             Control_Motor2(motor2_pwm,GPIO_PIN_SET);
        }
        // 控制电驱
        Wheel_pwm = (int16_t)(PorcessData.Remote_Left_X * 10000.0f);
        if(PorcessData.Key[11] == Key_Status_FREE)
            Wheel_pwm = 0;
        else if(PorcessData.Key[11] == Key_Status_PRESSED)
            Wheel_pwm = 12000;
        Control_Motor(Wheel_pwm ,GPIO_PIN_SET);
    }
}
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1)
    {
      if(Bluetooth_RxData[0] == 0xAA && Bluetooth_RxData[1] == 0x55)
      {
        Bluetooth_Data_Process(Bluetooth_RxData);
      }
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1,Bluetooth_RxData,Bluetooth_length);
    }
}
//控制电机
void Control_Motor(int16_t Pwm,GPIO_PinState _Enable_STBY_Pin)
{
    if(Pwm >= 0)
    {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
    }
    else 
    {
        HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
    }
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, _Enable_STBY_Pin);

    Pwm = (Pwm >= 0) ? Pwm : -Pwm;
   if(Pwm >20000)
        Pwm = 20000;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, Pwm); //设置电驱PWM 范围为0 - 20000
}
void Control_Motor2(int16_t Pwm,GPIO_PinState _Enable_STBY_Pin)
{
    if(Pwm >= 0)
    {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
    }
    else 
    {
        HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET);
    }
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, _Enable_STBY_Pin);

   Pwm = (Pwm >= 0) ? Pwm : -Pwm;
   if(Pwm >20000)
        Pwm = 20000;
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Pwm); //设置电驱PWM 范围为0 - 20000
}
#endif