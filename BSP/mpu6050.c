/*By 高过_Gaoguo*/
#include <math.h>
#include "mpu6050.h"
//弧度转角度1rad = 57.2958°
#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

// Setup MPU6050

#define MPU6050_ADDR 0xD0
uint16_t i2c_timeout = 1000;
const double Accel_Z_corrector = 14418.0;
uint32_t timer;
float yaw_angle = 0;
Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};
Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
uint8_t check;
uint8_t MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
//    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> � 2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> � 250 �/s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        return 0;
    }
    return 1;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
	
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    Kalman->angle += dt * rate;
		//更新协方差矩阵 P 的预测部分
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;
		//计算卡尔曼增益系数
    double S = Kalman->P[0][0] + Kalman->R_measure;
    double K[2];
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;
		//利用实际测得值纠正纠错预测值
    double gap = newAngle - Kalman->angle;
    Kalman->angle += K[0] * gap;
    Kalman->bias += K[1] * gap;
		//更新误差协方差矩阵
    double P00_temp = Kalman->P[0][0];
    double P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;
		
    return Kalman->angle;
}

void MPU6050_Read_All(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0 - 4;

		if(DataStruct->Gz > -1&&DataStruct->Gz < 1)
			DataStruct->Gz = 0;

    // Kalman angle solve
    double dt = (double)(HAL_GetTick() - timer) / 1000;
    timer = HAL_GetTick();
		
    double roll;
    double roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
        roll = atan2(DataStruct->Accel_Y_RAW,roll_sqrt) * RAD_TO_DEG;
    else
        roll = 0.0;
		
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
       DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
		
		// --- Yaw (Z轴)角度估计 ---
    // 没有磁力计无法从加速度计推断Yaw角，只能靠陀螺仪角速度积分
		
		yaw_angle += DataStruct->Gz * dt;  // 角速度积分（rad/s × s = rad）
		DataStruct->KalmanAngleZ = yaw_angle;
}
//Mahony算法通过融合陀螺仪和加速度计数据
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
#define Kp      10.0f                        // proportional gain governs rate of convergence to accelerometer/magnetometer
#define Ki      0.008f                       // integral gain governs rate of convergence of gyroscope biases
#define halfT   0.001f                   // half the sample period,sapmple freq=500Hz
float exInt = 0, eyInt = 0, ezInt = 0;    // scaled integral error
void IMU_Update(float gx, float gy, float gz, float ax, float ay, float az,double * yaw, double * pitch, double * roll)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    float temp0, temp1, temp2, temp3; 
 
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
 
    if (ax * ay * az == 0)
    {
        return;
    }
 
    norm = sqrt(ax * ax + ay * ay + az * az);       //
    ax = ax / norm;
    ay = ay / norm;
    az = az / norm;
 
    // estimated direction of gravity and flux (v and w)
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3 ;
 
    // error is sum of cross product between reference direction of fields and direction measured by sensors
    ex = (ay * vz - az * vy) ;
    ey = (az * vx - ax * vz) ;
    ez = (ax * vy - ay * vx) ;
 
    exInt = exInt + ex * Ki;
    eyInt = eyInt + ey * Ki;
    ezInt = ezInt + ez * Ki;
 
    // adjusted gyroscope measurements
    gx = gx + Kp * ex + exInt;
    gy = gy + Kp * ey + eyInt;
    gz = gz + Kp * ez + ezInt;
 
    // integrate quaternion rate and normalise
    temp0 = q0;
    temp1 = q1;
    temp2 = q2;
    temp3 = q3;
    q0 += (-temp1 * gx - temp2 * gy - temp3 * gz) * halfT;
    q1 += (temp0 * gx + temp2 * gz - temp3 * gy) * halfT;
    q2 += (temp0 * gy - temp1 * gz + temp3 * gx) * halfT;
    q3 += (temp0 * gz + temp1 * gy - temp2 * gx) * halfT;
 
    // normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 = q0 / norm;
    q1 = q1 / norm;
    q2 = q2 / norm;
    q3 = q3 / norm;
 
    *yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 57.3; // unit:degree
    *pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.3; // unit:degree
    *roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.3; // unit:degree
}

// 直接将原始数据转换为需要的单位
#define ACCEL_TO_MPS2    0.0005981445f    // (2 * 9.80665) / 32768
#define GYRO_TO_RADPS    0.000266312f     // (250 * π) / (180 * 32768)

void Mpu6050_Calculate_PeriodElapsedCallback(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 14, i2c_timeout);

    // 读取原始数据
    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    // 温度
    DataStruct->Temperature = (float)(temp / 340.0 + 36.53);
    
    // 加速度计：原始值 → m/s²（直接转换）
    DataStruct->Ax = DataStruct->Accel_X_RAW * ACCEL_TO_MPS2;
    DataStruct->Ay = DataStruct->Accel_Y_RAW * ACCEL_TO_MPS2;
    DataStruct->Az = DataStruct->Accel_Z_RAW * ACCEL_TO_MPS2 / (16384.0 / Accel_Z_corrector);
    
    // 陀螺仪：原始值 → rad/s（直接转换）
    // 注意：这里减去的是零偏，零偏单位需要保持一致
    // 如果Deviation_gyro是原始值，需要转换
    DataStruct->Gx = (DataStruct->Gyro_X_RAW) * GYRO_TO_RADPS;
    DataStruct->Gy = (DataStruct->Gyro_Y_RAW) * GYRO_TO_RADPS;
    DataStruct->Gz = (DataStruct->Gyro_Z_RAW) * GYRO_TO_RADPS;
    
    // Z轴零偏处理（在rad/s单位下）
    // 注意：之前的-4是在°/s单位下的，转换为rad/s是-4 * π/180 ≈ -0.0698 rad/s
    #define GZ_BIAS_RADPS  -0.0698f  // -4°/s 对应的 rad/s
    
    DataStruct->Gz += GZ_BIAS_RADPS;
    
    // 死区处理（在rad/s单位下）
    #define GZ_DEADZONE    0.01745f   // 约1°/s对应的rad/s
    
    if(DataStruct->Gz > -GZ_DEADZONE && DataStruct->Gz < GZ_DEADZONE)
        DataStruct->Gz = 0;
    
    double roll = 0.0f;
    IMU_Update( DataStruct->Gx, DataStruct->Gy, DataStruct->Gz, DataStruct->Ax, DataStruct->Ay, DataStruct->Az, &DataStruct->KalmanAngleZ, &DataStruct->KalmanAngleY, &roll);
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx * 57.2957795f, 0.001f);
}