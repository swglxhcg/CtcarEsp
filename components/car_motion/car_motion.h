#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "motor.h"

// 小车底盘轮子间距，单位:m
// Car chassis wheel spacing, unit :m
#define ROBOT_WIDTH                  (0.135f)
#define ROBOT_LENGTH                 (0.095f)

// 小车上下轮子、左右轮子间距和的一半。
// Half of the distance between the upper and lower wheels and the left and right wheels of the car.
#define ROBOT_APB                    (0.115f)


#define ROBOT_SPIN_SCALE             (5.0f)

// AtCar的一些参数
#define MOTOR_MAX_RPM 450                   // 电机最大转速(RPM)
#define MAX_RPM_RATIO 0.9                  // 每个电机允许的最大转速比例 MAX_RPM_ALLOWED = MOTOR_MAX_RPM * MAX_RPM_RATIO
#define MOTOR_OPERATING_VOLTAGE 7.4          // 电机工作电压(用于计算最大转速)
#define MOTOR_POWER_MAX_VOLTAGE 7.4          // 电机电源的最大电压(用于计算最大转速)
#define MOTOR_POWER_MEASURED_VOLTAGE 7.4     // 连接到电机的电源当前电压读数(用于校准)
#define COUNTS_PER_REV_WHEEL 1040              // 轮子编码器每转的脉冲数
#define COUNTS_PER_REV1 (COUNTS_PER_REV_WHEEL)              // 轮子1编码器每转的脉冲数
#define COUNTS_PER_REV2 (COUNTS_PER_REV_WHEEL)              // 轮子2编码器每转的脉冲数
#define COUNTS_PER_REV3 (COUNTS_PER_REV_WHEEL)             // 轮子3编码器每转的脉冲数
#define COUNTS_PER_REV4 (COUNTS_PER_REV_WHEEL)             // 轮子4编码器每转的脉冲数
#define WHEEL_DIAMETER 0.08                // 轮子直径(米)
#define LR_WHEELS_DISTANCE 0.17           // 左右轮子之间的距离
#define FR_WHEELS_DISTANCE 0.125            // 前后轮子之间的距离。如果是2WD/阿克曼转向模型请忽略此参数


typedef enum _motion_state {
    MOTION_STOP = 0,
    MOTION_RUN,
    MOTION_BACK,
    MOTION_LEFT,
    MOTION_RIGHT,
    MOTION_SPIN_LEFT,
    MOTION_SPIN_RIGHT,
    MOTION_BRAKE,

    MOTION_MAX_STATE
} motion_state_t;



typedef struct _car_motion
{
    float Vx;
    float Vy;
    float Wz;
} car_motion_t;



void Motion_Stop(uint8_t brake);
void Motion_Ctrl(float V_x, float V_y, float V_z);
void Motion_Ctrl_State(uint8_t state, float speed);
void Motion_Get_Speed(car_motion_t* car);


void Motion_Init(void);


#ifdef __cplusplus
}
#endif
