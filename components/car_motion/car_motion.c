#include "car_motion.h"
#include <math.h>

#include "stdio.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "motor.h"

#define PI 3.1415926
#define _MECANUM_ABCL (0.14750)

car_motion_t micro_car;

// 线速度和角速度
static float line_v = 0;
static float angular_v = 0;

static float speed_L1_setup = 0;
static float speed_L2_setup = 0;
static float speed_R1_setup = 0;
static float speed_R2_setup = 0;

// 限制值在指定范围内
int constrain(int value, int min_value, int max_value)
{
    if (value < min_value) {
        return min_value;
    } else if (value > max_value) {
        return max_value;
    } else {
        return value;
    }
}

// 麦轮解算函数
void mecanum_kinematics(float vx, float vy, float wz, float L) {
    // 输入：
    //   vx：机体坐标系下的X方向线速度（m/s，向前为正）
    //   vy：机体坐标系下的Y方向线速度（m/s，向左为正）
    //   wz：机体坐标系下的角速度（rad/s，逆时针为正）
    //   L：轮子中心到机器人旋转中心的距离（m，轮距参数，需根据机械结构测量）
    // 输出：
    //   speed_L1_setup：前左轮的线速度（m/s）
    //   speed_L2_setup：后左轮的线速度（m/s）
    //   speed_R1_setup：前右轮的线速度（m/s）
    //   speed_R2_setup：后右轮的线速度（m/s）

    // 计算每个轮子的线速度（单位：m/s）
	speed_L1_setup = (vx-vy-wz*_MECANUM_ABCL);
	speed_L2_setup = (vx+vy-wz*_MECANUM_ABCL);
	speed_R1_setup = -(-vx-vy-wz*_MECANUM_ABCL);
	speed_R2_setup = -(-vx+vy-wz*_MECANUM_ABCL);
}

// 麦轮逆解算函数
void mecanum_inverse_kinematics(double speed_L1, double speed_L2, double speed_R1, double speed_R2, car_motion_t* car) {
    // 输入：
    //   wheel_speeds[4]：四个轮子的线速度（m/s），顺序为[FL, FR, BL, BR]
    //   L：轮子中心到机器人旋转中心的距离（m，与正解算的L一致）
    // 输出：
    //   vx：机体X方向线速度（m/s，向前为正）
    //   vy：机体Y方向线速度（m/s，向左为正）
    //   wz：机体角速度（rad/s，逆时针为正）

    // 计算线速度vx（x方向）
    car->Vx = (speed_L1 + speed_L2 + speed_R1 + speed_R2) / 4.0;
    
    // 计算线速度vy（y方向）
    car->Vy = (speed_L2 + speed_R1 - speed_L1 - speed_R2) / 4.0;
    
    // 计算角速度wz（绕z轴）
    car->Wz = (speed_R2 - speed_L1) / (2.0 * _MECANUM_ABCL);
}


// 小车停止 Car stop
void Motion_Stop(uint8_t brake)
{
    Motor_Stop(brake);
}

// 控制小车运动
// Control car motion
void Motion_Ctrl(float V_x, float V_y, float V_z)
{
    // calculateRPM(V_x, V_y, V_z);
    mecanum_kinematics(V_x, V_y, V_z, 0.1);
    line_v = V_x;
    angular_v = V_z;

    // ESP_LOGI("CatMotion", "CtrlSpeedMotor: %.2f, %.2f, %.2f, %.2f", speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
    
    // 此处设置的速度单位为m/s，均为正值时，小车会向前进；即不论左右轮，值为正数时，轮子提供向前的速度
    Motor_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
}

// 获取小车运动的速度
// Get the speed of the car's motion
void Motion_Get_Speed(car_motion_t* car)
{
    float speed_m1 = 0, speed_m2 = 0, speed_m3 = 0, speed_m4 = 0;
    Motor_Get_Speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);

    // ESP_LOGI("CatMotion", "GetSpeedMotor: %.2f, %.2f, %.2f, %.2f", speed_m1, speed_m2, speed_m3, speed_m4);

    mecanum_inverse_kinematics(speed_m1, speed_m2, speed_m3, speed_m4, car);
    if(car->Wz == 0) car->Wz = 0;
    // ESP_LOGI("CatMotion", "GetSpeed: %.2f, %.2f, %.2f", car->Vx, car->Vy, car->Wz);
}

// 控制小车的运动状态
// Control the motion state of the car
void Motion_Ctrl_State(uint8_t state, float speed)
{
    if (speed < 0) speed = -speed;
    if (speed > 1.0) speed = 1.0;
    switch (state)
    {
    case MOTION_STOP:
        Motion_Stop(STOP_COAST);
        break;
    case MOTION_RUN:
        Motion_Ctrl(speed, 0, 0);
        break;
    case MOTION_BACK:
        Motion_Ctrl(-speed, 0, 0);
        break;
    case MOTION_LEFT:
        Motion_Ctrl(speed, 0, speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_RIGHT:
        Motion_Ctrl(speed, 0, -speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_LEFT:
        Motion_Ctrl(0, 0, speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_SPIN_RIGHT:
        Motion_Ctrl(0, 0, -speed*ROBOT_SPIN_SCALE);
        break;
    case MOTION_BRAKE:
        Motion_Stop(STOP_BRAKE);
        break;
    default:
        break;
    }
}
