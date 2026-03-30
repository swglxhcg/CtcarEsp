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

// 从编码器读取当前各轮子速度，单位mm/s
// Read the current speed of each wheel from the encoder in mm/s
void Motion_Get_Speed(car_motion_t* car)
{
    float speed_m1 = 0, speed_m2 = 0, speed_m3 = 0, speed_m4 = 0;
    Motor_Get_Speed(&speed_m1, &speed_m2, &speed_m3, &speed_m4);

    float robot_APB = Motion_Get_APB();

    car->Vx = (speed_m1 + speed_m2 + speed_m3 + speed_m4) / 4.0;
    car->Vy = (-speed_m1 + speed_m2 + speed_m3 - speed_m4) / 4.0;
    car->Wz = (-speed_m1 - speed_m2 + speed_m3 + speed_m4) / 4.0f / robot_APB;
}

// Returns half of the sum of the current cart wheel axles  返回当前小车轮子轴间距和的一半
float Motion_Get_APB(void)
{
    return ROBOT_APB;
}

// Returns the number of millimeters at which the current wheel has been turned  返回当前小车轮子转一圈多少毫米
float Motion_Get_Circle_MM(void)
{
    return MECANUM_CIRCLE_MM;
}

float Motion_Get_Circle_M(void)
{
    return MECANUM_CIRCLE_M;
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
    float robot_APB = Motion_Get_APB();
    float speed_lr = -V_y;
    float speed_fb = V_x;
    float speed_spin = -V_z * robot_APB;
    if (V_x == 0 && V_y == 0 && V_z == 0)
    {
        Motion_Stop(STOP_BRAKE);
        return;
    }

    // ESP_LOGI("CatMotion", "CtrlSpeedMotor: %.2f, %.2f, %.2f, %.2f", speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
    speed_L1_setup = speed_fb + speed_lr + speed_spin;
    speed_L2_setup = speed_fb - speed_lr + speed_spin;
    speed_R1_setup = speed_fb - speed_lr - speed_spin;
    speed_R2_setup = speed_fb + speed_lr - speed_spin;

    if (speed_L1_setup > MECANUM_LIMIT_SPEED) speed_L1_setup = MECANUM_LIMIT_SPEED;
    if (speed_L1_setup < -MECANUM_LIMIT_SPEED) speed_L1_setup = -MECANUM_LIMIT_SPEED;
    if (speed_L2_setup > MECANUM_LIMIT_SPEED) speed_L2_setup = MECANUM_LIMIT_SPEED;
    if (speed_L2_setup < -MECANUM_LIMIT_SPEED) speed_L2_setup = -MECANUM_LIMIT_SPEED;
    if (speed_R1_setup > MECANUM_LIMIT_SPEED) speed_R1_setup = MECANUM_LIMIT_SPEED;
    if (speed_R1_setup < -MECANUM_LIMIT_SPEED) speed_R1_setup = -MECANUM_LIMIT_SPEED;
    if (speed_R2_setup > MECANUM_LIMIT_SPEED) speed_R2_setup = MECANUM_LIMIT_SPEED;
    if (speed_R2_setup < -MECANUM_LIMIT_SPEED) speed_R2_setup = -MECANUM_LIMIT_SPEED;
    // 此处设置的速度单位为m/s，均为正值时，小车会向前进；即不论左右轮，值为正数时，轮子提供向前的速度
    Motor_Set_Speed(speed_L1_setup, speed_L2_setup, speed_R1_setup, speed_R2_setup);
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
