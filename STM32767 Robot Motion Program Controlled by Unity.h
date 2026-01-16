#ifndef ROBOTCONTROL_H
#define ROBOTCONTROL_H

#include "stdint.h"
#include "stdbool.h"

// 宏定义

#define INTERPOLATION_INTERVAL 2    // 插值间隔次数
#define INTERPOLATION_COUNT    2   // 插值次数

// 机械臂数据的最大最小值
#define BASE_MIN     -180
#define BASE_MAX     180
#define ARM_MIN      -20
#define ARM_MAX      20
#define DISPLACE_MIN -15
#define DISPLACE_MAX 0
#define GRIP_MIN     0
#define GRIP_MAX     50

extern int32_t ARM_DATA[5]; 

// 步态数组
extern float gait_data[5]; // 5个电机的角度，左前，左后，右前，右后，头

extern bool is_interpolating; // 是否正在插值
extern int target_index;      // gait_data_target_const的目标索引,0为左侧，1为右侧，2为复位

void InterpolateGaitData(); // 插值函数


void ArmForward();
void ArmBackward();
void ArmRotateLeft();
void ArmRotateRight();
void ArmUp();
void ArmDown();
void Grab();
void Release();
void stop();
void Armreload();

#endif // !ROBOTCONTROL_H