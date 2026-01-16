#include "RoboctControl.h"
#include "stdint.h"
#include "stdbool.h"
#include "tim.h"
#include "Connect.h"
#include "main.h"
// 该程序中进行数据的填充，填充得到的数据存放在ARM_DATA和gait_data中

// 修改后的 ARM_DATA 含义：位移，底座，大臂，小臂，夹具
int32_t ARM_DATA[5] = {-5, 0, 0, 0, 0}; // 位移，底座，大臂，小臂，夹具
// 步态数组，仅包含电机的角度
float gait_data[5] = {0.0f,0.0f,0.0f,0.0f,0.0f}; // 五个电机的角度，左前，左后，右前，右后，头部
//模型中，左侧角度大于0时为抬起，小于0时为收起，右侧角度小于0时为抬起，大于0时为收起
//实物中，进行左侧前进步态时，左前收起，左后抬起，右前收起，右后收起
//进行右侧前进步态时，左前抬起，左后收起，右前收起，右后抬起
//-30，30，-30，30
//30，-30，30，-30

float gait_data_target_const[3][4] = {
    {-20.0f, 12.0f, -30.0f, 30.0f},     // 左侧前进
    {20.0f, -20.0f, 30.0f, -30.0f}, // 右侧前进
    {0.0f, 0.0f, 0.0f, 0.0f}          // 复位
}; // 规定四个电机预设的目标角度

// 全局变量
bool is_interpolating = false; // 是否正在插值
int target_index      = 2;     // gait_data_target_const的目标索引

void InterpolateGaitData() // 插值函数
{
    // 等待机制
    static float gait_data_target[4]  = {0.0f}; // 四个电机的目标角度
    static float gait_data_current[4] = {0.0f}; // 四个电机的当前角度
    static int interpolation_step     = 0;      // 当前插值步数
    static int interpolation_counter  = 0;      // 插值调用计数器

    interpolation_counter++;

    // 如果未开始插值，初始化插值数据
    if (!is_interpolating) {
        is_interpolating   = true;
        interpolation_step = 0; // 重置插值步数
        for (int i = 0; i < 4; i++) {
            gait_data_current[i] = gait_data[i];                            // 记录当前数据
            gait_data_target[i]  = gait_data_target_const[target_index][i]; // 设置目标数据
        }
    }
    if (interpolation_counter < INTERPOLATION_INTERVAL) {
        return;
    }
    interpolation_counter = 0; // 重置计数器

    // 插值逻辑
    if (interpolation_step < INTERPOLATION_COUNT) {
        for (int i = 0; i < 4; i++) {
            float delta  = (gait_data_target[i] - gait_data_current[i]) / INTERPOLATION_COUNT; // 计算每步增量
            gait_data[i] = gait_data_current[i] + delta * (interpolation_step + 1);            // 更新插值结果
        }
        interpolation_step++;
    } else {
        // 插值完成
        for (int i = 0; i < 4; i++) {
            gait_data[i] = gait_data_target[i]; // 确保最终值精确
        }
        is_interpolating = false;
        UsefulCommand    = false; // 重置命令标志
    }
}

/*void ArmForward()
{
    // 位移数据减1
    ARM_DATA[0] = ARM_DATA[0] - 1;
    if (ARM_DATA[0] < DISPLACE_MIN) {
        ARM_DATA[0] = DISPLACE_MIN;
    }
}

void ArmBackward()
{
    // 位移数据加1
    ARM_DATA[0] = ARM_DATA[0] + 1;
    if (ARM_DATA[0] > DISPLACE_MAX) {
        ARM_DATA[0] = DISPLACE_MAX;
    }
}

void ArmRotateLeft()
{
    // 底盘数据减10
    ARM_DATA[1] = ARM_DATA[1] - 10;
    if (ARM_DATA[1] < BASE_MIN) {
        ARM_DATA[1] = BASE_MIN;
    }
}

void ArmRotateRight()
{
    // 底盘数据加10
    ARM_DATA[1] = ARM_DATA[1] + 10;
    if (ARM_DATA[1] > BASE_MAX) {
        ARM_DATA[1] = BASE_MAX;
    }
}

void ArmUp()
{
    // 大臂和小臂数据同时减10
    ARM_DATA[2] = ARM_DATA[2] + 10;
    ARM_DATA[3] = -ARM_DATA[2];
    if (ARM_DATA[2] > ARM_MAX) {
        ARM_DATA[2] = ARM_MAX;
        ARM_DATA[3] = -ARM_MAX;
    }
    if (ARM_DATA[3] < ARM_MIN) {
        ARM_DATA[3] = ARM_MIN;
        ARM_DATA[2] = -ARM_MIN;
    }
}

void ArmDown()
{
    // 大臂和小臂数据同时加10
    ARM_DATA[2] = ARM_DATA[2] - 10;
    ARM_DATA[3] = -ARM_DATA[2];
    if (ARM_DATA[2] < ARM_MIN) {
        ARM_DATA[2] = ARM_MIN;
        ARM_DATA[3] = -ARM_MIN;
    }
    if (ARM_DATA[3] > ARM_MAX) {
        ARM_DATA[3] = ARM_MAX;
        ARM_DATA[2] = -ARM_MAX;
    }
}

void Grab()
{
    // 夹具数据归零
    ARM_DATA[4] = GRIP_MIN;
}

void Release()
{
    // 夹具数据加10
    ARM_DATA[4] = ARM_DATA[4] + 10;
    if (ARM_DATA[4] > GRIP_MAX) {
        ARM_DATA[4] = GRIP_MAX;
    }
}

void Armreload()
{
    for (int i = 0; i < 5; i++) {
        ARM_DATA[i] = 0;
    }
    ARM_DATA[0]=-5;
}*/
