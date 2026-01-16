#include "Connect.h"
#include "stdint.h"
#include "stdbool.h"
#include "RoboctControl.h"
#include "spi.h"
#include "tim.h"
#include "main.h"

#define k1 0.2
#define k2 0.2
// 全局变量

uint8_t TxDataBufferFromUnity[UartTransmitDataLength]; // SPI发送数据缓冲区
uint8_t RXDataBufferFromUnity[UartTransmitDataLength]; // SPI接收数据缓冲区

uint8_t getdata_flag; // 测试
int testtimes;        // 测试
bool UsefulCommand = false;

int DataFromUnityToRobot = 0; // Unity控制实物变量 0为无操作，1为左腿，2为右腿，3为复位，4为左转，5为右转，6为大臂上，7为大臂下，8为小臂上，9为小臂下，执行完有效命令后归0
// 工具函数
void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim2, 0);           // 将计数器值设为0
    HAL_TIM_Base_Start(&htim2);                 // 启动定时器
    while (__HAL_TIM_GET_COUNTER(&htim2) < us); // 等待计数器达到指定值
    HAL_TIM_Base_Stop(&htim2);                  // 停止定时器
}

void data_to_uint8(void *input, uint8_t *output, size_t input_size)
{
    uint8_t *input_bytes = (uint8_t *)input;
    for (size_t i = 0; i < input_size; i++) {
        output[i] = input_bytes[i];
    }
}

void ClearBuffer(uint8_t *buffer, size_t length)
{
    memset(buffer, 0, length);
}
/// @brief ///////////////////////////////////////////////////////////
void DataPutin() // 填充电机数据给TxDataBufferFromUnity，下标从0开始,左前，右后，右前，左后
{
    FillMotorData();
    FillArmData();
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) // 定期在定时器中断中启动SPI通信
{
    if (htim == &htim2) {
        DataPutin(); // 填充数据
        SPI_TransmitReceive_NSS();
    }
}

void SPI_TransmitReceive_NSS() // 手动拉低NSS启动SPI通信中断式通信,发送数据在调用该函数之前或者在该函数中填充，接收数据在回调函数中处理
{

    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, TxDataBufferFromUnity, RXDataBufferFromUnity, UartTransmitDataLength, 1); // 进行SPI通信
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_10, GPIO_PIN_SET);
    // 通信完毕后进行数据解析
    GetCommandFromUnity(RXDataBufferFromUnity[0]);
    if (UsefulCommand) {
        InterpolateGaitData(); // 计算腿部插值数据
    }
}

void GetCommandFromUnity(uint8_t data) // 获取从Unity传来的命令
{
    if (data == 0x03) {
        UsefulCommand    = true;
        target_index     = 2;
        gait_data[4]     = 0;     // 头部复位
        is_interpolating = false; // 重启插值，目标角度为全0
        for (int i = 0; i < 3; i++) {
            Servo_Position[i] = 500;
        }
        Servo_Position[3]    = 1500; // 机械臂位置数据复位
        DataFromUnityToRobot = 3;    // 此时实物和unity复位
    }
    if (is_interpolating == false) {
        switch (data) {
            case 0x01: // 左腿前进
                UsefulCommand        = true;
                target_index         = 0;
                DataFromUnityToRobot = 1;
                break;
            case 0x02: // 右腿前进
                UsefulCommand        = true;
                target_index         = 1;
                DataFromUnityToRobot = 2;
                break;
            default:
                break;
        }
    }
    switch (data) {

        case 0x04: // 左转
            DataFromUnityToRobot = 4;
            gait_data[4]         = -20.0f; // 头部左转
            break;
        case 0x05: // 右转
            DataFromUnityToRobot = 5;
            gait_data[4]         = 20.0f; // 头部右转
            break;
        case 0x06: // 头部归正
            DataFromUnityToRobot = 6;
            gait_data[4]         = 0.0f; // 头部归正
            break;
        case 0x07: // 机械臂左转
            DataFromUnityToRobot = 7;
            break;
        case 0x08: // 机械臂右转
            DataFromUnityToRobot = 8;
            break;
        case 0x09: // 大臂上升
            DataFromUnityToRobot = 9;
            break;
        case 0x0A: // 大臂下降
            DataFromUnityToRobot = 10;
            break;
        case 0x0B: // 小臂上升
            DataFromUnityToRobot = 11;
            break;
        case 0x0C: // 小臂下降
            DataFromUnityToRobot = 12;
            break;
        case 0x0D: // 放松
            DataFromUnityToRobot = 13;
            break;
        case 0x0E: // 加紧
            DataFromUnityToRobot = 14;
            break;

        default:
            break;
    }
}

void GetCommandFromBlueTeeth(uint8_t data)
{
    if (data == 0x02) {
        UsefulCommand    = true;
        target_index     = 2;
        is_interpolating = false; // 重启插值，目标角度为全0
    }
    if (is_interpolating == false) {
        switch (data) {
            case 0x03: // 左腿前进
                UsefulCommand = true;
                target_index  = 0;
                break;
            case 0x04: // 右腿前进
                UsefulCommand = true;
                target_index  = 1;
                break;
            default:
                break;
        }
    }
}

void FillMotorData()
{
    for (int i = 0; i < 5; i++) {
        data_to_uint8(&gait_data[i], TxDataBufferFromUnity + i * sizeof(float), sizeof(float));
    }
}

void FillArmData()
{
    // 填充机械臂数据
    int AngleDataTemp[4] = {0};
    AngleDataTemp[0]     = 150 + 0.3 * (500 - Servo_Position[0]);
    if (AngleDataTemp[0] > 180) {
        AngleDataTemp[0] -= 360;
    }
    AngleDataTemp[1] = 0 + k1 * (Servo_Position[1] - 500);
    AngleDataTemp[2] = -AngleDataTemp[1]-8 + k2 * (500 - Servo_Position[2]);
    AngleDataTemp[3] = 30 - 0.06 * (Servo_Position[3] - 1500);
    ARM_DATA[0]      = -5;
    ARM_DATA[1]      = AngleDataTemp[0];
    ARM_DATA[2]      = AngleDataTemp[1];
    ARM_DATA[3]      = AngleDataTemp[2]; // 机械臂的底座大臂小臂由外部信号控制
    ARM_DATA[4]      = AngleDataTemp[3]; // 夹具也由外部信号控制
    for (int i = 0; i < 5; i++) {
        data_to_uint8(&ARM_DATA[i], TxDataBufferFromUnity + MotorDataLength + i * sizeof(int32_t), sizeof(int32_t));
    }
}
