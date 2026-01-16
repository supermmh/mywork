#ifndef _CONNECT_H
#define _CONNECT_H

#include "stdint.h"
#include "string.h"
#include "tim.h"
#include "main.h"
#include "spi.h"
#include "stdbool.h"


#define ARM_BODY_SIZE          20 // 机械臂数据包正文长度

#define MotorDataLength        20                                // 电机数据长度
#define UartTransmitDataLength (MotorDataLength + ARM_BODY_SIZE) // 发送数据长度

extern uint8_t TxDataBufferFromUnity[UartTransmitDataLength]; // SPI发送数据缓冲区
extern uint8_t RXDataBufferFromUnity[UartTransmitDataLength]; // SPI接收数据缓冲区

extern uint8_t getdata_flag; // 测试
extern int testtimes;        // 测试

extern bool UsefulCommand;

extern int DataFromUnityToRobot;


void delay_us(uint16_t us);
void data_to_uint8(void *input, uint8_t *output, size_t input_size);
void DataPutin(); // 填充电机数据给TxDataBufferFromUnity，下标从0开始

void FillMotorData();
void FillArmData();

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void SPI_TransmitReceive_NSS(); // 手动拉低NSS启动SPI通信阻塞式通信,发送数据在调用该函数之前或者在该函数中填充，接收数据在通信完毕后处理
void GetCommandFromUnity(uint8_t data);
void GetCommandFromBlueTeeth(uint8_t data);
int16_t uint8_to_int16(uint8_t input[2]); // 暂时不需要，后续可以改进

//void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
//void CheckHeaderAndReceive(uint8_t *buffer, size_t length, uint8_t *header, size_t header_length, uint8_t *processed_buffer, size_t body_length); // 遍历缓冲区，读取正文，处理正文


#endif // !_CONNECT_H