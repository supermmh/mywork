/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "gom_protocol.h"
#include "stdio.h"
#include "stdint.h"
#include "RoboctControl.h"
#include "Connect.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RS485_RxMode() (HAL_GPIO_WritePin(RS485_REDE_GPIO_Port, RS485_REDE_Pin, GPIO_PIN_RESET))
#define RS485_TxMode() (HAL_GPIO_WritePin(RS485_REDE_GPIO_Port, RS485_REDE_Pin, GPIO_PIN_SET))
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t bluetooth_receive[2];
char message_zuoqian[20];
char message_youqian[20];
char message_zuohou[20];
char message_youhou[20];
char message_servo1[20];
char message_servo2[20];
char message_servo3[20];

MotorCmd_t cmd_zuoqian   = {0};
MotorCmd_t cmd_youqian   = {0};
MotorCmd_t cmd_zuohou    = {0};
MotorCmd_t cmd_youhou    = {0};
MotorCmd_t cmd_tou       = {0};
MotorData_t data_zuoqian = {0};
MotorData_t data_youqian = {0};
MotorData_t data_zuohou  = {0};
MotorData_t data_youhou  = {0};
MotorData_t data_tou     = {0};

HAL_StatusTypeDef tx_res;
HAL_StatusTypeDef rx_res;

char Leg_zuoqian[4];
char Leg_youqian[4];
char Leg_zuohou[4];
char Leg_youhou[4];
float bujin   = 5.5;
float trot_kp = 0.8;
float trot_kw = 0.2;

float four_let_angle_init[5];

uint8_t receive_Servo_Pos[10];
uint8_t transmit_head[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
int Servo_Position[4]    = {500, 500, 500, 1500};
int index_pos;

#pragma pack(push, 1) // ǿ��1�ֽڶ��룬�����ṹ�����
typedef struct {
    uint8_t header;   // ��ͷ 0xA5
    uint8_t type;     // ���ͱ�ʶ 0x1D/0x2D/0x3D
    float float_data; // �������ݣ�4�ֽڣ�
    uint8_t footer;   // ��β 0x5A
} DataPacket;
#pragma pack(pop) // �ָ�Ĭ�϶���

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Moter_Parameter_Set(MotorCmd_t *cmd, float T, float W, float Pos, float K_P, float K_W);
void Moter_Cmd_Send_Get(void);
void Show_Pos(void);
void Moter_Safe(void);
void Moter_Stay(void);
void receive_Servo_Data(void);
void Moter_Trot_First(void);
void Moter_Trot_Second(void);
void Moter_guizheng(void);
void send_packet(UART_HandleTypeDef *huart, uint8_t type, float data);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_UART5_Init();
  MX_TIM1_Init();
  MX_UART7_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
    // OLED_Init();
    // OLED_Clear();
    HAL_UART_Receive_IT(&huart5, bluetooth_receive, 1);
    HAL_UART_Receive_IT(&huart7, receive_Servo_Pos, 6);
    HAL_TIM_Base_Start_IT(&htim2);
    RS485_RxMode();
    HAL_Delay(1000);
    Moter_Safe();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1) {

        Moter_Cmd_Send_Get();
        // Show_Pos();
        // receive_Servo_Data();

        switch (DataFromUnityToRobot) { // Unity控制命令
            case 1:
                // 左腿
                Moter_Trot_First();
                DataFromUnityToRobot = 0;
                break;
            case 2:
                // 右腿动
                Moter_Trot_Second();
                DataFromUnityToRobot = 0;
                break;
            case 3:
                // 所有电机归正
                Moter_guizheng();
                DataFromUnityToRobot = 0;
                break;
            case 4:
                // 调整头部角度.左转
                Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4] - 2, 2, 0.15);
                DataFromUnityToRobot = 0;
                break;
            case 5:
                // 调整头部角度.右转
                Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4] + 2, 2, 0.15);
                DataFromUnityToRobot = 0;
                break;
            case 6:
                // 调整头部角度.归正
                Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4], 2, 0.1);
                DataFromUnityToRobot = 0;
                break;
            case 7:
                // 左转
                HAL_UART_Transmit(&huart7, &transmit_head[0], sizeof(transmit_head[0]), 10);
                Servo_Position[0] -= 50;
                DataFromUnityToRobot = 0;
                break;
            case 8:
                // 右转
                HAL_UART_Transmit(&huart7, &transmit_head[1], sizeof(transmit_head[1]), 10);
                Servo_Position[0] += 50;
                DataFromUnityToRobot = 0;
                break;
            case 9:
                // 大臂上升
                HAL_UART_Transmit(&huart7, &transmit_head[2], sizeof(transmit_head[2]), 10);
                Servo_Position[1] -= 50;
                DataFromUnityToRobot = 0;
                break;
            case 10:
                // 大臂下降
                HAL_UART_Transmit(&huart7, &transmit_head[3], sizeof(transmit_head[3]), 10);
                Servo_Position[1] += 50;
                DataFromUnityToRobot = 0;
                break;
            case 11:
                // 小臂上升
                HAL_UART_Transmit(&huart7, &transmit_head[4], sizeof(transmit_head[4]), 10);
                Servo_Position[2] -= 50;
                DataFromUnityToRobot = 0;
                break;
            case 12:
                // 小臂下降
                HAL_UART_Transmit(&huart7, &transmit_head[5], sizeof(transmit_head[5]), 10);
                Servo_Position[2] += 50;
                DataFromUnityToRobot = 0;
                break;
            case 13:
                // 放松
                HAL_UART_Transmit(&huart7, &transmit_head[6], sizeof(transmit_head[6]), 10);
                Servo_Position[3] -= 100;
                DataFromUnityToRobot = 0;
                break;
            case 14:
                // 加紧
                HAL_UART_Transmit(&huart7, &transmit_head[7], sizeof(transmit_head[7]), 10);
                Servo_Position[3] += 100;
                DataFromUnityToRobot = 0;
                break;
            default:
                DataFromUnityToRobot = 0;
                break;
        }
        switch (bluetooth_receive[0]) {
            case 0x01:
                Moter_Safe();
                bluetooth_receive[0] = 0;
                break;
            case 0x02:
                Moter_Stay();
                bluetooth_receive[0] = 0;
                break;
            case 0x03:
                Moter_Trot_First();
                bluetooth_receive[0] = 0;
                break;
            case 0x04:
                Moter_Trot_Second();
                bluetooth_receive[0] = 0;
                break;

            case 0x05:
                HAL_UART_Transmit(&huart7, &transmit_head[0], sizeof(transmit_head[0]), 10);
                Servo_Position[0] -= 50;
                bluetooth_receive[0] = 0;
                break;
            case 0x06:
                HAL_UART_Transmit(&huart7, &transmit_head[1], sizeof(transmit_head[1]), 10);
                Servo_Position[0] += 50;
                bluetooth_receive[0] = 0;
                break;
            case 0x07:
                HAL_UART_Transmit(&huart7, &transmit_head[2], sizeof(transmit_head[2]), 10);
                Servo_Position[1] -= 50;
                bluetooth_receive[0] = 0;
                break;
            case 0x08:
                HAL_UART_Transmit(&huart7, &transmit_head[3], sizeof(transmit_head[3]), 10);
                Servo_Position[1] += 50;
                bluetooth_receive[0] = 0;
                break;
            case 0x09:
                HAL_UART_Transmit(&huart7, &transmit_head[4], sizeof(transmit_head[4]), 10);
                Servo_Position[2] -= 50;
                bluetooth_receive[0] = 0;
                break;
            case 0x010:
                HAL_UART_Transmit(&huart7, &transmit_head[5], sizeof(transmit_head[5]), 10);
                Servo_Position[2] += 50;
                bluetooth_receive[0] = 0;
                break;
            case 0x011:
                HAL_UART_Transmit(&huart7, &transmit_head[6], sizeof(transmit_head[6]), 10);
                Servo_Position[3] -= 100;
                bluetooth_receive[0] = 0;
                break;
            case 0x012:
                HAL_UART_Transmit(&huart7, &transmit_head[7], sizeof(transmit_head[7]), 10);
                Servo_Position[3] += 100;
                bluetooth_receive[0] = 0;
                break;
            case 0x13:
                Moter_guizheng();
                break;
            case 0x14:
                Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4] - 2, 2, 0.15);
				gait_data[4] = -20.0f;
                bluetooth_receive[0] = 0;
                break;
            case 0x15:
                Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4] + 2, 2, 0.15);
				gait_data[4] = 20.0f;
                bluetooth_receive[0] = 0;
                break;
            case 0x16:
                Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4], 2, 0.1);
				gait_data[4] = 0.0f;
                bluetooth_receive[0] = 0;
                break;
            default:
                bluetooth_receive[0] = 0;
                break;
        }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // HAL_UART_Receive_IT(&huart5,bluetooth_receive,1);
    // receive_Servo_Data();
    if (huart == &huart5) {
        GetCommandFromBlueTeeth(bluetooth_receive[0]);
        if (UsefulCommand) {
            InterpolateGaitData(); // 计算腿部插值数据
        }
        HAL_UART_Receive_IT(&huart5, bluetooth_receive, 1);
    }
    if (huart == &huart7) {
        receive_Servo_Data();
        HAL_UART_Receive_IT(&huart7, receive_Servo_Pos, 6);
    }
}

void Moter_Parameter_Set(MotorCmd_t *cmd, float T, float W, float Pos, float K_P, float K_W)
{
    cmd->T   = T;
    cmd->W   = W;
    cmd->Pos = Pos;
    cmd->K_P = K_P;
    cmd->K_W = K_W;
}

void Moter_Cmd_Send_Get()
{
    modify_data(&cmd_zuoqian);
    RS485_TxMode();
    tx_res = HAL_UART_Transmit(&huart1, (uint8_t *)&cmd_zuoqian.motor_send_data, sizeof(cmd_zuoqian.motor_send_data), 1);
    RS485_RxMode();
    rx_res = HAL_UART_Receive(&huart1, (uint8_t *)&data_zuoqian.motor_recv_data, sizeof(data_zuoqian.motor_recv_data), 1);
    extract_data(&data_zuoqian);
    HAL_Delay(1);

    modify_data(&cmd_youqian);
    RS485_TxMode();
    tx_res = HAL_UART_Transmit(&huart1, (uint8_t *)&cmd_youqian.motor_send_data, sizeof(cmd_youqian.motor_send_data), 1);
    RS485_RxMode();
    rx_res = HAL_UART_Receive(&huart1, (uint8_t *)&data_youqian.motor_recv_data, sizeof(data_youqian.motor_recv_data), 1);
    extract_data(&data_youqian);
    HAL_Delay(1);

    modify_data(&cmd_zuohou);
    RS485_TxMode();
    tx_res = HAL_UART_Transmit(&huart1, (uint8_t *)&cmd_zuohou.motor_send_data, sizeof(cmd_zuohou.motor_send_data), 1);
    RS485_RxMode();
    rx_res = HAL_UART_Receive(&huart1, (uint8_t *)&data_zuohou.motor_recv_data, sizeof(data_zuohou.motor_recv_data), 1);
    extract_data(&data_zuohou);
    HAL_Delay(1);

    modify_data(&cmd_youhou);
    RS485_TxMode();
    tx_res = HAL_UART_Transmit(&huart1, (uint8_t *)&cmd_youhou.motor_send_data, sizeof(cmd_youhou.motor_send_data), 1);
    RS485_RxMode();
    rx_res = HAL_UART_Receive(&huart1, (uint8_t *)&data_youhou.motor_recv_data, sizeof(data_youhou.motor_recv_data), 1);
    extract_data(&data_youhou);
    HAL_Delay(1);

    modify_data(&cmd_tou);
    RS485_TxMode();
    tx_res = HAL_UART_Transmit(&huart1, (uint8_t *)&cmd_tou.motor_send_data, sizeof(cmd_tou.motor_send_data), 1);
    RS485_RxMode();
    rx_res = HAL_UART_Receive(&huart1, (uint8_t *)&data_tou.motor_recv_data, sizeof(data_tou.motor_recv_data), 1);
    extract_data(&data_tou);
    HAL_Delay(1);
}

void Show_Pos()
{
    sprintf(message_zuoqian, "LF:%.2f", data_zuoqian.Pos);
    OLED_ShowString(1, 1, (uint8_t *)message_zuoqian, sizeof(message_zuoqian));

    sprintf(message_youqian, "RF:%.2f", data_youqian.Pos);
    OLED_ShowString(1, 2, (uint8_t *)message_youqian, sizeof(message_youqian));

    sprintf(message_zuohou, "RF:%.2f", data_zuohou.Pos);
    OLED_ShowString(1, 3, (uint8_t *)message_zuohou, sizeof(message_zuohou));

    sprintf(message_youhou, "RF:%.2f", data_youhou.Pos);
    OLED_ShowString(1, 4, (uint8_t *)message_youhou, sizeof(message_youhou));

    sprintf(message_servo1, "Ser1:%d", Servo_Position[0]);
    OLED_ShowString(1, 5, (uint8_t *)message_servo1, sizeof(message_servo1));

    sprintf(message_servo2, "Ser2:%d", Servo_Position[1]);
    OLED_ShowString(1, 6, (uint8_t *)message_servo2, sizeof(message_servo2));

    sprintf(message_servo3, "Ser3:%d", Servo_Position[2]);
    OLED_ShowString(1, 7, (uint8_t *)message_servo3, sizeof(message_servo3));
}

void Moter_Safe()
{
    cmd_zuoqian.id   = 0;
    cmd_zuoqian.mode = 1;
    Moter_Parameter_Set(&cmd_zuoqian, 0, 0, data_zuoqian.Pos, 0., 0);

    cmd_youqian.id   = 2;
    cmd_youqian.mode = 1;
    Moter_Parameter_Set(&cmd_youqian, 0, 0, data_youqian.Pos, 0, 0);

    cmd_zuohou.id   = 3;
    cmd_zuohou.mode = 1;
    Moter_Parameter_Set(&cmd_zuohou, 0, 0, data_youhou.Pos, 0, 0);

    cmd_youhou.id   = 4;
    cmd_youhou.mode = 1;
    Moter_Parameter_Set(&cmd_youhou, 0, 0, data_youhou.Pos, 0, 0);

    cmd_tou.id   = 1;
    cmd_tou.mode = 1;
    Moter_Parameter_Set(&cmd_tou, 0, 0, data_tou.Pos, 0, 0);

    Moter_Cmd_Send_Get();
}

void Moter_Stay()
{
    Moter_Parameter_Set(&cmd_zuoqian, 0, 0, data_zuoqian.Pos, 1.2, 0.15);

    Moter_Parameter_Set(&cmd_youqian, 0, 0, data_youqian.Pos, 1.2, 0.15);

    Moter_Parameter_Set(&cmd_zuohou, 0, 0, data_zuohou.Pos, 1.2, 0.15);

    Moter_Parameter_Set(&cmd_youhou, 0, 0, data_youhou.Pos, 1.2, 0.15);

    Moter_Parameter_Set(&cmd_tou, 0, 0, data_tou.Pos, 1.2, 0.15);

    Moter_Cmd_Send_Get();
    four_let_angle_init[0] = data_zuoqian.Pos;
    four_let_angle_init[1] = data_youqian.Pos;
    four_let_angle_init[2] = data_zuohou.Pos;
    four_let_angle_init[3] = data_youhou.Pos;
    four_let_angle_init[4] = data_tou.Pos;
}

void Moter_Trot_First()
{
    Moter_Parameter_Set(&cmd_zuoqian, 0, 0, four_let_angle_init[0] + bujin, trot_kp, trot_kw);

    Moter_Parameter_Set(&cmd_youqian, 0, 0, four_let_angle_init[1] + bujin - 1, trot_kp, trot_kw);

    Moter_Parameter_Set(&cmd_zuohou, 0, 0, four_let_angle_init[2] - bujin, trot_kp, trot_kw);

    Moter_Parameter_Set(&cmd_youhou, 0, 0, four_let_angle_init[3] - bujin, trot_kp, trot_kw);
}

void Moter_Trot_Second()
{
    Moter_Parameter_Set(&cmd_zuoqian, 0, 0, four_let_angle_init[0] - bujin, trot_kp, trot_kw);

    Moter_Parameter_Set(&cmd_youqian, 0, 0, four_let_angle_init[1] - bujin, trot_kp, trot_kw);

    Moter_Parameter_Set(&cmd_zuohou, 0, 0, four_let_angle_init[2] + bujin, trot_kp, trot_kw);

    Moter_Parameter_Set(&cmd_youhou, 0, 0, four_let_angle_init[3] + bujin, trot_kp, trot_kw);
}

void Moter_guizheng()
{
    Moter_Parameter_Set(&cmd_zuoqian, 0, 0, four_let_angle_init[0], 2, 0.15);

    Moter_Parameter_Set(&cmd_youqian, 0, 0, four_let_angle_init[1], 2, 0.15);

    Moter_Parameter_Set(&cmd_zuohou, 0, 0, four_let_angle_init[2], 2, 0.15);

    Moter_Parameter_Set(&cmd_youhou, 0, 0, four_let_angle_init[3], 2, 0.15);

    Moter_Parameter_Set(&cmd_tou, 0, 0, four_let_angle_init[4], 2, 0.15);
}

void receive_Servo_Data()
{
    if (receive_Servo_Pos[0] == 0xAA && receive_Servo_Pos[5] == 0x55) {
        Servo_Position[0] = (receive_Servo_Pos[1] << 24) | (receive_Servo_Pos[2] << 16) | (receive_Servo_Pos[3] << 8) | receive_Servo_Pos[4];
        return;
    } else if (receive_Servo_Pos[0] == 0xBB && receive_Servo_Pos[5] == 0x55) {
        Servo_Position[1] = (receive_Servo_Pos[1] << 24) | (receive_Servo_Pos[2] << 16) | (receive_Servo_Pos[3] << 8) | receive_Servo_Pos[4];

    } else if (receive_Servo_Pos[0] == 0xCC && receive_Servo_Pos[5] == 0x55) {
        Servo_Position[2] = (receive_Servo_Pos[1] << 24) | (receive_Servo_Pos[2] << 16) | (receive_Servo_Pos[3] << 8) | receive_Servo_Pos[4];
    }
    for (int i = 0; i < 10; i++) {
        receive_Servo_Pos[i] = 0;
    }
}

void send_packet(UART_HandleTypeDef *huart, uint8_t type, float data)
{
    DataPacket packet;
    packet.header     = 0xA5; // �̶���ͷ
    packet.type       = type; // ָ������
    packet.float_data = data; // ��両������
    packet.footer     = 0x5A; // �̶���β

    // �����������ݰ�
    HAL_UART_Transmit(huart, (uint8_t *)&packet, sizeof(DataPacket), HAL_MAX_DELAY);
}

/* USER CODE END 4 */

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {
    }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
