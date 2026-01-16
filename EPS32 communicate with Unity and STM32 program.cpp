#include <Arduino.h>
#include <WiFi.h>
#include <HardwareSerial.h>
#include <driver/spi_slave.h>  // 使用 ESP32 的 SPI 从机库
#include <driver/spi_common.h> // 包含 SPI 常量定义
#include "esp_wifi.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_task_wdt.h" // 添加头文件

// 使用 HSPI_HOST（SPI2）作为 SPI 外设
#ifndef HSPI_HOST
#define HSPI_HOST SPI2_HOST // 定义 HSPI_HOST 为 SPI2_HOST
#endif

const char *ssid = "TEST_WIFI";
const char *password = "123456789";

WiFiServer server(8080);

#define MOTOR_DATA_NUM 5
#define ARM_DATA_NUM 5

// 确保 TotalDataLength 向上对齐到 4 的倍数
#define TotalDataLength ((MOTOR_DATA_NUM * sizeof(float) + ARM_DATA_NUM * sizeof(int32_t) + 3) & ~3)
#define tcpReceiveDataLength sizeof(uint8_t)

#define SPI_SCK 4
#define SPI_MISO 5
#define SPI_MOSI 6
#define SPI_SS 7

uint8_t *spiTxBuffer = nullptr; // 发送缓冲区
uint8_t *spiRxBuffer = nullptr; // 接收缓冲区
uint8_t lastReceivedData[TotalDataLength] = {0};
spi_slave_transaction_t newTrans = {

    .length = TotalDataLength * 8,          // 数据长度（以位为单位）
    .tx_buffer = (const void *)spiTxBuffer, // 指定发送缓冲区
    .rx_buffer = (void *)spiRxBuffer,       // 指定接收缓冲区

};

uint8_t *tcpSendDataBuffer = nullptr;
uint8_t *tcpReceiveDataBuffer = nullptr;
uint16_t header = 0x1E8A;
uint16_t footer = 0x2F9B;
bool isLittleEndianSystem = false;
volatile bool GetUnityDataFlag = false; // 添加 volatile 关键字

void swapEndian(void *data, size_t length);
void checkEndian();
bool TCPSendData(WiFiClient client, size_t length); // 修改返回类型为 bool
bool TCPReceiveData(WiFiClient client, void *data, size_t length); // 修改返回类型为 bool
void spiTransactionCallback(spi_slave_transaction_t *trans);

void setup()
{
    Serial.begin(9600);
    checkEndian();
    // 检查重启原因
    esp_reset_reason_t reason = esp_reset_reason();
    Serial.printf("Reset reason: %d\n", reason);

    // 分配支持 DMA 的内存
    spiTxBuffer = (uint8_t *)heap_caps_malloc(TotalDataLength, MALLOC_CAP_DMA);
    spiRxBuffer = (uint8_t *)heap_caps_malloc(TotalDataLength, MALLOC_CAP_DMA);

    if (!spiTxBuffer || !spiRxBuffer)
    {
        Serial.println("Failed to allocate DMA-capable memory for SPI buffers");
        while (1)
            ; // 停止程序运行
    }

    tcpSendDataBuffer = new uint8_t[TotalDataLength + sizeof(header) + sizeof(footer)];
    tcpReceiveDataBuffer = new uint8_t[tcpReceiveDataLength];

    memset(spiTxBuffer, 0, TotalDataLength);
    memset(spiRxBuffer, 0, TotalDataLength);
    memset(tcpSendDataBuffer, 0, TotalDataLength + sizeof(header) + sizeof(footer));
    memset(tcpReceiveDataBuffer, 0, tcpReceiveDataLength);

    WiFi.softAP(ssid, password);
    server.begin();

    // 配置 SPI 从机模式
    spi_bus_config_t buscfg = {
        .mosi_io_num = SPI_MOSI,
        .miso_io_num = SPI_MISO,
        .sclk_io_num = SPI_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = TotalDataLength};
    spi_slave_interface_config_t slvcfg = {
        .spics_io_num = SPI_SS,
        .flags = 0,
        .queue_size = 1, // 只需要一个事务队列
        .mode = 0,
        .post_setup_cb = NULL,
        .post_trans_cb = spiTransactionCallback};

    // 初始化 SPI 从机
    esp_err_t ret = spi_slave_initialize(HSPI_HOST, &buscfg, &slvcfg, SPI_DMA_CH_AUTO); // 自动分配 DMA 通道
    if (ret != ESP_OK)
    {
        Serial.printf("Failed to initialize SPI slave: %s\n", esp_err_to_name(ret));
        while (1)
            ; // 停止程序运行
    }
    else
    {
        Serial.printf("Successfully initialize SPI\n");
    }

    spi_slave_transaction_t initialTrans = {};
    initialTrans.length = TotalDataLength * 8;          // 数据长度（以位为单位）
    initialTrans.tx_buffer = (const void *)spiTxBuffer; // 指定发送缓冲区
    initialTrans.rx_buffer = (void *)spiRxBuffer;       // 指定接收缓冲区
    ret = spi_slave_queue_trans(HSPI_HOST, &initialTrans, portMAX_DELAY);

    newTrans.tx_buffer = (const void *)spiTxBuffer;
    newTrans.rx_buffer = (void *)spiRxBuffer;

    if (ret != ESP_OK)
    {
        Serial.printf("Failed to queue SPI transaction: %s\n", esp_err_to_name(ret));
        while (1)
            ; // 停止程序运行
    }
    else
    {
        Serial.printf("Successfully queue SPI transaction\n");
    }
}

void loop()
{
    WiFiClient client = server.available();

    if (client)
    {
        Serial.write("\nconnected");

        while (client.connected())
        {
            static unsigned long lastExecutionTime = 0; // 上次执行的时间戳
            unsigned long currentTime = millis();       // 当前时间

            // 每 10ms 执行一次
            if (currentTime - lastExecutionTime >= 10)
            {
                lastExecutionTime = currentTime;

                // 接收 TCP 客户端的数据
                bool receiveSuccess = true; // 标志接收是否成功
                if (client.available())
                {
                    // 检查 TCPReceiveData 的返回值
                    if (TCPReceiveData(client, tcpReceiveDataBuffer, tcpReceiveDataLength))
                    {
                         Serial.write("get data!\n");
                         GetUnityDataFlag = true;
                    }
                    else
                    {
                        // 接收失败，可能客户端已断开
                        Serial.write("\nTCP Receive Failed, client likely disconnected.");
                        receiveSuccess = false; // 标记接收失败
                    }
                }
                else
                {

                    if (spiTxBuffer) // 添加 null 检查以防万一
                    {
                        spiTxBuffer[0] = 0xFF;
                    }
                }

                // 如果接收失败，则跳过发送并退出循环
                if (!receiveSuccess)
                {
                    break;
                }

                // 将接收到的数据发送到 TCP 客户端
                // 检查 TCPSendData 的返回值
                if (!TCPSendData(client, TotalDataLength))
                {
                    // 发送失败，客户端可能已断开
                    Serial.write("\nTCP Send Failed, client likely disconnected.");
                    break; // 退出循环
                }
            }

            // 检查客户端是否断开连接 (在每次循环迭代结束时检查)
            if (!client.connected())
            {
                Serial.write("\ndisconnect");
                break; // 退出循环
            }
        }
        // 在客户端断开后显式停止客户端，释放资源
        Serial.write("\nClient disconnected.");
        client.stop();
        Serial.write("\nClient stopped.");
    }
}

void spiTransactionCallback(spi_slave_transaction_t *trans)
{
    // 确保 spiRxBuffer 和 tcpSendDataBuffer 有效
    if (spiRxBuffer && tcpSendDataBuffer)
    {
        memcpy(tcpSendDataBuffer + sizeof(header), spiRxBuffer, TotalDataLength);
    }
    if (spiTxBuffer) // 添加 null 检查以防万一
    {
        memset(spiTxBuffer, 0xFF, TotalDataLength); // 填充 0xFF 或根据需要更新数据
    }

    if (GetUnityDataFlag)
    {
        if (spiTxBuffer && tcpReceiveDataBuffer) // 添加 null 检查以防万一
        {
             spiTxBuffer[0] = tcpReceiveDataBuffer[0];
        }
        GetUnityDataFlag = false;
    }
    Serial.write("\nGet Data from SPI\n");

    // 提交新的事务
    // 确保 newTrans 的 tx_buffer 和 rx_buffer 指向有效的 DMA 内存
    newTrans.tx_buffer = (const void *)spiTxBuffer;
    newTrans.rx_buffer = (void *)spiRxBuffer;
    esp_err_t ret = spi_slave_queue_trans(HSPI_HOST, &newTrans, portMAX_DELAY);
     if (ret != ESP_OK)
    {
        Serial.printf("Failed to queue SPI transaction in callback: %s\n", esp_err_to_name(ret));
        // 错误处理，可能需要重置 SPI 或重启
    }
}

bool TCPSendData(WiFiClient client, size_t length)
{
    if (!client || !client.connected()) return false; // 检查客户端是否有效和连接

    memcpy(tcpSendDataBuffer, &header, sizeof(header));
    memcpy(tcpSendDataBuffer + sizeof(header) + length, &footer, sizeof(footer));

    if (!isLittleEndianSystem)
    {
        swapEndian(tcpSendDataBuffer, sizeof(header));
        swapEndian(tcpSendDataBuffer + sizeof(header), length);
        swapEndian(tcpSendDataBuffer + sizeof(header) + length, sizeof(footer));
    }

    // 检查 write 的返回值
    size_t bytesWritten = client.write(tcpSendDataBuffer, length + sizeof(header) + sizeof(footer));

    // 如果写入的字节数小于期望值，认为发送失败
    if (bytesWritten != length + sizeof(header) + sizeof(footer))
    {
        Serial.printf("Partial TCP write: wrote %zu of %zu bytes\n", bytesWritten, length + sizeof(header) + sizeof(footer));
        return false; // 发送失败
    }

    return true; // 发送成功
}

bool TCPReceiveData(WiFiClient client, void *data, size_t length)
{
    if (!client || !client.connected()) return false; // 检查客户端是否有效和连接

    uint8_t *buffer = static_cast<uint8_t *>(data);
    // readBytes 会阻塞直到读取到 length 字节或连接关闭
    size_t bytesRead = client.readBytes(buffer, length);

    // 如果读取的字节数小于期望值，认为接收失败或连接关闭
    if (bytesRead != length)
    {
        Serial.printf("Partial TCP read: read %zu of %zu bytes\n", bytesRead, length);
        return false; // 接收失败
    }

    if (!isLittleEndianSystem)
    {
        swapEndian(buffer, length);
    }
    return true; // 接收成功
}


void checkEndian()
{
    uint16_t test = 0x1234;
    uint8_t *p = (uint8_t *)&test;
    isLittleEndianSystem = (p[0] == 0x34);
}

void swapEndian(void *data, size_t length)
{
    uint8_t *byteData = static_cast<uint8_t *>(data);
    for (size_t i = 0; i < length / 2; i++)
    {
        uint8_t temp = byteData[i];
        byteData[i] = byteData[length - 1 - i];
        byteData[length - 1 - i] = temp;
    }
}