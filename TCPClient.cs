using System;
using System.Net.Sockets;
using System.Text;
using System.Collections;
using UnityEngine;
using System.Collections.Generic; // 需要使用 List 作为缓冲区

public class TCPClient : MonoBehaviour
{
    private TcpClient client;
    private NetworkStream stream;
    public bool isConnected = false;

    public const string IP_location = "192.168.4.1";
    public const int IP_port = 8080;
    public string serverIP = IP_location; // 替换为实际的服务器IP
    public int serverPort = IP_port; // 替换为实际的服务器端口

    private const int RECEIVE_DATA_LENGTH = 44; // 2 字节包头 + 5 * 4 字节电机数据 + 5 * 4 字节 Jetson 数据 + 2 字节包尾 = 44
    private const int SEND_DATA_LENGTH = 1; // 发送数据长度为 1 字节
    public const int MOTOR_DATA_ARRAY_LENGTH = 5; // 电机数据数组长度改为 5
    public const int JETSON_DATA_ARRAY_LENGTH = 5; // Jetson 数据数组长度保持 5
    private const int HEADER_SIZE = 2; // 包头大小
    private const int FOOTER_SIZE = 2; // 包尾大小
    private const int MOTOR_DATA_SIZE = 4; // 每个电机数据的大小 (float 是 4 字节)
    private const int JETSON_DATA_SIZE = 4; // 每个 Jetson 数据的大小 (int 是 4 字节)
    private const int MOTOR_DATA_OFFSET = HEADER_SIZE; // 电机数据的起始偏移量
    private const int JETSON_DATA_OFFSET = MOTOR_DATA_OFFSET + MOTOR_DATA_ARRAY_LENGTH * MOTOR_DATA_SIZE; // Jetson 数据的起始偏移量改为 2 + 5 * 4 = 22
    private const ushort HEADER_VALUE = 0x1E8A; // 包头值
    private const ushort FOOTER_VALUE = 0x2F9B; // 包尾值
    private const byte DEFAULT_DATA_TO_SEND = 0xF0; // 默认发送数据

    // private byte[] receivedData = new byte[RECEIVE_DATA_LENGTH]; // 不再直接使用固定大小数组接收
    private List<byte> receiveBuffer = new List<byte>(); // 使用 List 作为接收缓冲区
    private byte[] sendDataBuffer = new byte[SEND_DATA_LENGTH];

    private MotorData[] motorDataArray = new MotorData[MOTOR_DATA_ARRAY_LENGTH]; // 存储接收到的电机数据，长度已更新
    private int[] jetson = new int[JETSON_DATA_ARRAY_LENGTH]; // 存储接收到的 Jetson 数据，长度已更新
    private object dataLock = new object(); // 用于线程同步

    private byte dataToSend = DEFAULT_DATA_TO_SEND; // 默认发送数据

    private float lastLogTime = 0f; // 上次记录日志的时间
    private int receivedPacketCount = 0; // 记录接收到的数据包数量
    private int sentPacketCount = 0; // 记录发出的指令数量
    private int lowPacketCount = 0; // 记录每秒收到的包少于10个的次数
    private int totalSeconds = 0; // 记录总的秒数

    public struct MotorData
    {
        public float angle;

        public override string ToString()
        {
            return $"Angle: {angle}";
        }
    }

    public MotorData[] GetMotorData()
    {
        lock (dataLock)
        {
            return (MotorData[])motorDataArray.Clone();
        }
    }

    public int[] GetJetsonData()
    {
        lock (dataLock)
        {
            return (int[])jetson.Clone();
        }
    }

    public void SetDataToSend(byte data)
    {
        dataToSend = data;
    }

    void Start()
    {
        // 设置游戏帧率
        Time.fixedDeltaTime = 1.0f / 75.0f;
        StartCoroutine(ConnectToServer());
    }

    IEnumerator ConnectToServer()
    {
        try
        {
            client = new TcpClient(serverIP, serverPort);
            client.NoDelay = true; // 禁用 Nagle 算法，减少发送延迟
            stream = client.GetStream();
            isConnected = true;
            Debug.Log("Connected to server");

            // 开始接收和发送协程
            StartCoroutine(ReceiveData());
            StartCoroutine(SendData());
        }
        catch (Exception e)
        {
            Debug.LogError("连接服务器失败: " + e.Message);
        }

        yield return null;
    }

    IEnumerator ReceiveData()
    {
        byte[] tempBuffer = new byte[1024]; // 用于从流中临时读取数据的小缓冲区

        while (isConnected)
        {
            try
            {
                // 从流中读取所有可用数据到临时缓冲区
                if (stream.DataAvailable)
                {
                    int bytesRead = stream.Read(tempBuffer, 0, tempBuffer.Length);
                    if (bytesRead > 0)
                    {
                        // 将读取到的数据添加到接收缓冲区
                        lock (receiveBuffer)
                        {
                            for (int i = 0; i < bytesRead; i++)
                            {
                                receiveBuffer.Add(tempBuffer[i]);
                            }
                        }
                    }
                }

                // 解析缓冲区中的数据包
                lock (receiveBuffer)
                {
                    while (receiveBuffer.Count >= RECEIVE_DATA_LENGTH)
                    {
                        // 查找包头
                        int headerIndex = -1;
                        for (int i = 0; i <= receiveBuffer.Count - HEADER_SIZE; i++)
                        {
                            ushort header = BitConverter.ToUInt16(receiveBuffer.ToArray(), i);
                            if (header == HEADER_VALUE)
                            {
                                headerIndex = i;
                                break;
                            }
                        }

                        if (headerIndex != -1)
                        {
                            // 如果找到包头，检查是否有足够的数据构成一个完整包
                            if (receiveBuffer.Count - headerIndex >= RECEIVE_DATA_LENGTH)
                            {
                                // 检查包尾
                                int footerIndex = headerIndex + RECEIVE_DATA_LENGTH - FOOTER_SIZE;
                                ushort footer = BitConverter.ToUInt16(receiveBuffer.ToArray(), footerIndex);

                                if (footer == FOOTER_VALUE)
                                {
                                    // 找到一个完整且有效的包
                                    byte[] packet = receiveBuffer.GetRange(headerIndex, RECEIVE_DATA_LENGTH).ToArray();

                                    lock (dataLock)
                                    {
                                        // 解析电机数据部分 (5个 float)
                                        for (int i = 0; i < motorDataArray.Length; i++)
                                        {
                                            motorDataArray[i].angle = BitConverter.ToSingle(packet, MOTOR_DATA_OFFSET + i * sizeof(float));
                                        }

                                        // 解析 Jetson 数据部分 (5个 int)
                                        for (int i = 0; i < jetson.Length; i++)
                                        {
                                            jetson[i] = BitConverter.ToInt32(packet, JETSON_DATA_OFFSET + i * sizeof(int));
                                        }

                                        receivedPacketCount++; // 增加接收到的数据包计数
                                    }

                                    // 从缓冲区中移除已处理的数据包及其之前的所有数据
                                    receiveBuffer.RemoveRange(0, headerIndex + RECEIVE_DATA_LENGTH);
                                }
                                else
                                {
                                    // 包尾不匹配，可能是无效数据或同步错误，丢弃到下一个包头
                                    Debug.LogWarning("Invalid packet footer, discarding data up to next header.");
                                    receiveBuffer.RemoveRange(0, headerIndex + HEADER_SIZE); // 丢弃包头及之前的数据
                                }
                            }
                            else
                            {
                                // 数据不足以构成一个完整包，等待更多数据
                                break;
                            }
                        }
                        else
                        {
                            // 没有找到包头，丢弃缓冲区中的所有数据（或者可以更保守地只丢弃一部分）
                            // 为了防止缓冲区无限增长，如果长时间没有找到包头，可以考虑清空缓冲区
                            if (receiveBuffer.Count > RECEIVE_DATA_LENGTH * 2) // 例如，如果缓冲区大小超过两个包长
                            {
                                Debug.LogWarning("No header found in buffer, clearing buffer to prevent overflow.");
                                receiveBuffer.Clear();
                            }
                            else
                            {
                                // 缓冲区数据不足或没有找到包头，等待更多数据
                                break;
                            }
                        }
                    }
                }


                // 每秒记录一次日志
                if (Time.time - lastLogTime >= 1f)
                {
                    totalSeconds++; // 增加总的秒数
                    if (receivedPacketCount < 10)
                    {
                        lowPacketCount++; // 增加每秒收到的包少于10个的次数
                    }

                    float lowPacketRatio = (float)lowPacketCount / totalSeconds;
                    Debug.Log($"Ratio of seconds with less than 10 packets: {lowPacketRatio:P2}");
                    Debug.Log($"Received {receivedPacketCount} packets in the last second. Buffer size: {receiveBuffer.Count}");


                    receivedPacketCount = 0; // 重置接收到的数据包计数器
                    sentPacketCount = 0; // 重置发出的指令计数器
                    lastLogTime = Time.time;
                }
            }
            catch (Exception e)
            {
                Debug.LogError("通信错误: " + e.Message);
                isConnected = false;
            }

            yield return null; // 每一帧都检查和处理数据
        }
    }

    IEnumerator SendData()
    {
        while (isConnected)
        {
            try
            {
                // 发送数据
                sendDataBuffer[0] = dataToSend;
                stream.Write(sendDataBuffer, 0, SEND_DATA_LENGTH);
                sentPacketCount++; // 增加发出的指令计数

                // 恢复默认数据
                dataToSend = DEFAULT_DATA_TO_SEND;
            }
            catch (Exception e)
            {
                Debug.LogError("通信错误: " + e.Message);
                isConnected = false;
            }

            yield return new WaitForSeconds(0.25f); // 每秒发送4次
        }
    }

    void OnApplicationQuit()
    {
        isConnected = false; // 设置为false以退出协程循环

        if (stream != null)
        {
            stream.Close();
        }
        if (client != null)
        {
            client.Close();
        }
    }
}
