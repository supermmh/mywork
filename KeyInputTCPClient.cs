using UnityEngine;

public class ButtonEventHandler : MonoBehaviour
{

    private int[] JetData = new int[5]; // 新的数组，用于存储数据，位移，底盘，大臂，小臂，夹具
    private TCPClient tcpClient;
    private RobotArmController robotArmController;
    void Start()
    {
        tcpClient = FindObjectOfType<TCPClient>();
        robotArmController = FindObjectOfType<RobotArmController>();
        for (int i = 0; i < 5; i++)
        {
            JetData[i] = 0;
        }

    }

    public void LeftForward()
    {
        if (tcpClient != null)
        {   
            tcpClient.SetDataToSend(0x01); // 示例数据
            // Debug.Log("Forward pressed, data sent: 0x01");
        }

    }

    public void RightForward()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x02); // 示例数据
            // Debug.Log("Backward pressed, data sent: 0x02");
        }
    }

    public void Reload()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x03); // 示例数据
            // Debug.Log("Forward pressed, data sent: 0x03");
        }
    }


    public void HeadTurnLeft()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x04); // 示例数据
            // Debug.Log("Forward pressed, data sent: 0x04");
        }
    }
    public void HeadTurnRight()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x05); // 示例数据
            // Debug.Log("Forward pressed, data sent: 0x05");
        }
    }
    public void HeadTurnBack()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x06); // 示例数据
            // Debug.Log("Forward pressed, data sent: 0x06");
        }
    }
    public void ArmRotateLeft()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x07); // 示例数据
            // Debug.Log("Arm Rotate Left pressed, data sent: 0x07");
        }

    }

    public void ArmRotateRight()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x08); // 示例数据
            // Debug.Log("Arm Rotate Right pressed, data sent: 0x08");
        }

    }

    public void BigArmUp()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x09); // 示例数据
            // Debug.Log("Arm Up pressed, data sent: 0x09");
        }

    }

    public void BigArmDown()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x0A); // 示例数据
            // Debug.Log("Arm Down pressed, data sent: 0x0A");
        }

    }

    public void SmallArmUp()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x0B); // 示例数据
            // Debug.Log("Arm Up pressed, data sent: 0x0B");
        }
    }

    public void SmallArmDown()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x0C); // 示例数据
            // Debug.Log("Arm Down pressed, data sent: 0x0C");
        }
    }
    public void Release()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x0D); // 示例数据
            // Debug.Log("Release pressed, data sent: 0x0D");
        }
    }

    public void Grab()
    {
        if (tcpClient != null)
        {
            tcpClient.SetDataToSend(0x0E); // 示例数据
            // Debug.Log("Grab pressed, data sent: 0x0E");
        }
    }



    // 可以根据需要添加更多按键事件函数
}
