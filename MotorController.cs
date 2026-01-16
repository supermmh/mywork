using UnityEngine;
using System.Collections.Generic; // 确保包含 List

public class MotorController : MonoBehaviour
{
    private TCPClient tcpClient;
    public GameObject[] motors; // 绑定的电机数组,分别为左前，左后，右前，右后，头部，电机的hingejoint只有一个，可以直接选中

    [Header("Additional Hinges Controlled by Motor 5")]
    [Tooltip("拖拽第一个需要跟随第五个电机转动的新铰链的GameObject")]
    public GameObject additionalHinge1;
    [Tooltip("拖拽第二个需要跟随第五个电机转动的新铰链的GameObject")]
    public GameObject additionalHinge2;


    private TCPClient.MotorData[] previousMotorDataArray; // 保存上一次收到的电机数据

    private const float SPRING_STIFFNESS = 100f; // 弹簧刚度
    private const float SPRING_DAMPER = 30f; // 弹簧阻尼



    void Start()
    {
        tcpClient = FindObjectOfType<TCPClient>();
        // 初始化 previousMotorDataArray，大小应与 TCPClient 中的电机数据数组一致
        if (tcpClient != null)
        {
             previousMotorDataArray = new TCPClient.MotorData[TCPClient.MOTOR_DATA_ARRAY_LENGTH];
        }
        else
        {
             Debug.LogError("MotorController: TCPClient reference is not set. Disabling controller.");
             enabled = false;
        }
    }

    void FixedUpdate()
    {
        if (tcpClient != null && tcpClient.isConnected) // 确保 TCPClient 已连接
        {
            TCPClient.MotorData[] motorDataArray;
            lock (tcpClient) // 锁定以安全访问共享数据
            {
                motorDataArray = tcpClient.GetMotorData();
            }

            // 检查接收到的电机数据长度是否符合预期
            if (motorDataArray == null || motorDataArray.Length < TCPClient.MOTOR_DATA_ARRAY_LENGTH)
            {
                // 数据不完整，跳过本次更新
                Debug.LogWarning("MotorController: Received motor data is incomplete.");
                return;
            }


            // 更新前五个电机的状态 (与原来逻辑一致)
            for (int i = 0; i < motors.Length; i++)
            {
                if (i < motorDataArray.Length) // 再次检查以防万一
                {
                    UpdateMotor(motors[i], motorDataArray[i].angle, previousMotorDataArray[i].angle);
                }
            }

            // 更新两个新的铰链的状态，使用第五个电机的数据 (索引为 4)
            int motor5Index = 4; // 第五个电机的数据索引
            if (motorDataArray.Length > motor5Index) // 确保存在第五个电机的数据
            {
                float targetAngleForNewHinges = motorDataArray[motor5Index].angle;
                float previousTargetAngleForNewHinges = previousMotorDataArray[motor5Index].angle;

                if (additionalHinge1 != null)
                {
                    UpdateMotor(additionalHinge1, targetAngleForNewHinges, previousTargetAngleForNewHinges);
                }
                if (additionalHinge2 != null)
                {
                    UpdateMotor(additionalHinge2, targetAngleForNewHinges, previousTargetAngleForNewHinges);
                }
            }
            else
            {
                 Debug.LogWarning($"MotorController: Motor data array does not contain data for motor index {motor5Index}. Cannot update additional hinges.");
            }


            // 保存当前电机数据以供下次比较
            // 遍历长度应与 TCPClient 中的电机数据数组一致
            for (int i = 0; i < motorDataArray.Length; i++)
            {
                previousMotorDataArray[i] = motorDataArray[i];
            }
        }
    }

    void UpdateMotor(GameObject motor, float targetAngle, float previousTargetAngle)
    {
        HingeJoint hingeJoint = motor.GetComponent<HingeJoint>();
        if (hingeJoint != null)
        {
            // 获取当前的弹簧设置
            JointSpring spring = hingeJoint.spring;

            // 总是更新目标角度
            spring.targetPosition = targetAngle;

            // 如果在编辑器中没有启用弹簧，则使用脚本中定义的参数
            if (!hingeJoint.useSpring)
            {
                spring.spring = SPRING_STIFFNESS; // 使用脚本定义的弹簧刚度
                spring.damper = SPRING_DAMPER; // 使用脚本定义的阻尼
                hingeJoint.useSpring = true; // 启用弹簧
            }
            // 如果在编辑器中已经启用了弹簧，则保留编辑器的 spring 和 damper 参数

            // 应用更新后的弹簧设置
            hingeJoint.spring = spring;


            // 输出当前角度和上一次目标角度的差值
            float currentAngle = hingeJoint.angle;
            float angleDifference = Mathf.Abs(currentAngle - previousTargetAngle);
            //Debug.Log($"Motor: Current Angle = {currentAngle}, Previous Target Angle = {previousTargetAngle}, Difference = {angleDifference}");

            // 如果接近目标角度，减速 (此逻辑保持不变)
            if (angleDifference < 0.5f)
            {
                JointMotor motorSettings = hingeJoint.motor;
                motorSettings.targetVelocity = 0;
                hingeJoint.motor = motorSettings;
            }
        }
    }

    void TestMotorRotation()
    {
        foreach (var motor in motors)
        {
            // 让每个电机以速度10持续旋转
            // 注意：这里调用 UpdateMotor 会设置 targetPosition，而不是持续旋转
            // 如果需要测试持续旋转，应该直接修改 hingeJoint.motor.targetVelocity
            // UpdateMotor(motor, (short)(motor.transform.localEulerAngles.y + 10), 0); // 原始代码，可能不是期望的测试行为
            HingeJoint hingeJoint = motor.GetComponent<HingeJoint>();
            if (hingeJoint != null)
            {
                JointMotor motorSettings = hingeJoint.motor;
                motorSettings.targetVelocity = 10; // 设置目标速度为10
                motorSettings.force = float.PositiveInfinity; // 设置足够大的力来达到目标速度
                hingeJoint.motor = motorSettings;
                hingeJoint.useMotor = true; // 启用电机
            }
        }
    }
}