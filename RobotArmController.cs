using UnityEngine;

public class RobotArmController : MonoBehaviour
{
    private TCPClient tcpClient;
    public GameObject[] rotationMotors; // 用于控制转动的电机，分别是底盘转动、大臂、小臂
    public GameObject positionMotor; // 用于控制位置的电机，导轨
    public GameObject[] angleMotors; // 用于控制角度的电机，两个夹具
    public GameObject hingeA; // 外部铰链A，舵机连接板
    public GameObject hingeB; // 外部铰链B，夹具

    private float lastLogTime = 0f; // 上次输出日志的时间
    private float KArmPosition = 0.1f; // 位置增益

    private const int INITIAL_HINGE_A_ANGLE = -25; // 初始化铰链A的角度，与小臂同步
    private const int INITIAL_HINGE_B_ANGLE = -6; // 初始化铰链B的角度
    private const int INITIAL_BIG_ARM_ANGLE = 0; // 初始化大臂的角度
    private const int INITIAL_SMALL_ARM_ANGLE = -8; // 初始化小臂的角度
    void Start()
    {
        tcpClient = FindObjectOfType<TCPClient>();
        // 初始化铰链的角度
        UpdateRotationMotor(hingeA, INITIAL_HINGE_A_ANGLE);
        UpdateRotationMotor(hingeB, INITIAL_HINGE_B_ANGLE);
        UpdateRotationMotor(rotationMotors[1], INITIAL_BIG_ARM_ANGLE, "机械臂2连接底板件"); // 大臂
        UpdateRotationMotor(rotationMotors[2], INITIAL_SMALL_ARM_ANGLE, "机械臂2"); // 小臂

    }

    void FixedUpdate()
    {
        // 获取 Jetson 数据
        if (tcpClient != null)
        {
            int[] jetsonData = tcpClient.GetJetsonData();

            if (jetsonData.Length >= 5)
            {
                // 控制转动
                UpdateRotationMotor(rotationMotors[0], jetsonData[1]);
                UpdateRotationMotor(rotationMotors[1], jetsonData[2], "机械臂2连接底板件");
                UpdateRotationMotor(rotationMotors[2], jetsonData[3], "机械臂2");

                // 控制位置
                UpdatePositionMotor(positionMotor, jetsonData[0]);

                // 控制角度
                for (int i = 0; i < angleMotors.Length; i++)
                {
                    UpdateAngleMotor(angleMotors[i], jetsonData[4], null);
                }

                int bigArmAngle = jetsonData[2];
                int smallArmAngle = jetsonData[3];
                int hingeAAngle = -(smallArmAngle+bigArmAngle)+3;
                int hingeBAngle = bigArmAngle + smallArmAngle - 10;

                UpdateRotationMotor(hingeA, hingeAAngle);
                UpdateRotationMotor(hingeB, hingeBAngle);

                // 每秒输出一次电机和滑块当前位置与目标位置的差值
                if (Time.time - lastLogTime >= 1.0f)
                {
                    //ReportRotationMotorsDifference(jetsonData);
                    //ReportPositionMotorDifference(jetsonData[3]);
                    //ReportAngleMotorsDifference(jetsonData[4]);

                    lastLogTime = Time.time;
                }
            }
        }


    }

    void UpdateRotationMotor(GameObject motor, int targetAngle)
    {
        HingeJoint hingeJoint = motor.GetComponent<HingeJoint>();
        if (hingeJoint != null)
        {
            // 设置弹簧的目标位置
            JointSpring springSettings = hingeJoint.spring;
            springSettings.targetPosition = targetAngle;
            hingeJoint.spring = springSettings;
            hingeJoint.useSpring = true;
        }
    }

    void UpdateRotationMotor(GameObject motor, int targetAngle, string connectedBodyName)
    {
        HingeJoint[] hingeJoints = motor.GetComponents<HingeJoint>();
        HingeJoint hingeJoint = null;
        foreach (var hj in hingeJoints)
        {
            if (hj.connectedBody != null && hj.connectedBody.name == connectedBodyName)
            {
                hingeJoint = hj;
                break;
            }
        }

        if (hingeJoint != null)
        {
            // 设置弹簧的目标位置
            JointSpring springSettings = hingeJoint.spring;
            springSettings.targetPosition = targetAngle;
            hingeJoint.spring = springSettings;
            hingeJoint.useSpring = true;
        }
        else
        {
            Debug.LogWarning($"HingeJoint with connected body {connectedBodyName} not found on {motor.name}");
        }
    }

    void UpdatePositionMotor(GameObject motor, int targetPositionData)
    {
        ConfigurableJoint joint = motor.GetComponent<ConfigurableJoint>();
        if (joint != null)
        {
            // 设置目标位置
            joint.targetPosition = new Vector3(targetPositionData * KArmPosition, 0, 0);
        }
    }

    void UpdateAngleMotor(GameObject motor, int targetAngleData, string hingeJointName = null)
    {
        HingeJoint hingeJoint = null;
        if (string.IsNullOrEmpty(hingeJointName))
        {
            hingeJoint = motor.GetComponent<HingeJoint>();
        }
        else
        {
            HingeJoint[] hingeJoints = motor.GetComponents<HingeJoint>();
            foreach (var hj in hingeJoints)
            {
                if (hj.name == hingeJointName)
                {
                    hingeJoint = hj;
                    break;
                }
            }
        }

        if (hingeJoint != null)
        {
            // 设置弹簧的目标位置
            JointSpring springSettings = hingeJoint.spring;
            springSettings.targetPosition = targetAngleData;
            hingeJoint.spring = springSettings;
            hingeJoint.useSpring = true;
        }
    }

    void ReportRotationMotorsDifference(int[] jetsonData)
    {
        for (int i = 0; i < rotationMotors.Length; i++)
        {
            GameObject motor = rotationMotors[i];
            HingeJoint hingeJoint = motor.GetComponent<HingeJoint>();
            if (hingeJoint != null)
            {
                int currentAngle = Mathf.RoundToInt(hingeJoint.angle);
                int targetAngle = jetsonData[i];
                int angleDifference = Mathf.Abs(currentAngle - targetAngle);
                Debug.Log($"Rotation Motor {hingeJoint.name}: Current Angle = {currentAngle}, Target Angle = {targetAngle}, Difference = {angleDifference}");
            }
        }
    }

    void ReportPositionMotorDifference(int targetPositionData)
    {
        Rigidbody rb = positionMotor.GetComponent<Rigidbody>();
        if (rb != null)
        {
            Vector3 currentPosition = rb.position;
            Vector3 targetPosition = new Vector3(targetPositionData * KArmPosition, 0, 0);
            Vector3 difference = currentPosition - targetPosition;
            Debug.Log($"Position Motor: Current Position = {currentPosition}, Target Position = {targetPosition}, Difference = {difference}");
        }
    }

    void ReportAngleMotorsDifference(int targetAngleData)
    {
        for (int i = 0; i < angleMotors.Length; i++)
        {
            GameObject motor = angleMotors[i];
            HingeJoint hingeJoint = motor.GetComponent<HingeJoint>();
            if (hingeJoint != null)
            {
                int currentAngle = Mathf.RoundToInt(hingeJoint.angle);
                int angleDifference = Mathf.Abs(currentAngle - targetAngleData);
                Debug.Log($"Angle Motor {hingeJoint.name}: Current Angle = {currentAngle}, Target Angle = {targetAngleData}, Difference = {angleDifference}");
            }
        }
    }
}
