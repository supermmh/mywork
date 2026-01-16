using UnityEngine;
using System.IO;
using System.Text;
using System.Globalization;
using System.Collections.Generic; // 用于List
using System.Linq; // 用于 LINQ 查询

public class StateLogger : MonoBehaviour
{
    [Tooltip("输出的CSV文件名基础名称")]
    public string outputFileNameBase = "robot_state_log";
    [Tooltip("日志记录的频率 (秒)。0表示每帧记录。")]
    public float logIntervalSeconds = 0.1f;

    // --- TCPClient (仍然需要它来获取目标值) ---
    public TCPClient tcpClient;

    // --- 直接引用的 GameObjects ---

    [Header("Motors (e.g., Chassis/Wheel Motors)")]
    public List<GameObject> motorGameObjects;

    // --- 内部状态 ---
    private float timeSinceLastLog = 0f;
    private string outputFilePath;
    private StreamWriter writer;
    private float startTime; // 记录脚本启动的时间

    void Start()
    {
        if (tcpClient == null) tcpClient = FindObjectOfType<TCPClient>();
        if (tcpClient == null)
        {
            Debug.LogError("StateLogger: TCPClient reference is not set. Disabling logger.");
            enabled = false;
            return;
        }

        if (motorGameObjects == null || motorGameObjects.Count < TCPClient.MOTOR_DATA_ARRAY_LENGTH)
        {
            Debug.LogWarning($"StateLogger: motorGameObjects list is null or has less than {TCPClient.MOTOR_DATA_ARRAY_LENGTH} elements. " +
                             "Actual data for some motors may be NaN.");
        }

        // --- 生成新的文件名 ---
        int executionNumber = 1;
        string directoryPath = Application.persistentDataPath;
        string searchPattern = $"{outputFileNameBase}_*.csv";

        // 获取所有符合模式的文件
        string[] existingFiles = Directory.GetFiles(directoryPath, searchPattern);

        // 查找最大的执行次数
        if (existingFiles.Length > 0)
        {
            int maxNumber = 0;
            foreach (string filePath in existingFiles)
            {
                string fileName = Path.GetFileNameWithoutExtension(filePath);
                // 尝试从文件名中提取数字部分 (例如 "robot_state_log_12" -> "12")
                string numberPart = fileName.Substring(outputFileNameBase.Length + 1); // +1 是为了跳过 "_"
                if (int.TryParse(numberPart, out int currentNumber))
                {
                    if (currentNumber > maxNumber)
                    {
                        maxNumber = currentNumber;
                    }
                }
            }
            executionNumber = maxNumber + 1; // 下一个执行次数
        }

        // 构建新的文件名
        string newFileName = $"{outputFileNameBase}_{executionNumber}.csv";
        outputFilePath = Path.Combine(directoryPath, newFileName);

        Debug.Log($"StateLogger: Logging to new file: {outputFilePath}");

        try
        {
            // 使用 append: false 创建新文件, 并指定编码方式为 UTF8
            writer = new StreamWriter(outputFilePath, false, Encoding.UTF8);
            writer.AutoFlush = true;
            // 总是写入头部，因为是新文件
            WriteHeader();
        }
        catch (IOException ex)
        {
            Debug.LogError($"StateLogger: Error creating or opening file: {ex.Message}");
            enabled = false;
        }

        startTime = Time.time; // 记录脚本启动的时间
    }


    void OnDestroy()
    {
        if (writer != null) { writer.Close(); writer = null; }
    }
    void OnApplicationQuit() => OnDestroy();

    void Update()
    {
        if (writer == null || !enabled) return;
        timeSinceLastLog += Time.deltaTime;
        if (timeSinceLastLog >= logIntervalSeconds || logIntervalSeconds <= 0)
        {
            LogCurrentState();
            timeSinceLastLog = (logIntervalSeconds > 0) ? timeSinceLastLog % logIntervalSeconds : 0f;
        }
    }

    void WriteHeader()
    {
        if (writer == null) return;
        StringBuilder sb = new StringBuilder();
        sb.Append("Timestamp,IsConnected,");

        // 普通电机 (Motor Targets 对应 Motor Actuals)
        // 替换为固定的数据名称
        sb.Append("TargetLeftFront,ActualLeftFront,");
        sb.Append("TargetLeftHind,ActualLeftHind,");
        sb.Append("TargetRightFront,ActualRightFront,");
        sb.Append("TargetRightHind,ActualRightHind,");
        sb.Append("TargetHead,ActualHead,");

        // 移除最后一个逗号
        if (sb.Length > 0) sb.Length--;
        writer.WriteLine(sb.ToString());
    }

    void LogCurrentState()
    {
        // 只有当时间超过 startTime + 1 秒后，才记录数据
        if (Time.time < startTime + 1f)
        {
            return; // 跳过本次记录
        }

        if (writer == null) return;
        StringBuilder sb = new StringBuilder();

        sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F3},", Time.time);
        sb.Append($"{tcpClient.isConnected.ToString().ToLowerInvariant()},");

        // --- Targets and Actuals (Paired) ---

        TCPClient.MotorData[] motorTargets = tcpClient.GetMotorData();

        // 普通电机 (Motor Targets 对应 Motor Actuals)
        for (int i = 0; i < TCPClient.MOTOR_DATA_ARRAY_LENGTH; i++)
        {
            // Target
            sb.Append(i < motorTargets.Length ? string.Format(CultureInfo.InvariantCulture, "{0:F2},", motorTargets[i].angle) : "NaN,");
            // Actual
            sb.AppendFormat(CultureInfo.InvariantCulture, "{0:F2},",
                            (motorGameObjects != null && i < motorGameObjects.Count) ?
                            GetHingeAngleFromGameObject(motorGameObjects[i], null) : float.NaN);
        }


        // 移除最后一个逗号
        if (sb.Length > 0) sb.Length--;
        writer.WriteLine(sb.ToString());
    }

    private float GetHingeAngleFromGameObject(GameObject go, string connectedBodyName = null)
    {
        if (go == null) return float.NaN;
        HingeJoint[] hingeJoints = go.GetComponents<HingeJoint>();
        if (hingeJoints == null || hingeJoints.Length == 0) return float.NaN;

        HingeJoint targetHinge = null;
        if (string.IsNullOrEmpty(connectedBodyName))
        {
            targetHinge = hingeJoints[0];
        }
        else
        {
            foreach (var hj in hingeJoints)
            {
                if (hj.connectedBody != null && hj.connectedBody.name == connectedBodyName) { targetHinge = hj; break; }
            }
            if (targetHinge == null) { return float.NaN; }
        }
        return (targetHinge != null) ? targetHinge.angle : float.NaN;
    }
}