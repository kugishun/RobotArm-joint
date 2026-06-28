using System;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using UnityEngine;

public class MyCobotUdpJointSender : MonoBehaviour
{
    [Header("Joint Source")]
    [SerializeField] private UpperArmToUrdfRobotJoints jointSource;
    [SerializeField] private bool findSourceOnSameObject = true;

    [Header("UDP Target")]
    [SerializeField] private string remoteIp = "192.168.1.150";
    [SerializeField] private int remotePort = 7010;
    [SerializeField] private bool sendContinuously = true;
    [SerializeField] private float sendRateHz = 20f;
    [SerializeField] private bool sendOnceOnEnable = false;

    [Header("Gripper")]
    [SerializeField] private bool includeGripper = false;
    [SerializeField] private float[] gripperValues = { 0f, 0f, 0f };

    [Header("Debug")]
    [SerializeField] private bool logSentPayload = false;
    [SerializeField] private float logIntervalSeconds = 0.5f;

    private UdpClient udpClient;
    private IPEndPoint remoteEndPoint;
    private float sendTimer;
    private float lastLogTime = float.NegativeInfinity;
    private float SendIntervalSeconds => 1f / Mathf.Clamp(sendRateHz, 0.1f, 20f);

    private void Awake()
    {
        ResolveJointSource();
    }

    private void OnEnable()
    {
        OpenSocket();

        if (sendOnceOnEnable)
        {
            SendNow();
        }
    }

    private void Update()
    {
        if (!sendContinuously)
        {
            return;
        }

        sendTimer += Time.deltaTime;
        float sendIntervalSeconds = SendIntervalSeconds;
        if (sendTimer < sendIntervalSeconds)
        {
            return;
        }

        sendTimer %= sendIntervalSeconds;
        SendNow();
    }

    private void OnDisable()
    {
        CloseSocket();
    }

    private void OnDestroy()
    {
        CloseSocket();
    }

    private void OnValidate()
    {
        remotePort = Mathf.Clamp(remotePort, 1, 65535);
        sendRateHz = Mathf.Clamp(sendRateHz, 0.1f, 20f);
        logIntervalSeconds = Mathf.Max(0f, logIntervalSeconds);

        if (gripperValues == null)
        {
            gripperValues = new float[0];
        }
    }

    [ContextMenu("Send Now")]
    public void SendNow()
    {
        ResolveJointSource();

        if (jointSource == null)
        {
            Debug.LogWarning($"{nameof(MyCobotUdpJointSender)} requires a {nameof(UpperArmToUrdfRobotJoints)} source.", this);
            return;
        }

        if (udpClient == null || remoteEndPoint == null)
        {
            OpenSocket();
        }

        if (udpClient == null || remoteEndPoint == null)
        {
            return;
        }

        float[] angles = jointSource.GetRobotJoints();
        if (!HasValidValues(angles) || (includeGripper && !HasValidValues(gripperValues)))
        {
            Debug.LogWarning($"{nameof(MyCobotUdpJointSender)} skipped UDP send because payload contains NaN or Infinity.", this);
            return;
        }

        string payload = BuildPayloadJson(angles, includeGripper ? gripperValues : null);
        byte[] packet = Encoding.UTF8.GetBytes(payload);

        try
        {
            udpClient.Send(packet, packet.Length, remoteEndPoint);
            LogPayloadIfNeeded(payload);
        }
        catch (SocketException exception)
        {
            Debug.LogWarning($"{nameof(MyCobotUdpJointSender)} UDP send failed: {exception.Message}", this);
        }
        catch (ObjectDisposedException)
        {
            udpClient = null;
        }
    }

    private void ResolveJointSource()
    {
        if (jointSource != null || !findSourceOnSameObject)
        {
            return;
        }

        jointSource = GetComponent<UpperArmToUrdfRobotJoints>();
    }

    private void OpenSocket()
    {
        if (udpClient != null && remoteEndPoint != null)
        {
            return;
        }

        if (!IPAddress.TryParse(remoteIp, out IPAddress remoteAddress))
        {
            Debug.LogWarning($"{nameof(MyCobotUdpJointSender)} remoteIp is invalid: {remoteIp}", this);
            return;
        }

        CloseSocket();
        udpClient = new UdpClient();
        remoteEndPoint = new IPEndPoint(remoteAddress, remotePort);
    }

    private void CloseSocket()
    {
        if (udpClient == null)
        {
            remoteEndPoint = null;
            return;
        }

        udpClient.Close();
        udpClient = null;
        remoteEndPoint = null;
    }

    private string BuildPayloadJson(float[] angles, float[] gripper)
    {
        StringBuilder builder = new StringBuilder(128);
        builder.Append("{\"angles\":");
        AppendFloatArray(builder, angles);

        if (gripper != null)
        {
            builder.Append(",\"gripper\":");
            AppendFloatArray(builder, gripper);
        }

        builder.Append('}');
        return builder.ToString();
    }

    private void AppendFloatArray(StringBuilder builder, float[] values)
    {
        builder.Append('[');

        for (int index = 0; index < values.Length; index++)
        {
            if (index > 0)
            {
                builder.Append(',');
            }

            builder.Append(values[index].ToString("G9", CultureInfo.InvariantCulture));
        }

        builder.Append(']');
    }

    private bool HasValidValues(float[] values)
    {
        if (values == null)
        {
            return false;
        }

        for (int index = 0; index < values.Length; index++)
        {
            if (float.IsNaN(values[index]) || float.IsInfinity(values[index]))
            {
                return false;
            }
        }

        return true;
    }

    private void LogPayloadIfNeeded(string payload)
    {
        if (!logSentPayload || Time.time - lastLogTime < logIntervalSeconds)
        {
            return;
        }

        Debug.Log($"{nameof(MyCobotUdpJointSender)} sent {remoteIp}:{remotePort} {payload}", this);
        lastLogTime = Time.time;
    }
}
