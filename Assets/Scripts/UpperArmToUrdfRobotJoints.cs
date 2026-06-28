using System.Collections;
using UnityEngine;

/// <summary>
/// This script maps Humanoid right-arm motion to a URDF robot arm inside Unity.
/// UDP communication and real robot control are intentionally separated into a later phase.
/// </summary>
public class UpperArmToUrdfRobotJoints : MonoBehaviour
{
    [Header("Humanoid")]
    [SerializeField] private Animator humanoidAnimator;

    [Header("URDF Robot Joints")]
    [SerializeField] private ArticulationBody joint1;
    [SerializeField] private ArticulationBody joint2;
    [SerializeField] private ArticulationBody joint3;
    [SerializeField] private ArticulationBody joint4;
    [SerializeField] private ArticulationBody joint5;
    [SerializeField] private ArticulationBody joint6;

    [Header("Robot Base Angles")]
    [SerializeField] private float robotBaseJ1 = 0f;
    [SerializeField] private float robotBaseJ2 = 0f;
    [SerializeField] private float robotBaseJ3 = 0f;
    [SerializeField] private float robotBaseJ4 = 0f;
    [SerializeField] private float robotBaseJ5 = 0f;
    [SerializeField] private float robotBaseJ6 = 0f;

    [Header("Mapping")]
    [SerializeField] private float j1Scale = 1f;
    [SerializeField] private float j2Scale = 1f;
    [SerializeField] private float j3Scale = 1f;
    [SerializeField] private float j4Scale = 1f;
    [SerializeField] private float j5Scale = 1f;
    [SerializeField] private float j6Scale = 1f;
    [SerializeField] private float j1Offset = 0f;
    [SerializeField] private float j2Offset = 0f;
    [SerializeField] private float j3Offset = 0f;
    [SerializeField] private float j4Offset = 0f;
    [SerializeField] private float j5Offset = 0f;
    [SerializeField] private float j6Offset = 0f;
    [SerializeField] private bool invertJ1 = false;
    [SerializeField] private bool invertJ2 = false;
    [SerializeField] private bool invertJ3 = false;
    [SerializeField] private bool invertJ4 = false;
    [SerializeField] private bool invertJ5 = false;
    [SerializeField] private bool invertJ6 = false;

    [Header("Joint Limits")]
    [SerializeField] private float j1Min = -165f;
    [SerializeField] private float j1Max = 165f;
    [SerializeField] private float j2Min = -165f;
    [SerializeField] private float j2Max = 165f;
    [SerializeField] private float j3Min = -165f;
    [SerializeField] private float j3Max = 165f;
    [SerializeField] private float j4Min = -165f;
    [SerializeField] private float j4Max = 165f;
    [SerializeField] private float j5Min = -165f;
    [SerializeField] private float j5Max = 165f;
    [SerializeField] private float j6Min = -180f;
    [SerializeField] private float j6Max = 180f;

    [Header("Stability")]
    [SerializeField] private float yawUpdateThreshold = 0.1f;
    [SerializeField, Range(0f, 1f)] private float smoothing = 0.2f;

    [Header("Articulation Drive")]
    [SerializeField] private bool applyDriveSettings = true;
    [SerializeField] private float driveStiffness = 10000f;
    [SerializeField] private float driveDamping = 100f;
    [SerializeField] private float driveForceLimit = 1000f;
    [SerializeField] private bool logJointDiagnosticsOnStart = true;

    [Header("Calibration")]
    [SerializeField] private bool autoCalibrateOnStart = true;
    [SerializeField] private float autoCalibrateDelay = 2f;
    [SerializeField] private bool driveOnlyAfterCalibration = true;

    [Header("Debug")]
    [SerializeField] private bool showDebugGui = true;
    [SerializeField] private bool showDebugLog = false;
    [SerializeField] private float debugLogInterval = 1f;

    private Transform upperArm;
    private Transform lowerArm;
    private Transform rightHand;

    private Vector3 armDir;
    private Vector3 forearmDir;
    private float rawYaw;
    private float rawPitch;
    private float rawElbow;
    private float rawForearmYaw;
    private float rawHandPitch;
    private float rawHandRoll;
    private float previousRawYaw;
    private float previousRawForearmYaw;
    private float baseYaw;
    private float basePitch;
    private float baseElbow;
    private float baseForearmYaw;
    private float baseHandPitch;
    private float baseHandRoll;
    private float deltaYaw;
    private float deltaPitch;
    private float deltaElbow;
    private float deltaForearmYaw;
    private float deltaHandPitch;
    private float deltaHandRoll;
    private float robotJ1;
    private float robotJ2;
    private float robotJ3;
    private float robotJ4;
    private float robotJ5;
    private float robotJ6;
    private float smoothedJ1;
    private float smoothedJ2;
    private float smoothedJ3;
    private float smoothedJ4;
    private float smoothedJ5;
    private float smoothedJ6;
    private float lastDebugLogTime;
    private bool isCalibrated;
    private bool hasValidBones;
    private readonly bool[] warnedJoints = new bool[6];

    public float SmoothedJ1 => smoothedJ1;
    public float SmoothedJ2 => smoothedJ2;
    public float SmoothedJ3 => smoothedJ3;
    public float SmoothedJ4 => smoothedJ4;
    public float SmoothedJ5 => smoothedJ5;
    public float SmoothedJ6 => smoothedJ6;
    public float RawYaw => rawYaw;
    public float RawPitch => rawPitch;
    public float RawElbow => rawElbow;
    public float RawForearmYaw => rawForearmYaw;
    public float RawHandPitch => rawHandPitch;
    public float RawHandRoll => rawHandRoll;
    public float DeltaYaw => deltaYaw;
    public float DeltaPitch => deltaPitch;
    public float DeltaElbow => deltaElbow;
    public float DeltaForearmYaw => deltaForearmYaw;
    public float DeltaHandPitch => deltaHandPitch;
    public float DeltaHandRoll => deltaHandRoll;
    public Vector3 ArmDirection => armDir;
    public Vector3 ForearmDirection => forearmDir;
    public bool IsCalibrated => isCalibrated;

    private void Awake()
    {
        if (humanoidAnimator == null)
        {
            humanoidAnimator = GetComponentInChildren<Animator>();
        }
    }

    private void Start()
    {
        hasValidBones = TryInitializeBones();
        smoothedJ1 = Mathf.Clamp(robotBaseJ1 + j1Offset, j1Min, j1Max);
        smoothedJ2 = Mathf.Clamp(robotBaseJ2 + j2Offset, j2Min, j2Max);
        smoothedJ3 = Mathf.Clamp(robotBaseJ3 + j3Offset, j3Min, j3Max);
        smoothedJ4 = Mathf.Clamp(robotBaseJ4 + j4Offset, j4Min, j4Max);
        smoothedJ5 = Mathf.Clamp(robotBaseJ5 + j5Offset, j5Min, j5Max);
        smoothedJ6 = Mathf.Clamp(robotBaseJ6 + j6Offset, j6Min, j6Max);
        ConfigureJointDrive(joint1);
        ConfigureJointDrive(joint2);
        ConfigureJointDrive(joint3);
        ConfigureJointDrive(joint4);
        ConfigureJointDrive(joint5);
        ConfigureJointDrive(joint6);

        if (logJointDiagnosticsOnStart)
        {
            LogJointDiagnostics();
        }

        if (hasValidBones && autoCalibrateOnStart)
        {
            StartCoroutine(AutoCalibrateAfterDelay());
        }
    }

    private void Update()
    {
        if (!hasValidBones)
        {
            return;
        }

        UpdateArmAngles();

        if (!isCalibrated && driveOnlyAfterCalibration)
        {
            return;
        }

        UpdateRobotAngles();
        SetJointTarget(joint1, smoothedJ1);
        SetJointTarget(joint2, smoothedJ2);
        SetJointTarget(joint3, smoothedJ3);
        SetJointTarget(joint4, smoothedJ4);
        SetJointTarget(joint5, smoothedJ5);
        SetJointTarget(joint6, smoothedJ6);
        WriteDebugLogIfNeeded();
    }

    private bool TryInitializeBones()
    {
        if (humanoidAnimator == null)
        {
            Debug.LogError($"{nameof(UpperArmToUrdfRobotJoints)} requires a Humanoid Animator.");
            return false;
        }

        if (!humanoidAnimator.isHuman)
        {
            Debug.LogError($"{nameof(UpperArmToUrdfRobotJoints)} requires an Animator with a Humanoid Avatar.", humanoidAnimator);
            return false;
        }

        upperArm = humanoidAnimator.GetBoneTransform(HumanBodyBones.RightUpperArm);
        lowerArm = humanoidAnimator.GetBoneTransform(HumanBodyBones.RightLowerArm);
        rightHand = humanoidAnimator.GetBoneTransform(HumanBodyBones.RightHand);

        if (upperArm == null || lowerArm == null || rightHand == null)
        {
            Debug.LogError($"{nameof(UpperArmToUrdfRobotJoints)} could not find RightUpperArm, RightLowerArm, or RightHand bones.", humanoidAnimator);
            return false;
        }

        return true;
    }

    private IEnumerator AutoCalibrateAfterDelay()
    {
        yield return new WaitForSeconds(autoCalibrateDelay);
        UpdateArmAngles();
        Calibrate();
    }

    private void UpdateArmAngles()
    {
        armDir = lowerArm.position - upperArm.position;
        if (armDir.sqrMagnitude <= Mathf.Epsilon)
        {
            return;
        }

        armDir.Normalize();

        float horizontalLength = new Vector2(armDir.x, armDir.z).magnitude;
        if (horizontalLength > yawUpdateThreshold)
        {
            rawYaw = Mathf.Atan2(armDir.x, armDir.z) * Mathf.Rad2Deg;
            previousRawYaw = rawYaw;
        }
        else
        {
            rawYaw = previousRawYaw;
        }

        rawPitch = Mathf.Atan2(armDir.y, horizontalLength) * Mathf.Rad2Deg;

        forearmDir = rightHand.position - lowerArm.position;
        if (forearmDir.sqrMagnitude <= Mathf.Epsilon)
        {
            return;
        }

        forearmDir.Normalize();

        rawElbow = Vector3.Angle(armDir, forearmDir);

        float forearmHorizontalLength = new Vector2(forearmDir.x, forearmDir.z).magnitude;
        if (forearmHorizontalLength > yawUpdateThreshold)
        {
            rawForearmYaw = Mathf.Atan2(forearmDir.x, forearmDir.z) * Mathf.Rad2Deg;
            previousRawForearmYaw = rawForearmYaw;
        }
        else
        {
            rawForearmYaw = previousRawForearmYaw;
        }

        Vector3 handEuler = rightHand.rotation.eulerAngles;
        rawHandPitch = NormalizeAngle(handEuler.x);
        rawHandRoll = NormalizeAngle(handEuler.z);
    }

    private void UpdateRobotAngles()
    {
        deltaYaw = Mathf.DeltaAngle(baseYaw, rawYaw);
        deltaPitch = rawPitch - basePitch;
        deltaElbow = rawElbow - baseElbow;
        deltaForearmYaw = Mathf.DeltaAngle(baseForearmYaw, rawForearmYaw);
        deltaHandPitch = Mathf.DeltaAngle(baseHandPitch, rawHandPitch);
        deltaHandRoll = Mathf.DeltaAngle(baseHandRoll, rawHandRoll);

        float j1Sign = invertJ1 ? -1f : 1f;
        float j2Sign = invertJ2 ? -1f : 1f;
        float j3Sign = invertJ3 ? -1f : 1f;
        float j4Sign = invertJ4 ? -1f : 1f;
        float j5Sign = invertJ5 ? -1f : 1f;
        float j6Sign = invertJ6 ? -1f : 1f;

        robotJ1 = robotBaseJ1 + deltaYaw * j1Scale * j1Sign + j1Offset;
        robotJ2 = robotBaseJ2 + deltaPitch * j2Scale * j2Sign + j2Offset;
        robotJ3 = robotBaseJ3 + deltaElbow * j3Scale * j3Sign + j3Offset;
        robotJ4 = robotBaseJ4 + deltaForearmYaw * j4Scale * j4Sign + j4Offset;
        robotJ5 = robotBaseJ5 + deltaHandPitch * j5Scale * j5Sign + j5Offset;
        robotJ6 = robotBaseJ6 + deltaHandRoll * j6Scale * j6Sign + j6Offset;

        robotJ1 = Mathf.Clamp(robotJ1, j1Min, j1Max);
        robotJ2 = Mathf.Clamp(robotJ2, j2Min, j2Max);
        robotJ3 = Mathf.Clamp(robotJ3, j3Min, j3Max);
        robotJ4 = Mathf.Clamp(robotJ4, j4Min, j4Max);
        robotJ5 = Mathf.Clamp(robotJ5, j5Min, j5Max);
        robotJ6 = Mathf.Clamp(robotJ6, j6Min, j6Max);

        smoothedJ1 = Mathf.Lerp(smoothedJ1, robotJ1, smoothing);
        smoothedJ2 = Mathf.Lerp(smoothedJ2, robotJ2, smoothing);
        smoothedJ3 = Mathf.Lerp(smoothedJ3, robotJ3, smoothing);
        smoothedJ4 = Mathf.Lerp(smoothedJ4, robotJ4, smoothing);
        smoothedJ5 = Mathf.Lerp(smoothedJ5, robotJ5, smoothing);
        smoothedJ6 = Mathf.Lerp(smoothedJ6, robotJ6, smoothing);
    }

    private void SetJointTarget(ArticulationBody joint, float targetDeg)
    {
        if (joint == null)
        {
            return;
        }

        if (joint.jointType != ArticulationJointType.RevoluteJoint && joint.jointType != ArticulationJointType.PrismaticJoint)
        {
            WarnOnceForJoint(joint);
            return;
        }

        ArticulationDrive drive = joint.xDrive;
        if (applyDriveSettings)
        {
            drive.stiffness = driveStiffness;
            drive.damping = driveDamping;
            drive.forceLimit = driveForceLimit;
        }

        if (drive.lowerLimit < drive.upperLimit)
        {
            targetDeg = Mathf.Clamp(targetDeg, drive.lowerLimit, drive.upperLimit);
        }

        drive.target = targetDeg;
        joint.xDrive = drive;
    }

    private void ConfigureJointDrive(ArticulationBody joint)
    {
        if (joint == null || !applyDriveSettings)
        {
            return;
        }

        ArticulationDrive drive = joint.xDrive;
        drive.stiffness = driveStiffness;
        drive.damping = driveDamping;
        drive.forceLimit = driveForceLimit;
        joint.xDrive = drive;
    }

    private void WarnOnceForJoint(ArticulationBody joint)
    {
        int jointIndex = GetJointIndex(joint);
        if (jointIndex >= 0 && warnedJoints[jointIndex])
        {
            return;
        }

        if (jointIndex >= 0)
        {
            warnedJoints[jointIndex] = true;
        }

        Debug.LogWarning($"{nameof(UpperArmToUrdfRobotJoints)} expected a movable ArticulationBody, but '{joint.name}' is {joint.jointType}. Assign the ArticulationBody on the link that represents the revolute joint.", joint);
    }

    [ContextMenu("Log Joint Diagnostics")]
    public void LogJointDiagnostics()
    {
        LogJointDiagnostics("joint1", joint1);
        LogJointDiagnostics("joint2", joint2);
        LogJointDiagnostics("joint3", joint3);
        LogJointDiagnostics("joint4", joint4);
        LogJointDiagnostics("joint5", joint5);
        LogJointDiagnostics("joint6", joint6);
    }

    private void LogJointDiagnostics(string label, ArticulationBody joint)
    {
        if (joint == null)
        {
            Debug.LogWarning($"{nameof(UpperArmToUrdfRobotJoints)} {label} is not assigned.", this);
            return;
        }

        ArticulationDrive drive = joint.xDrive;
        Debug.Log(
            $"{nameof(UpperArmToUrdfRobotJoints)} {label}: name={joint.name}, type={joint.jointType}, " +
            $"twistLock={joint.twistLock}, xDrive target={drive.target:F2}, limits=({drive.lowerLimit:F2}, {drive.upperLimit:F2}), " +
            $"stiffness={drive.stiffness:F2}, damping={drive.damping:F2}, forceLimit={drive.forceLimit:F2}",
            joint);
    }

    private int GetJointIndex(ArticulationBody joint)
    {
        if (joint == joint1) return 0;
        if (joint == joint2) return 1;
        if (joint == joint3) return 2;
        if (joint == joint4) return 3;
        if (joint == joint5) return 4;
        if (joint == joint6) return 5;
        return -1;
    }

    private float NormalizeAngle(float angle)
    {
        return Mathf.DeltaAngle(0f, angle);
    }

    [ContextMenu("Calibrate Upper Arm")]
    public void Calibrate()
    {
        if (!hasValidBones)
        {
            hasValidBones = TryInitializeBones();
            if (!hasValidBones)
            {
                return;
            }
        }

        baseYaw = rawYaw;
        basePitch = rawPitch;
        baseElbow = rawElbow;
        baseForearmYaw = rawForearmYaw;
        baseHandPitch = rawHandPitch;
        baseHandRoll = rawHandRoll;
        deltaYaw = 0f;
        deltaPitch = 0f;
        deltaElbow = 0f;
        deltaForearmYaw = 0f;
        deltaHandPitch = 0f;
        deltaHandRoll = 0f;
        isCalibrated = true;

        smoothedJ1 = Mathf.Clamp(robotBaseJ1 + j1Offset, j1Min, j1Max);
        smoothedJ2 = Mathf.Clamp(robotBaseJ2 + j2Offset, j2Min, j2Max);
        smoothedJ3 = Mathf.Clamp(robotBaseJ3 + j3Offset, j3Min, j3Max);
        smoothedJ4 = Mathf.Clamp(robotBaseJ4 + j4Offset, j4Min, j4Max);
        smoothedJ5 = Mathf.Clamp(robotBaseJ5 + j5Offset, j5Min, j5Max);
        smoothedJ6 = Mathf.Clamp(robotBaseJ6 + j6Offset, j6Min, j6Max);

        Debug.Log($"{nameof(UpperArmToUrdfRobotJoints)} calibrated. baseYaw={baseYaw:F2}, basePitch={basePitch:F2}, baseElbow={baseElbow:F2}, baseForearmYaw={baseForearmYaw:F2}, baseHandPitch={baseHandPitch:F2}, baseHandRoll={baseHandRoll:F2}", this);
    }

    public Vector2 GetRobotJ1J2()
    {
        return new Vector2(smoothedJ1, smoothedJ2);
    }

    public float[] GetRobotJoints()
    {
        return new[] { smoothedJ1, smoothedJ2, smoothedJ3, smoothedJ4, smoothedJ5, smoothedJ6 };
    }

    private void WriteDebugLogIfNeeded()
    {
        if (!showDebugLog || Time.time - lastDebugLogTime < debugLogInterval)
        {
            return;
        }

        lastDebugLogTime = Time.time;
        Debug.Log(
            $"armDir={armDir} forearmDir={forearmDir} raw=({rawYaw:F2}, {rawPitch:F2}, {rawElbow:F2}, {rawForearmYaw:F2}, {rawHandPitch:F2}, {rawHandRoll:F2}) " +
            $"delta=({deltaYaw:F2}, {deltaPitch:F2}, {deltaElbow:F2}, {deltaForearmYaw:F2}, {deltaHandPitch:F2}, {deltaHandRoll:F2}) " +
            $"robot=({robotJ1:F2}, {robotJ2:F2}, {robotJ3:F2}, {robotJ4:F2}, {robotJ5:F2}, {robotJ6:F2}) " +
            $"smoothed=({smoothedJ1:F2}, {smoothedJ2:F2}, {smoothedJ3:F2}, {smoothedJ4:F2}, {smoothedJ5:F2}, {smoothedJ6:F2})",
            this);
    }

    private void OnGUI()
    {
        if (!showDebugGui)
        {
            return;
        }

        const float width = 360f;
        const float lineHeight = 22f;
        Rect area = new Rect(10f, 10f, width, 10f + lineHeight * 14f);

        GUILayout.BeginArea(area, GUI.skin.box);
        GUILayout.Label($"Calibrated: {isCalibrated}");
        GUILayout.Label($"armDir: {armDir.x:F2}, {armDir.y:F2}, {armDir.z:F2}");
        GUILayout.Label($"forearmDir: {forearmDir.x:F2}, {forearmDir.y:F2}, {forearmDir.z:F2}");
        GUILayout.Label($"rawYaw / rawPitch: {rawYaw:F2} / {rawPitch:F2}");
        GUILayout.Label($"rawElbow / rawForearmYaw: {rawElbow:F2} / {rawForearmYaw:F2}");
        GUILayout.Label($"rawHandPitch / rawHandRoll: {rawHandPitch:F2} / {rawHandRoll:F2}");
        GUILayout.Label($"deltaYaw / deltaPitch: {deltaYaw:F2} / {deltaPitch:F2}");
        GUILayout.Label($"deltaJ3-J6: {deltaElbow:F2} / {deltaForearmYaw:F2} / {deltaHandPitch:F2} / {deltaHandRoll:F2}");
        GUILayout.Label($"robotJ1-J3: {robotJ1:F2} / {robotJ2:F2} / {robotJ3:F2}");
        GUILayout.Label($"robotJ4-J6: {robotJ4:F2} / {robotJ5:F2} / {robotJ6:F2}");
        GUILayout.Label($"smoothedJ1-J3: {smoothedJ1:F2} / {smoothedJ2:F2} / {smoothedJ3:F2}");
        GUILayout.Label($"smoothedJ4-J6: {smoothedJ4:F2} / {smoothedJ5:F2} / {smoothedJ6:F2}");

        if (GUILayout.Button("Calibrate"))
        {
            Calibrate();
        }

        GUILayout.EndArea();
    }
}
