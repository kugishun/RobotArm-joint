using UnityEngine;

public partial class HandPoseIkController : MonoBehaviour
{
    [Min(0.01f)] public float positionScale = 0.5f;
    [Min(1f)] public float maxJointSpeed = 45f;
    public bool followOrientation = true;
    [Tooltip("Optional calibrated grasp point; defaults to the sixth joint frame (flange).")]
    public Transform toolCenterPoint;
    [Header("IK start pose")]
    [Min(1f)] public float startSpeedDegrees = 30f;
    public bool PreparingStartPose { get; private set; }
    public bool StartPoseFailed { get; private set; }
    private float startElapsed, settledTime;
    private float[] requestedStartPose;
    private string startDiagnostics = "";
    private UpperArmToUrdfRobotJoints calibrationSource;
    [Header("Tracking visualization")]
    public bool showTrackingPoints = true;
    public string TrackingPointName => toolCenterPoint != null ? toolCenterPoint.name : "J6 flange (default)";
    public enum InputMode { Mocopi, TargetMarker }
    public InputMode Mode { get; private set; }
    public Transform TargetMarker { get; private set; }
    private GameObject ownedMarker;
    public void SetInputMode(InputMode mode)
    {
        ResetRangeCalibration();
        calibrated = applied = false;
        PreparingStartPose = StartPoseFailed = false;
        Mode = mode;
        if (ownedMarker != null) ownedMarker.SetActive(mode == InputMode.TargetMarker);
    }
    private ArticulationBody[] joints;
    private Animator animator;
    private Transform hand;
    private Vector3 handOrigin;
    private Quaternion handRotation, inputToWorld;
    private Pose toolOrigin;
    private float[] targets = new float[6];
    private bool calibrated, applied;
    public string Status { get; private set; } = "Calibrate to begin";
    public float PositionError { get; private set; }
    public float OrientationError { get; private set; }
    public bool CanSendJointTargets => Mode == InputMode.Mocopi && isActiveAndEnabled && calibrated && applied;
    public float[] GetRobotJoints() => (float[])targets.Clone();
    public void Configure(UpperArmToUrdfRobotJoints existing)
    {
        calibrationSource = existing;
        joints = existing.RobotChain;
        animator = existing.HumanoidAnimator;
    }
    public static bool Finite(float v) => !float.IsNaN(v) && !float.IsInfinity(v);
    private static bool Valid(Vector3 p) => Finite(p.x) && Finite(p.y) && Finite(p.z);
    private static bool Valid(Quaternion q) => Finite(q.x) && Finite(q.y) && Finite(q.z) && Finite(q.w) && q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w > 0.5f;
    private bool Ready(bool requireInput = true)
    {
        if (joints == null || joints.Length != 6) return false;
        if (requireInput && Mode == InputMode.Mocopi)
        {
            if (animator == null || !animator.isHuman) return false;
            hand = animator.GetBoneTransform(HumanBodyBones.RightHand);
            if (hand == null || !Valid(hand.position) || !Valid(hand.rotation)) return false;
        }
        for (int i = 0; i < 6; i++)
        {
            var j = joints[i];
            if (j == null || j.jointType != ArticulationJointType.RevoluteJoint || j.dofCount != 1 ||
                j.twistLock == ArticulationDofLock.LockedMotion || !Finite(j.jointPosition[0]) ||
                !Valid(j.transform.position) || !Valid(j.transform.rotation)) return false;
            if (i > 0 && !j.transform.IsChildOf(joints[i-1].transform)) return false;
            if (!Finite(j.xDrive.lowerLimit) || !Finite(j.xDrive.upperLimit) || j.xDrive.lowerLimit > j.xDrive.upperLimit) return false;
        }
        return Valid(Tip.position) && Valid(Tip.rotation) &&
            Finite(positionScale) && positionScale > 0 && Finite(maxJointSpeed) && maxJointSpeed > 0;
    }
    private Transform Tip => toolCenterPoint != null ? toolCenterPoint : joints[5].transform;
    public void BeginStartPose()
    {
        ResetRangeCalibration();
        calibrated = applied = false;
        PreparingStartPose = false;
        StartPoseFailed = true;
        if (!Ready(false) || calibrationSource == null ||
            !calibrationSource.TryGetCalibrationTargets(out requestedStartPose) ||
            !Finite(startSpeedDegrees) || startSpeedDegrees <= 0)
        { Status = "Invalid start pose configuration"; return; }
        for (int i = 0; i < 6; i++) targets[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
        startElapsed = settledTime = 0;
        PreparingStartPose = true;
        StartPoseFailed = false;
        Status = "Moving to A calibration pose";
    }
    private void UpdateStartPose()
    {
        if (!Ready(false) || !Finite(startSpeedDegrees) || startSpeedDegrees <= 0)
        { FailStartPose("Invalid configuration during start motion"); return; }
        startElapsed += Time.deltaTime;
        if (startElapsed > 30f) { FailStartPose("Start pose timed out. " + startDiagnostics); return; }
        bool settled = true;
        float largestError = 0;
        float largestSpeed = 0;
        int errorJoint = 0;
        for (int i = 0; i < 6; i++)
        {
            targets[i] = Mathf.MoveTowards(targets[i], requestedStartPose[i], startSpeedDegrees * Time.deltaTime);
            settled &= Mathf.Abs(targets[i] - requestedStartPose[i]) < .01f;
            var j = joints[i];
            float error = Mathf.Abs(j.jointPosition[0] * Mathf.Rad2Deg - requestedStartPose[i]);
            if (error > largestError) { largestError = error; errorJoint = i; }
            largestSpeed = Mathf.Max(largestSpeed, Mathf.Abs(j.jointVelocity[0] * Mathf.Rad2Deg));
            var drive = j.xDrive;
            drive.target = targets[i];
            drive.stiffness = 10000; drive.damping = 100; drive.forceLimit = 1000;
            j.xDrive = drive;
            settled &= Mathf.Abs(j.jointPosition[0] * Mathf.Rad2Deg - targets[i]) < 1f &&
                Mathf.Abs(j.jointVelocity[0] * Mathf.Rad2Deg) < 2f;
        }
        settledTime = settled ? settledTime + Time.deltaTime : 0;
        startDiagnostics = $"J{errorJoint+1} error {largestError:F1} deg; max speed {largestSpeed:F1} deg/s";
        Status = $"Moving to A calibration pose ({startElapsed:F0}/30 s)\n{startDiagnostics}\nWaiting: error < 1 deg, speed < 2 deg/s for 0.5 s";
        if (settledTime < .5f) return;
        PreparingStartPose = false;
        Status = "Start pose ready. Hold hand still and calibrate.";
        if (Mode == InputMode.TargetMarker) Calibrate();
    }
    private void FailStartPose(string message)
    {
        PreparingStartPose = false;
        StartPoseFailed = true;
        Status = message;
        foreach (var j in joints)
        {
            if (j == null || j.dofCount != 1 || !Finite(j.jointPosition[0])) continue;
            var drive = j.xDrive;
            drive.target = j.jointPosition[0] * Mathf.Rad2Deg;
            j.xDrive = drive;
        }
    }
    // Explicit recovery: use the measured pose, never pretend that A's start pose was reached.
    public void UseCurrentPose()
    {
        ResetRangeCalibration();
        calibrated = applied = false;
        if (!Ready(false))
        { Status = "Cannot use current pose: invalid robot configuration"; return; }
        PreparingStartPose = false;
        StartPoseFailed = false;
        for (int i = 0; i < 6; i++)
        {
            targets[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
            var drive = joints[i].xDrive;
            drive.target = targets[i];
            joints[i].xDrive = drive;
        }
        Status = "Current pose held (A start pose skipped). Calibrate when stopped.";
    }
    public void Calibrate()
    {
        if (PreparingStartPose || StartPoseFailed) return;
        if (Mode == InputMode.Mocopi && synchronizeWorkspace)
        {
            BeginRangeCalibration();
            return;
        }
        calibrated = applied = false;
        if (!Ready() || !Tip.IsChildOf(joints[5].transform) && Tip != joints[5].transform)
        { Status = "Invalid hand, joints or tool point"; return; }
        toolOrigin = new Pose(Tip.position, Tip.rotation);
        if (Mode == InputMode.TargetMarker)
        {
            if (ownedMarker == null)
            {
                ownedMarker = new GameObject("IK Target Marker");
                UnityEngine.SceneManagement.SceneManager.MoveGameObjectToScene(ownedMarker, gameObject.scene);
                TargetMarker = ownedMarker.transform;
            }
            ownedMarker.SetActive(true);
            TargetMarker.SetPositionAndRotation(toolOrigin.position, toolOrigin.rotation);
        }
        else
        {
            handOrigin = hand.position;
            handRotation = hand.rotation;
            // Capture the avatar heading at calibration; world up remains world up.
            inputToWorld = Quaternion.Inverse(Quaternion.Euler(0, animator.transform.eulerAngles.y, 0));
        }
        for (int i = 0; i < 6; i++) targets[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
        calibrated = true;
        Status = "Tracking";
    }
    private void OnDisable()
    {
        ResetRangeCalibration();
        PreparingStartPose = false;
        calibrated = applied = false;
        if (ownedMarker != null) ownedMarker.SetActive(false);
    }
    private void OnDestroy() { if (ownedMarker != null) Destroy(ownedMarker); }
    private bool TryGetTarget(out Pose target)
    {
        target = default;
        if (Mode == InputMode.TargetMarker)
        {
            if (TargetMarker == null || !TargetMarker.gameObject.activeInHierarchy) return false;
            target = new Pose(TargetMarker.position, followOrientation ? TargetMarker.rotation : toolOrigin.rotation);
        }
        else
        {
            if (hand == null) return false;
            Vector3 displacement = inputToWorld * (hand.position - handOrigin);
            if (synchronizeWorkspace)
            {
                if (RangePhase != WorkspacePhase.Tracking) return false;
                return TryGetReferenceTarget(out target);
            }
            else displacement *= positionScale;
            target = new Pose(toolOrigin.position + displacement,
                followOrientation ? inputToWorld * (hand.rotation * Quaternion.Inverse(handRotation)) * Quaternion.Inverse(inputToWorld) * toolOrigin.rotation : toolOrigin.rotation);
        }
        return Valid(target.position) && Valid(target.rotation);
    }
    private void OnDrawGizmos()
    {
        if (!isActiveAndEnabled || !showTrackingPoints || !HasTip()) return;
        DrawPoseGizmo(new Pose(Tip.position, Tip.rotation), Color.cyan, 0.009f);
        if (Mode == InputMode.Mocopi && synchronizeWorkspace &&
            (RangePhase == WorkspacePhase.Ready || RangePhase == WorkspacePhase.Tracking))
        {
            // Draw sampled axis segments, not a box falsely suggesting reachable corners.
            for (int axis = 0; axis < 3; axis++)
            {
                Vector3 low = toolOrigin.position, high = toolOrigin.position;
                low[axis] += robotMinimum[axis]; high[axis] += robotMaximum[axis];
                Gizmos.color = axis == 0 ? Color.red : axis == 1 ? Color.green : Color.blue;
                Gizmos.DrawLine(low, high);
                Gizmos.DrawWireSphere(low, .007f); Gizmos.DrawWireSphere(high, .007f);
            }
        }
        if (calibrated && TryGetTarget(out Pose target))
        {
            DrawPoseGizmo(target, Color.yellow, 0.016f);
            Gizmos.color = Color.yellow;
            Gizmos.DrawLine(Tip.position, target.position);
        }
        if (Mode == InputMode.Mocopi && hand != null)
        {
            DrawPoseGizmo(new Pose(hand.position, hand.rotation), Color.magenta, .012f);
            if (TryGetNeutralComparison(out Pose neutral, out Pose currentHand))
            {
                DrawPoseGizmo(neutral, Color.green, .02f);
                Gizmos.color = Color.green;
                Gizmos.DrawLine(neutral.position, currentHand.position);
            }
        }
    }
    private bool HasTip() => joints != null && joints.Length == 6 && joints[5] != null;
    private static void DrawPoseGizmo(Pose pose, Color color, float radius)
    {
        Gizmos.color = color; Gizmos.DrawWireSphere(pose.position, radius);
        Gizmos.color = Color.red; Gizmos.DrawLine(pose.position, pose.position + pose.rotation * Vector3.right * .06f);
        Gizmos.color = Color.green; Gizmos.DrawLine(pose.position, pose.position + pose.rotation * Vector3.up * .06f);
        Gizmos.color = Color.blue; Gizmos.DrawLine(pose.position, pose.position + pose.rotation * Vector3.forward * .06f);
    }
    // Screen overlays avoid renderer/shader dependencies and are visible without Game-view Gizmos.
    private void OnGUI()
    {
        Camera camera = Camera.main;
        if (!showTrackingPoints || !HasTip() || camera == null) return;
        DrawScreenPoint(camera, Tip.position, "ACTUAL", Color.cyan, 10, -24);
        if (TryGetNeutralComparison(out Pose neutral, out _))
            DrawScreenPoint(camera, neutral.position, "NEUTRAL", Color.green, 24, -48);
        if (calibrated && TryGetTarget(out Pose target))
            DrawScreenPoint(camera, target.position, "TARGET", Color.yellow, 18, 8);
        if (Mode == InputMode.Mocopi && hand != null)
            DrawScreenPoint(camera, hand.position, "HAND INPUT", Color.magenta, 12, 8);
    }
    private static void DrawScreenPoint(Camera camera, Vector3 world, string label, Color color, float size, float labelOffset)
    {
        if (!Valid(world)) return;
        Vector3 point = camera.WorldToScreenPoint(world);
        if (point.z <= 0 || !camera.pixelRect.Contains(new Vector2(point.x, point.y))) return;
        float x = point.x, y = Screen.height - point.y;
        Color previous = GUI.color;
        GUI.color = color;
        // Hollow target square lets coincident actual/target markers remain distinguishable.
        GUI.DrawTexture(new Rect(x-size/2, y-size/2, size, 2), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(x-size/2, y+size/2-2, size, 2), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(x-size/2, y-size/2, 2, size), Texture2D.whiteTexture);
        GUI.DrawTexture(new Rect(x+size/2-2, y-size/2, 2, size), Texture2D.whiteTexture);
        GUI.color = Color.black;
        GUI.DrawTexture(new Rect(x+12, y+labelOffset, 105, 22), Texture2D.whiteTexture);
        GUI.color = color;
        GUI.Label(new Rect(x+15, y+labelOffset, 105, 22), label);
        GUI.color = previous;
    }
    private void LateUpdate()
    {
        applied = false;
        if (PreparingStartPose) { UpdateStartPose(); return; }
        if (!calibrated) return;
        if (!Ready()) { Status = "Invalid input - holding"; return; }
        if (Mode == InputMode.Mocopi && synchronizeWorkspace && !RangeConfigurationValid())
        {
            calibrated = false;
            RangePhase = WorkspacePhase.Error;
            Status = RangeMessage = "Tool, orientation or limits changed. Restart range calibration.";
            return;
        }
        if (!TryGetTarget(out Pose target)) { Status = "Invalid target - holding"; return; }
        Vector3[] pivots = new Vector3[6], axes = new Vector3[6];
        float[] current = new float[6], seed = new float[6], lower = new float[6], upper = new float[6];
        for (int i = 0; i < 6; i++)
        {
            var j = joints[i];
            pivots[i] = j.transform.TransformPoint(j.anchorPosition);
            axes[i] = j.transform.rotation * j.anchorRotation * Vector3.right;
            current[i] = j.jointPosition[0];
            seed[i] = targets[i] * Mathf.Deg2Rad;
            lower[i] = j.twistLock == ArticulationDofLock.FreeMotion ? -Mathf.PI : j.xDrive.lowerLimit * Mathf.Deg2Rad;
            upper[i] = j.twistLock == ArticulationDofLock.FreeMotion ? Mathf.PI : j.xDrive.upperLimit * Mathf.Deg2Rad;
        }
        Pose actual = new Pose(Tip.position, Tip.rotation);
        float[] solution = ArmPoseSolver.Solve(pivots, axes, current, seed, lower, upper, actual, target);
        for (int i = 0; i < 6; i++) if (!Finite(solution[i])) { Status = "IK failed - holding"; return; }
        if (Mode == InputMode.Mocopi && synchronizeWorkspace)
        {
            ArmPoseSolver.Forward(pivots, axes, current, solution, actual, out Pose predicted, out _, out _);
            if (!PoseWithinTolerance(predicted, target) && referencesReady &&
                TrySolveFromReferences(pivots, axes, current, lower, upper, actual, target, out float[] referenceSolution))
            {
                solution = referenceSolution;
                ArmPoseSolver.Forward(pivots, axes, current, solution, actual, out predicted, out _, out _);
            }
            if (!PoseWithinTolerance(predicted, target))
            {
                PositionError = Vector3.Distance(actual.position, target.position);
                OrientationError = Quaternion.Angle(actual.rotation, target.rotation);
                Status = "Target outside estimated reachable workspace - holding. Move hand back.";
                return;
            }
        }
        for (int i = 0; i < 6; i++)
        {
            targets[i] = Mathf.Clamp(Mathf.MoveTowards(targets[i], solution[i] * Mathf.Rad2Deg, maxJointSpeed * Time.deltaTime), lower[i]*Mathf.Rad2Deg, upper[i]*Mathf.Rad2Deg);
            var drive = joints[i].xDrive;
            drive.stiffness = 10000; drive.damping = 100; drive.forceLimit = 1000;
            drive.target = targets[i]; joints[i].xDrive = drive;
        }
        PositionError = Vector3.Distance(actual.position, target.position);
        OrientationError = Quaternion.Angle(actual.rotation, target.rotation);
        Status = PositionError < 0.01f && OrientationError < 5f ? "Tracking" : "Converging / target may be unreachable";
        applied = true;
    }
}
