using System.Collections;
using UnityEngine;

public partial class HandPoseIkController
{
    [Header("Human / robot workspace calibration")]
    public bool synchronizeWorkspace = true;
    public enum WorkspacePhase { None, Raise, Forward, Side, Estimating, Ready, Tracking, Error }
    public WorkspacePhase RangePhase { get; private set; }
    public Vector3 HumanMinimum => humanMinimum;
    public Vector3 HumanMaximum => humanMaximum;
    public Vector3 RobotMinimum => robotMinimum;
    public Vector3 RobotMaximum => robotMaximum;
    public string RangeMessage { get; private set; } = "";
    private Vector3 humanMinimum, humanMaximum, robotMinimum, robotMaximum;
    private Coroutine rangeRoutine;
    private readonly Vector3[] referenceHuman = new Vector3[3];
    private readonly Pose[] referenceRobot = new Pose[3];
    private readonly Quaternion[] referenceHandRotations = new Quaternion[3];
    private Vector3 measurementHandOrigin;
    private bool referencesReady;
    private Transform calibrationTorso;
    private Vector3 torsoOrigin;
    private Quaternion torsoRotation;
    private bool neutralCaptured;
    public bool TryGetNeutralComparison(out Pose neutral, out Pose current)
    {
        neutral = new Pose(handOrigin, handRotation);
        current = default;
        if (!neutralCaptured || Mode != InputMode.Mocopi || hand == null ||
            !Valid(hand.position) || !Valid(hand.rotation)) return false;
        current = new Pose(hand.position, hand.rotation);
        return true;
    }
    public string NeutralComparisonText
    {
        get
        {
            if (!TryGetNeutralComparison(out Pose neutral, out Pose current))
                return "After range calibration, set your current comfortable hand pose as neutral.";
            Vector3 difference = inputToWorld * (current.position - neutral.position);
            float centimeters = Vector3.Distance(neutral.position, current.position) * 100f;
            float degrees = Quaternion.Angle(neutral.rotation, current.rotation);
            return $"NEUTRAL position (world m): {neutral.position.ToString("F3")}\n" +
                $"CURRENT position (world m): {current.position.ToString("F3")}\n" +
                $"NEUTRAL rotation (world deg): {neutral.rotation.eulerAngles.ToString("F1")}\n" +
                $"CURRENT rotation (world deg): {current.rotation.eulerAngles.ToString("F1")}\n" +
                $"Current - neutral (right/up/front cm): {(difference * 100f).ToString("F1")}\n" +
                $"Distance from neutral: {centimeters:F1} cm\n" +
                $"Rotation from neutral: {degrees:F1} deg\n" +
                "Reference display only. Returning to this pose is not required.";
        }
    }
    private bool rangeOrientationMode;
    private Transform rangeTool;
    private float[] rangeLower, rangeUpper;
    private const float MinimumHumanSpan = .05f;
    private const float ProbeStep = .01f;

    private void ResetRangeCalibration()
    {
        referencesReady = false;
        neutralCaptured = false;
        if (rangeRoutine != null) StopCoroutine(rangeRoutine);
        rangeRoutine = null;
        RangePhase = WorkspacePhase.None;
        RangeMessage = "";
    }

    private void BeginRangeCalibration()
    {
        ResetRangeCalibration();
        calibrated = applied = false;
        if (!Ready() || !Tip.IsChildOf(joints[5].transform) && Tip != joints[5].transform)
        { RangePhase = WorkspacePhase.Error; Status = RangeMessage = "Invalid hand or robot. Check configuration."; return; }
        // Hold the measured pose throughout human boundary capture and numerical probing.
        for (int i = 0; i < 6; i++)
        {
            targets[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
            var drive = joints[i].xDrive; drive.target = targets[i]; joints[i].xDrive = drive;
        }
        toolOrigin = new Pose(Tip.position, Tip.rotation);
        handOrigin = hand.position;
        measurementHandOrigin = handOrigin;
        handRotation = hand.rotation;
        neutralCaptured = false; // Temporary measurement origin; final neutral is chosen after range capture.
        inputToWorld = Quaternion.Inverse(Quaternion.Euler(0, animator.transform.eulerAngles.y, 0));
        calibrationTorso = animator.GetBoneTransform(HumanBodyBones.Hips);
        if (calibrationTorso == null) calibrationTorso = animator.transform;
        torsoOrigin = calibrationTorso.position;
        torsoRotation = calibrationTorso.rotation;
        humanMinimum = humanMaximum = robotMinimum = robotMaximum = Vector3.zero;
        rangeOrientationMode = followOrientation;
        rangeTool = Tip;
        RangePhase = WorkspacePhase.Raise;
        Status = RangeMessage = "Range reference captured. Raise your right arm, hold still, then capture UP.";
    }

    public void CaptureRangePose()
    {
        if (RangePhase != WorkspacePhase.Raise && RangePhase != WorkspacePhase.Forward && RangePhase != WorkspacePhase.Side) return;
        if (!Ready()) { RangeMessage = "Hand input unavailable. Try again."; return; }
        if (Vector3.Distance(calibrationTorso.position, torsoOrigin) > .05f || Quaternion.Angle(calibrationTorso.rotation, torsoRotation) > 15f)
        { RangeMessage = "Keep torso at the neutral position and direction, then retry."; return; }
        Vector3 displacement = inputToWorld * (hand.position - handOrigin);
        int axis = RangePhase == WorkspacePhase.Raise ? 1 : RangePhase == WorkspacePhase.Forward ? 2 : 0;
        if (displacement[axis] < MinimumHumanSpan)
        { RangeMessage = "Move at least 5 cm in the requested positive direction (UP / FRONT / RIGHT)."; return; }
        humanMinimum = Vector3.Min(humanMinimum, displacement);
        referenceHuman[RangePhase == WorkspacePhase.Raise ? 0 : RangePhase == WorkspacePhase.Forward ? 1 : 2] = displacement;
        referenceHandRotations[RangePhase == WorkspacePhase.Raise ? 0 : RangePhase == WorkspacePhase.Forward ? 1 : 2] = hand.rotation;
        humanMaximum = Vector3.Max(humanMaximum, displacement);
        // Small negative excursions are noise, not measured lower bounds.
        for (int i = 0; i < 3; i++) if (humanMinimum[i] > -MinimumHumanSpan) humanMinimum[i] = 0;
        if (RangePhase == WorkspacePhase.Raise)
        { RangePhase = WorkspacePhase.Forward; RangeMessage = "Extend right arm forward, hold still, then capture FRONT."; }
        else if (RangePhase == WorkspacePhase.Forward)
        { RangePhase = WorkspacePhase.Side; RangeMessage = "Extend right arm to your right side, hold still, then capture RIGHT."; }
        else
        {
            RangePhase = WorkspacePhase.Estimating;
            RangeMessage = "Computing UP / FRONT / RIGHT robot reference poses.";
            rangeRoutine = StartCoroutine(EstimateRobotWorkspace());
        }
        Status = RangeMessage;
    }

    // Piecewise normalization preserves neutral and independently maps each observed side.
    // A side with no human measurement is locked at neutral, never invented by mirroring.
    public static Vector3 MapWorkspaceDisplacement(Vector3 value, Vector3 humanMin, Vector3 humanMax, Vector3 robotMin, Vector3 robotMax)
    {
        Vector3 result = Vector3.zero;
        for (int i = 0; i < 3; i++)
        {
            if (value[i] >= 0 && humanMax[i] >= MinimumHumanSpan)
                result[i] = Mathf.Clamp01(value[i] / humanMax[i]) * robotMax[i];
            else if (value[i] < 0 && humanMin[i] <= -MinimumHumanSpan)
                result[i] = Mathf.Clamp01(value[i] / humanMin[i]) * robotMin[i];
        }
        return result;
    }

    private static bool PoseWithinTolerance(Pose actual, Pose target) =>
        Valid(actual.position) && Valid(actual.rotation) &&
        Vector3.Distance(actual.position, target.position) <= .003f &&
        Quaternion.Angle(actual.rotation, target.rotation) <= 2f;

    private bool RangeConfigurationValid()
    {
        if (Tip != rangeTool || followOrientation != rangeOrientationMode || rangeLower == null || rangeUpper == null) return false;
        for (int i = 0; i < 6; i++)
        {
            var j = joints[i];
            float low = j.twistLock == ArticulationDofLock.FreeMotion ? -Mathf.PI : j.xDrive.lowerLimit * Mathf.Deg2Rad;
            float high = j.twistLock == ArticulationDofLock.FreeMotion ? Mathf.PI : j.xDrive.upperLimit * Mathf.Deg2Rad;
            if (Mathf.Abs(low - rangeLower[i]) > .0001f || Mathf.Abs(high - rangeUpper[i]) > .0001f) return false;
        }
        return true;
    }

    private IEnumerator EstimateRobotWorkspace()
    {
        if (!Ready(false)) { RangePhase = WorkspacePhase.Error; RangeMessage = "Invalid robot geometry."; yield break; }
        Vector3[] pivots = new Vector3[6], axes = new Vector3[6];
        float[] reference = new float[6];
        rangeLower = new float[6]; rangeUpper = new float[6];
        for (int i = 0; i < 6; i++)
        {
            var j = joints[i];
            pivots[i] = j.transform.TransformPoint(j.anchorPosition);
            axes[i] = j.transform.rotation * j.anchorRotation * Vector3.right;
            reference[i] = j.jointPosition[0];
            rangeLower[i] = j.twistLock == ArticulationDofLock.FreeMotion ? -Mathf.PI : j.xDrive.lowerLimit * Mathf.Deg2Rad;
            rangeUpper[i] = j.twistLock == ArticulationDofLock.FreeMotion ? Mathf.PI : j.xDrive.upperLimit * Mathf.Deg2Rad;
        }
        Pose referenceTip = new Pose(Tip.position, Tip.rotation);
        if (!BuildSpecifiedRobotReferences(pivots, axes, reference, referenceTip))
        {
            RangePhase = WorkspacePhase.Error;
            Status = RangeMessage = "Invalid reference angles or nearly collinear human poses. Restart calibration.";
            yield break;
        }
        RangePhase = WorkspacePhase.Ready;
        Status = RangeMessage = "References ready. Choose a comfortable hand pose, then start.";
        rangeRoutine = null;
        yield break;
    }

    public string RangeStartBlockReason
    {
        get
        {
            if (!isActiveAndEnabled || Mode != InputMode.Mocopi || !synchronizeWorkspace)
                return "Workspace tracking is not selected.";
            if (PreparingStartPose || StartPoseFailed) return "Complete robot start pose preparation first.";
            if (RangePhase == WorkspacePhase.Tracking) return "Tracking is already active.";
            if (RangePhase != WorkspacePhase.Ready) return "Complete UP, FRONT, RIGHT captures and range estimation.";
            if (!Ready()) return "Valid hand and robot input required.";
            if (!RangeConfigurationValid() ||
                Vector3.Distance(Tip.position, toolOrigin.position) > .01f || Quaternion.Angle(Tip.rotation, toolOrigin.rotation) > 3f)
                return "Robot/tool settings or pose changed. Restart calibration.";
            return "";
        }
    }
    public bool CanStartRangeTracking => string.IsNullOrEmpty(RangeStartBlockReason);

    public void StartRangeTracking()
    {
        // Use the same live conditions as the button, and recheck at click time.
        string reason = RangeStartBlockReason;
        if (!string.IsNullOrEmpty(reason)) { RangeMessage = reason; return; }
        // Preserve the captured endpoint positions; only their reference origin changes.
        Vector3 shift = inputToWorld * (hand.position - handOrigin);
        humanMinimum = Vector3.Min(Vector3.zero, humanMinimum - shift);
        humanMaximum = Vector3.Max(Vector3.zero, humanMaximum - shift);
        handOrigin = hand.position;
        handRotation = hand.rotation;
        neutralCaptured = true;
        for (int i = 0; i < 6; i++) targets[i] = joints[i].jointPosition[0] * Mathf.Rad2Deg;
        calibrated = true;
        applied = false;
        RangePhase = WorkspacePhase.Tracking;
        Status = RangeMessage = "Tracking with calibrated workspace scaling";
    }
}
