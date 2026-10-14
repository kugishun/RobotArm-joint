using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Installs in scenes containing the legacy mapping, before any Start/LateUpdate.</summary>
[DefaultExecutionOrder(-1000)]
public class RobotControlMenu : MonoBehaviour
{
    private UpperArmToUrdfRobotJoints legacy;
    private HandPoseIkController ik;
    private readonly System.Collections.Generic.List<MyCobotUdpJointSender> senders = new System.Collections.Generic.List<MyCobotUdpJointSender>();
    private bool choosing = true, allowUdp;
    private string mode = "Choose control method";
    private Vector2 menuScroll;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Register()
    {
        SceneManager.sceneLoaded -= Install;
        SceneManager.sceneLoaded += Install;
    }
    private static void Install(Scene scene, LoadSceneMode loadMode)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        foreach (var source in root.GetComponentsInChildren<UpperArmToUrdfRobotJoints>(true))
        {
            if (!source.gameObject.activeInHierarchy || source.GetComponent<RobotControlMenu>() != null) continue;
            source.gameObject.AddComponent<RobotControlMenu>().Initialize(source);
        }
    }
    private void Initialize(UpperArmToUrdfRobotJoints source)
    {
        legacy = source;
        legacy.PrepareForSelection();
        legacy.enabled = false;
        ik = source.GetComponent<HandPoseIkController>();
        if (ik == null) ik = source.gameObject.AddComponent<HandPoseIkController>();
        ik.Configure(source);
        ik.enabled = false;
        // The importer keyboard controller creates JointControl components in Start.
        // Disable it before Start so it cannot overwrite the selected controller's drives.
        var chain = source.RobotChain;
        if (chain[0] != null)
        foreach (var behaviour in chain[0].transform.root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            string type = behaviour.GetType().FullName;
            if (type == "Unity.Robotics.UrdfImporter.Control.Controller" || type == "JointControl")
                behaviour.enabled = false;
        }
        HoldSimulationPose();
        foreach (var candidate in FindObjectsByType<MyCobotUdpJointSender>(FindObjectsInactive.Include, FindObjectsSortMode.None)) if (candidate.UsesSource(source)) senders.Add(candidate);
        foreach (var sender in senders) sender.MenuAllowsSending = false;
    }
    private void Choose(bool useIk, bool markerInput = false)
    {
        legacy.PrepareForSelection();
        legacy.enabled = !useIk;
        ik.SetInputMode(markerInput ? HandPoseIkController.InputMode.TargetMarker : HandPoseIkController.InputMode.Mocopi);
        ik.enabled = useIk;
        foreach (var sender in senders)
        {
            sender.SelectIkSource(useIk ? ik : null);
            sender.MenuAllowsSending = allowUdp && !markerInput;
        }
        mode = markerInput ? "IK target marker (simulation only)" : useIk ? "IK hand pose" : "Original joint mapping";
        choosing = false;
        if (useIk) ik.BeginStartPose();
        // Calibration is explicit after Start and after mocopi has supplied a pose.
    }
    private void ReturnToMenu()
    {
        foreach (var sender in senders) sender.MenuAllowsSending = false;
        legacy.PrepareForSelection();
        legacy.enabled = false;
        ik.enabled = false;
        HoldSimulationPose();
        choosing = true;
    }
    private void HoldSimulationPose()
    {
        foreach (var joint in legacy.RobotChain)
        {
            if (joint == null || joint.jointType != ArticulationJointType.RevoluteJoint || joint.dofCount != 1) continue;
            float angle = joint.jointPosition[0] * Mathf.Rad2Deg;
            if (!HandPoseIkController.Finite(angle)) continue;
            var drive = joint.xDrive;
            drive.target = angle;
            drive.stiffness = 10000; drive.damping = 100; drive.forceLimit = 1000;
            joint.xDrive = drive;
        }
    }
    private void OnGUI()
    {
        if (legacy == null || ik == null)
        {
            GUI.Box(new Rect(10, 10, 420, 65), "Control menu not initialized.\nStop Play mode, wait for compilation, then Play again.");
            return;
        }
        float width = Mathf.Min(420, Screen.width - 20);
        Rect area = choosing ? new Rect((Screen.width-width)/2, 30, width, Mathf.Min(390, Screen.height-40)) : new Rect(Screen.width-width-10, 10, width, Mathf.Min(620, Screen.height-20));
        GUILayout.BeginArea(area, GUI.skin.box);
        menuScroll = GUILayout.BeginScrollView(menuScroll);
        GUILayout.Label(choosing ? "Robot control - select method" : mode);
        if (choosing)
        {
            GUILayout.Label("Choose a method, then hold your hand still and calibrate.");
            GUILayout.Label("IK tracks displacement from calibration (Unity world axes).\nThe default tool point is the J6 flange.");
            ik.followOrientation = GUILayout.Toggle(ik.followOrientation, "IK: follow hand orientation (off = fixed orientation)");
            ik.synchronizeWorkspace = GUILayout.Toggle(ik.synchronizeWorkspace, "Mocopi: UP / FRONT / RIGHT reference mapping");
            if (!ik.synchronizeWorkspace)
            {
                GUILayout.Label($"IK movement scale: {ik.positionScale:F2}");
                ik.positionScale = GUILayout.HorizontalSlider(ik.positionScale, 0.1f, 1f);
            }
            else GUILayout.Label("Begin range capture -> UP -> FRONT -> RIGHT -> choose neutral and start.");
            if (legacy.TryGetCalibrationTargets(out float[] startPose))
                GUILayout.Label($"IK start pose from A (J1-J6):\n{string.Join(", ", System.Array.ConvertAll(startPose, angle => angle.ToString("F0")))} deg");
            else GUILayout.Label("A calibration pose is invalid. Check joint settings.");
            allowUdp = GUILayout.Toggle(allowUdp, "Enable physical robot UDP output");
            if (GUILayout.Button("Original joint mapping (A)", GUILayout.Height(36))) Choose(false);
            if (GUILayout.Button("Hand position / orientation IK (B)", GUILayout.Height(36))) Choose(true);
            if (GUILayout.Button("IK test: target marker (no mocopi)", GUILayout.Height(36))) Choose(true, true);
        }
        else
        {
            string status = ik.enabled ? ik.Status : (legacy.IsCalibrated ? "Tracking" : "Calibrate to begin");
            if (string.IsNullOrEmpty(status)) status = "State unavailable. Stop Play mode and restart.";
            GUILayout.Label("Status: " + status);
            if (ik.enabled) GUILayout.Label($"Position error: {ik.PositionError * 1000:F1} mm\nOrientation error: {ik.OrientationError:F1} deg");
            if (ik.enabled)
            {
                ik.showTrackingPoints = GUILayout.Toggle(ik.showTrackingPoints, "Show tracking points");
                GUILayout.Label($"Tracking point: {ik.TrackingPointName}\nCyan: ACTUAL / Yellow: TARGET\nMagenta: HAND INPUT / Green: NEUTRAL");
            }
            bool markerInput = ik.enabled && ik.Mode == HandPoseIkController.InputMode.TargetMarker;
            GUILayout.Label(allowUdp && !markerInput ? "UDP output enabled" : "Unity simulation only");
            if (ik.enabled && ik.StartPoseFailed && GUILayout.Button("Retry start pose")) ik.BeginStartPose();
            if (ik.enabled && (ik.PreparingStartPose || ik.StartPoseFailed))
            {
                GUILayout.Label("Calibration waits for the robot to reach and settle at A's pose.\nTo proceed from its present pose instead, cancel the start move:");
                if (GUILayout.Button("Use current pose (cancel start move)", GUILayout.Height(32))) ik.UseCurrentPose();
            }
            if (markerInput && !ik.PreparingStartPose && !ik.StartPoseFailed)
            {
                GUILayout.Label("Use the buttons below to move the target.\nScene view: W/E select tools; drag arrows/rings to move.\nW/E alone do not move the marker.");
#if UNITY_EDITOR
                if (GUILayout.Button("Select target marker") && ik.TargetMarker != null)
                {
                    UnityEditor.Selection.activeGameObject = ik.TargetMarker.gameObject;
                    UnityEditor.Tools.current = UnityEditor.Tool.Move;
                    var sceneView = UnityEditor.SceneView.lastActiveSceneView;
                    if (sceneView != null) { sceneView.Show(); sceneView.Focus(); sceneView.FrameSelected(); }
                }
#endif
                DrawMarkerControls();
            }
            bool controlsEnabled = GUI.enabled;
            GUI.enabled = controlsEnabled && !(ik.enabled && (ik.PreparingStartPose || ik.StartPoseFailed));
            bool rangeInput = ik.enabled && !markerInput && ik.synchronizeWorkspace;
            if (rangeInput) DrawRangeCalibration();
            if (GUILayout.Button(markerInput ? "Reset marker to current tool pose" : rangeInput ? "Begin / restart range calibration" : "Calibrate / recenter", GUILayout.Height(32)))
            {
                if (ik.enabled) ik.Calibrate(); else legacy.Calibrate();
            }
            GUI.enabled = controlsEnabled;
            if (GUILayout.Button("Change control method", GUILayout.Height(32))) ReturnToMenu();
        }
        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }
    private void DrawRangeCalibration()
    {
        GUILayout.Label("Workspace calibration: " + ik.RangePhase);
        GUILayout.Label(ik.NeutralComparisonText);
        GUILayout.Label(ik.RangeMessage);
        string capture = null;
        switch (ik.RangePhase)
        {
            case HandPoseIkController.WorkspacePhase.Raise: capture = "1. Capture arm UP"; break;
            case HandPoseIkController.WorkspacePhase.Forward: capture = "2. Capture arm FRONT"; break;
            case HandPoseIkController.WorkspacePhase.Side: capture = "3. Capture arm RIGHT"; break;
        }
        if (capture != null && GUILayout.Button(capture, GUILayout.Height(36))) ik.CaptureRangePose();
        if (ik.RangePhase != HandPoseIkController.WorkspacePhase.None)
        {
            GUILayout.Label($"Human observed min (m): {ik.HumanMinimum.ToString("F2")}\nHuman observed max (m): {ik.HumanMaximum.ToString("F2")}");
            GUILayout.Label($"Robot reference min (m): {ik.RobotMinimum.ToString("F2")}\nRobot reference max (m): {ik.RobotMaximum.ToString("F2")}");
            GUILayout.Label("UP: 0,0,0,90,90,0\nFRONT: 0,-90,0,90,90,0\nRIGHT: -90,-90,0,90,90,0\nReference points, not maximum workspace bounds.");
        }
        string blockedReason = ik.RangeStartBlockReason;
        bool previousEnabled = GUI.enabled;
        GUI.enabled = previousEnabled && string.IsNullOrEmpty(blockedReason);
        if (GUILayout.Button("Set current pose as neutral and start", GUILayout.Height(36))) ik.StartRangeTracking();
        GUI.enabled = previousEnabled;
        GUILayout.Label(string.IsNullOrEmpty(blockedReason) ? "This hand pose will be recorded as neutral.\nOn start, the robot moves toward its mapped position." : blockedReason);
    }
    private void DrawMarkerControls()
    {
        Transform marker = ik.TargetMarker;
        if (marker == null)
        {
            GUILayout.Label("Marker not created. Check status above, then reset.");
            return;
        }
        Vector3 p = marker.position;
        GUILayout.Label($"Target world position (m): {p.x:F3}, {p.y:F3}, {p.z:F3}");
        GUILayout.Label("Move: 1 cm per click (world axes)");
        DrawMarkerAxis(marker, "X", Vector3.right, false);
        DrawMarkerAxis(marker, "Y", Vector3.up, false);
        DrawMarkerAxis(marker, "Z", Vector3.forward, false);
        GUILayout.Label("Rotate: 5 degrees per click (world axes)");
        bool previousEnabled = GUI.enabled;
        GUI.enabled = previousEnabled && ik.followOrientation;
        DrawMarkerAxis(marker, "X", Vector3.right, true);
        DrawMarkerAxis(marker, "Y", Vector3.up, true);
        DrawMarkerAxis(marker, "Z", Vector3.forward, true);
        GUI.enabled = previousEnabled;
        if (!ik.followOrientation) GUILayout.Label("Rotation locked: enable orientation in the start menu.");
        if (Time.timeScale == 0f) GUILayout.Label("Time scale is zero. Resume simulation to track the target.");
    }
    private static void DrawMarkerAxis(Transform marker, string label, Vector3 axis, bool rotate)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, GUILayout.Width(25));
        int direction = 0;
        if (GUILayout.Button("-", GUILayout.Height(26))) direction = -1;
        if (GUILayout.Button("+", GUILayout.Height(26))) direction = 1;
        if (direction != 0)
        {
            if (rotate) marker.rotation = Quaternion.AngleAxis(direction * 5f, axis) * marker.rotation;
            else marker.position += axis * (direction * 0.01f);
        }
        GUILayout.EndHorizontal();
    }
    private void OnDisable()
    {
        foreach (var sender in senders) sender.MenuAllowsSending = false;
        if (legacy != null) { legacy.PrepareForSelection(); legacy.enabled = false; }
        if (ik != null) ik.enabled = false;
    }
}

