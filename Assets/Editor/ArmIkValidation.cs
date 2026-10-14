using System;
using System.IO;
using UnityEditor;
using UnityEngine;

public static class ArmIkValidation
{
    [MenuItem("Tools/Robot Arm/Validate IK Solver")]
    public static void Run()
    {
        // Pure geometry tests: do not load a scene, enter Play, or send UDP.
        Vector3[] p = { Vector3.zero, new Vector3(0,.1f,0), new Vector3(0,.3f,0), new Vector3(0,.5f,0), new Vector3(0,.55f,0), new Vector3(0,.6f,0) };
        Vector3[] a = { Vector3.up, Vector3.right, Vector3.right, Vector3.right, Vector3.up, Vector3.forward };
        float[] zero = new float[6], low = new float[6], high = new float[6];
        for (int i=0;i<6;i++) { low[i]=-2.8f; high[i]=2.8f; }
        Pose rest = new Pose(new Vector3(0,.7f,0), Quaternion.identity);
        ArmPoseSolver.Forward(new[]{Vector3.zero},new[]{Vector3.forward},new[]{0f},new[]{Mathf.PI/2},
            new Pose(Vector3.right,Quaternion.identity),out Pose rotated,out _,out _);
        Require(Vector3.Distance(rotated.position,Vector3.up)<1e-5f,"Known 90 degree forward kinematics");
        float[] reference = { .3f,-.5f,1f,-.4f,.4f,.2f };
        ArmPoseSolver.Forward(p,a,zero,reference,rest,out Pose tip,out Vector3[] rp,out Vector3[] ra);
        float[] wanted = { .36f,-.55f,1.1f,-.45f,.46f,.24f };
        ArmPoseSolver.Forward(p,a,zero,wanted,rest,out Pose goal,out _,out _);
        float[] result = ArmPoseSolver.Solve(rp,ra,reference,reference,low,high,tip,goal,100);
        ArmPoseSolver.Forward(rp,ra,reference,result,tip,out Pose reached,out _,out _);
        Require(Vector3.Distance(reached.position,goal.position)<.002f,"Reachable position");
        Require(Quaternion.Angle(reached.rotation,goal.rotation)<1f,"Reachable orientation");
        // Capturing an already rotated chain must preserve the same FK.
        ArmPoseSolver.Forward(rp,ra,reference,wanted,tip,out Pose recaptured,out _,out _);
        Require(Vector3.Distance(recaptured.position,goal.position)<1e-5f,"Nonzero reference geometry");
        Require(Quaternion.Angle(recaptured.rotation,goal.rotation)<.05f,"Nonzero reference rotation");
        goal.position = Vector3.one * 100;
        result = ArmPoseSolver.Solve(p,a,zero,zero,low,high,rest,goal);
        for(int i=0;i<6;i++) Require(HandPoseIkController.Finite(result[i]) && result[i]>=low[i] && result[i]<=high[i],"Unreachable target remains finite and limited");
        Vector3 error = ArmPoseSolver.RotationError(Quaternion.AngleAxis(-179,Vector3.up),Quaternion.AngleAxis(179,Vector3.up));
        Require(Mathf.Abs(error.magnitude*Mathf.Rad2Deg-2)<.01f,"Quaternion boundary uses short rotation");
        ValidateMarkerInput();
        ValidateWorkspaceMapping();
        Directory.CreateDirectory("Logs");
        File.WriteAllText("Logs/ArmIkValidation.txt","PASS: known FK; reachable pose; nonzero reference; unreachable limits; quaternion boundary. No runtime/hardware tests.");
        Debug.Log("IK geometry validation passed.");
    }
    private static void Require(bool pass,string name) { if(!pass) throw new Exception("IK validation failed: "+name); }
    [MenuItem("Tools/Robot Arm/Validate Workspace Mapping")]
    public static void ValidateWorkspaceMapping()
    {
        Vector3 humanMin = new Vector3(-.2f, -.3f, 0);
        Vector3 humanMax = new Vector3(.5f, .6f, .4f);
        Vector3 robotMin = new Vector3(-.1f, -.15f, -.2f);
        Vector3 robotMax = new Vector3(.25f, .3f, .2f);
        Func<Vector3, Vector3> map = value => HandPoseIkController.MapWorkspaceDisplacement(value, humanMin, humanMax, robotMin, robotMax);
        Require(map(Vector3.zero) == Vector3.zero, "Range neutral preserved");
        Require(Vector3.Distance(map(humanMax), robotMax) < 1e-6f, "Positive endpoints synchronized");
        Require(Vector3.Distance(map(humanMax * .5f), robotMax * .5f) < 1e-6f, "Half-range per axis");
        Require(Vector3.Distance(map(humanMax * 3f), robotMax) < 1e-6f, "Human input saturates at boundary");
        Require(Vector3.Distance(map(new Vector3(-.4f, -.6f, -1)), new Vector3(-.1f, -.15f, 0)) < 1e-6f, "Negative bounds and unmeasured side");
        Vector3 tiny = HandPoseIkController.MapWorkspaceDisplacement(Vector3.one, Vector3.zero, Vector3.one * .001f, robotMin, robotMax);
        Require(tiny == Vector3.zero, "Degenerate spans do not amplify noise");
        Vector3 h0 = Vector3.up, h1 = Vector3.forward, h2 = Vector3.right;
        Vector3 r0 = new Vector3(.1f,.5f,0), r1 = new Vector3(.1f,.1f,.4f), r2 = new Vector3(.5f,.1f,0);
        Require(Vector3.Distance(HandPoseIkController.MapThreeReferencePositions(h0,h0,h1,h2,r0,r1,r2,out _),r0)<1e-5f,"UP reference exact");
        Require(Vector3.Distance(HandPoseIkController.MapThreeReferencePositions(h1,h0,h1,h2,r0,r1,r2,out _),r1)<1e-5f,"FRONT reference exact");
        Require(Vector3.Distance(HandPoseIkController.MapThreeReferencePositions(h2,h0,h1,h2,r0,r1,r2,out _),r2)<1e-5f,"RIGHT reference exact");
        Require(Vector3.Distance(HandPoseIkController.MapThreeReferencePositions((h0+h1)*.5f,h0,h1,h2,r0,r1,r2,out _),(r0+r1)*.5f)<1e-5f,"Reference interpolation");
        var host = new GameObject("Workspace lifecycle test");
        try
        {
            var controller = host.AddComponent<HandPoseIkController>();
            var property = typeof(HandPoseIkController).GetProperty("RangePhase");
            property.SetValue(controller, HandPoseIkController.WorkspacePhase.Estimating);
            controller.SetInputMode(HandPoseIkController.InputMode.TargetMarker);
            Require(controller.RangePhase == HandPoseIkController.WorkspacePhase.None, "Input switch invalidates measured ranges");
            Require(!controller.CanSendJointTargets, "No output while uncalibrated");
        }
        finally { UnityEngine.Object.DestroyImmediate(host); }
        Debug.Log("Workspace mapping validation passed.");
    }
    private static void ValidateMarkerInput()
    {
        var host = new GameObject("IK input test");
        var marker = new GameObject("Marker test");
        try
        {
            var controller = host.AddComponent<HandPoseIkController>();
            controller.SetInputMode(HandPoseIkController.InputMode.TargetMarker);
            typeof(HandPoseIkController).GetProperty("TargetMarker").SetValue(controller, marker.transform);
            marker.transform.SetPositionAndRotation(new Vector3(.1f, .2f, .3f), Quaternion.Euler(20,30,40));
            var method = typeof(HandPoseIkController).GetMethod("TryGetTarget", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic);
            object[] args = { default(Pose) };
            Require((bool)method.Invoke(controller, args), "Marker input works without Animator");
            Pose pose = (Pose)args[0];
            Require(Vector3.Distance(pose.position, marker.transform.position) < 1e-6f, "Marker position is unscaled world position");
            Require(Quaternion.Angle(pose.rotation, marker.transform.rotation) < .05f, "Marker orientation");
            typeof(HandPoseIkController).GetField("toolOrigin", System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic).SetValue(controller, new Pose(Vector3.zero, Quaternion.identity));
            controller.followOrientation = false;
            Require((bool)method.Invoke(controller, args) && Quaternion.Angle(((Pose)args[0]).rotation, Quaternion.identity) < .05f, "Fixed orientation ignores marker rotation");
            foreach (string field in new[] { "calibrated", "applied" })
                typeof(HandPoseIkController).GetField(field, System.Reflection.BindingFlags.Instance | System.Reflection.BindingFlags.NonPublic).SetValue(controller, true);
            Require(!controller.CanSendJointTargets, "Marker input must never publish UDP targets");
            marker.SetActive(false);
            Require(!(bool)method.Invoke(controller, args), "Inactive marker rejected");
            controller.SetInputMode(HandPoseIkController.InputMode.Mocopi);
            Require(!(bool)method.Invoke(controller, args), "Switch back requires hand input");
            Require(!controller.CanSendJointTargets, "Switch invalidates calibration");
        }
        finally { UnityEngine.Object.DestroyImmediate(host); UnityEngine.Object.DestroyImmediate(marker); }
    }
}
