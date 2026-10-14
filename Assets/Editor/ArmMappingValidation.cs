using System;
using System.Collections.Generic;
using System.Reflection;
using UnityEditor;
using UnityEngine;

/// <summary>Edit-mode checks only: no Play mode, scene loading, UDP sending or robot movement.</summary>
public static class ArmMappingValidation
{
    private const BindingFlags Private = BindingFlags.Instance | BindingFlags.NonPublic;
    private static readonly List<string> Results = new List<string>();

    [MenuItem("Tools/Robot Arm/Validate A Mapping")]
    public static void Run()
    {
        Results.Clear();
        Check("Calibration samples the current pose", () => WithSource(source =>
        {
            source.Calibrate();
            Near(source.RawYaw, 0f);
            Bone(source, "lowerArm").position = Vector3.right;
            Bone(source, "rightHand").position = Vector3.right + Vector3.up;
            source.Calibrate();
            Near(Get<float>(source, "baseYaw"), 90f);
            Require(!source.CanSendJointTargets, "Calibration must not expose unapplied targets.");
        }));
        Check("Degenerate forearm does not partially update features", () => WithSource(source =>
        {
            source.Calibrate();
            float yaw = source.RawYaw;
            Bone(source, "lowerArm").position = Vector3.right;
            Bone(source, "rightHand").position = Vector3.right;
            Require(!(bool)Call(source, "UpdateArmAngles"), "Degenerate pose was accepted.");
            Near(source.RawYaw, yaw);
            source.Calibrate();
            Require(!source.IsCalibrated && !source.CanSendJointTargets, "Invalid calibration was accepted.");
        }));
        Check("Elbow calibration crossing +/-180 uses the short difference", () => WithSource(source =>
        {
            Set(source, "baseElbow", 179f);
            Set(source, "rawElbow", -179f);
            Call(source, "UpdateRobotAngles", 1f / 60f);
            Near(source.DeltaElbow, 2f);
        }));
        Check("Smoothing has equal step response at 30/60/120 Hz", () => WithSource(source =>
        {
            Set(source, "rawYaw", 80f);
            float[] outputs = new float[3];
            int[] rates = { 30, 60, 120 };
            for (int i = 0; i < rates.Length; i++)
            {
                Set(source, "smoothedJ1", 0f);
                for (int frame = 0; frame < rates[i] / 10; frame++)
                    Call(source, "UpdateRobotAngles", 1f / rates[i]);
                outputs[i] = source.SmoothedJ1;
            }
            Near(outputs[0], outputs[1]);
            Near(outputs[1], outputs[2]);
            Near(outputs[1], 80f * (1f - Mathf.Pow(0.8f, 6)));
        }));
        Check("Drive limits and exported command agree", () => WithJoint((source, joint) =>
        {
            object[] args = { joint, 50f };
            Require((bool)Call(source, "SetJointTarget", args), "Revolute joint rejected.");
            Set(source, "smoothedJ1", (float)args[1]);
            Near(source.GetRobotJoints()[0], joint.xDrive.target);
            Near(joint.xDrive.target, 10f);
        }));
        Check("IK start pose reads A bases, offsets and intersected limits without moving", () => WithJoint((source, joint) =>
        {
            for (int i = 1; i <= 6; i++) Set(source, "joint" + i, joint);
            Set(source, "robotBaseJ4", 90f);
            Set(source, "j4Offset", 15f);
            Set(source, "j4Max", 100f);
            var drive = joint.xDrive;
            drive.lowerLimit = -120f; drive.upperLimit = 95f; drive.target = 12f;
            joint.xDrive = drive;
            Require(source.TryGetCalibrationTargets(out float[] pose), "Valid A pose rejected.");
            Near(pose[3], 95f);
            Near(joint.xDrive.target, 12f);
            Set(source, "j4Min", 110f);
            Require(!source.TryGetCalibrationTargets(out _), "Disjoint limits accepted.");
        }));
        Check("Equal drive limits are respected", () => WithJoint((source, joint) =>
        {
            ArticulationDrive drive = joint.xDrive;
            drive.lowerLimit = drive.upperLimit = 4f;
            joint.xDrive = drive;
            object[] args = { joint, 50f };
            Require((bool)Call(source, "SetJointTarget", args), "Equal limit rejected.");
            Near((float)args[1], 4f);
        }));
        Check("Conflicting mapping and drive limits suppress application", () => WithJoint((source, joint) =>
        {
            Set(source, "j1Min", 20f);
            Set(source, "j1Max", 30f);
            Require(!(bool)Call(source, "SetJointTarget", joint, 25f), "Disjoint limits accepted.");
        }));
        Check("Prismatic joints reject angle commands", () => WithJoint((source, joint) =>
        {
            joint.jointType = ArticulationJointType.PrismaticJoint;
            Require(!(bool)Call(source, "SetJointTarget", joint, 5f), "Degree command accepted by prismatic joint.");
        }));
        Check("Nonfinite commands are rejected", () => WithJoint((source, joint) =>
        {
            Require(!(bool)Call(source, "SetJointTarget", joint, float.NaN), "NaN accepted.");
        }));
        Check("Uncalibrated, disabled and invalid sources cannot transmit", () => WithSource(source =>
        {
            Require(!source.CanSendJointTargets, "Startup source ready.");
            source.Calibrate();
            Set(source, "hasAppliedTargets", true);
            Require(source.CanSendJointTargets, "Valid source not ready.");
            source.enabled = false;
            Require(!source.CanSendJointTargets, "Disabled source ready.");
            source.enabled = true;
            Set(source, "hasValidPose", false);
            Require(!source.CanSendJointTargets, "Invalid source ready.");
        }));
        Check("Characterization: world yaw responds to rigid body rotation", () => WithSource(source =>
        {
            source.Calibrate();
            Quaternion rotation = Quaternion.AngleAxis(45f, Vector3.up);
            Bone(source, "lowerArm").position = rotation * Vector3.forward;
            Bone(source, "rightHand").position = rotation * (Vector3.forward + Vector3.up);
            Call(source, "UpdateArmAngles");
            Near(source.RawYaw, 45f);
        }));
        Check("Characterization: J4 changes during planar elbow flexion at raised arm", () => WithSource(source =>
        {
            Vector3 upper = new Vector3(0.5f, 0.7071068f, 0.5f);
            Vector3 axis = Vector3.Cross(upper, Vector3.up).normalized;
            Bone(source, "lowerArm").position = upper;
            Bone(source, "rightHand").position = upper + upper;
            Call(source, "UpdateArmAngles");
            float initial = source.RawForearmYaw;
            // Bend across vertical in the same plane: horizontal direction reverses.
            Bone(source, "rightHand").position = upper + Quaternion.AngleAxis(100f, axis) * upper;
            Call(source, "UpdateArmAngles");
            Require(Mathf.Abs(Mathf.DeltaAngle(initial, source.RawForearmYaw)) > 170f,
                "Expected existing yaw reversal not reproduced.");
        }));

        int failures = Results.FindAll(result => result.StartsWith("FAIL")).Count;
        string report = string.Join("\n", Results) + $"\n{Results.Count - failures}/{Results.Count} checks passed.";
        Debug.Log(report);
        System.IO.Directory.CreateDirectory("Logs");
        System.IO.File.WriteAllText("Logs/ArmMappingValidation.txt", report);
        if (Application.isBatchMode) EditorApplication.Exit(failures == 0 ? 0 : 1);
        else if (failures > 0) throw new Exception(report);
    }

    private static void Check(string name, Action check)
    {
        try { check(); Results.Add("PASS " + name); }
        catch (Exception exception) { Results.Add("FAIL " + name + ": " + exception.GetBaseException().Message); }
    }

    private static void WithSource(Action<UpperArmToUrdfRobotJoints> check)
    {
        GameObject root = new GameObject("MappingValidationOnly");
        try
        {
            var source = root.AddComponent<UpperArmToUrdfRobotJoints>();
            foreach (string name in new[] { "upperArm", "lowerArm", "rightHand" })
            {
                var bone = new GameObject(name).transform;
                bone.SetParent(root.transform);
                Set(source, name, bone);
            }
            Bone(source, "upperArm").position = Vector3.zero;
            Bone(source, "lowerArm").position = Vector3.forward;
            Bone(source, "rightHand").position = Vector3.forward + Vector3.up;
            Set(source, "hasValidBones", true);
            check(source);
        }
        finally { UnityEngine.Object.DestroyImmediate(root); }
    }

    private static void WithJoint(Action<UpperArmToUrdfRobotJoints, ArticulationBody> check)
    {
        WithSource(source =>
        {
            var root = new GameObject("ArticulationRoot");
            root.transform.SetParent(source.transform);
            root.AddComponent<ArticulationBody>().immovable = true;
            var link = new GameObject("Joint1");
            link.transform.SetParent(root.transform);
            var joint = link.AddComponent<ArticulationBody>();
            joint.jointType = ArticulationJointType.RevoluteJoint;
            joint.twistLock = ArticulationDofLock.LimitedMotion;
            ArticulationDrive drive = joint.xDrive;
            drive.lowerLimit = -10f;
            drive.upperLimit = 10f;
            joint.xDrive = drive;
            Set(source, "joint1", joint);
            check(source, joint);
        });
    }

    private static Transform Bone(object target, string name) => Get<Transform>(target, name);
    private static T Get<T>(object target, string name) => (T)target.GetType().GetField(name, Private).GetValue(target);
    private static void Set(object target, string name, object value) => target.GetType().GetField(name, Private).SetValue(target, value);
    private static object Call(object target, string name, params object[] args) => target.GetType().GetMethod(name, Private).Invoke(target, args);
    private static void Require(bool condition, string message) { if (!condition) throw new Exception(message); }
    private static void Near(float actual, float expected) => Require(Mathf.Abs(actual - expected) < 0.001f, $"Expected {expected}, got {actual}.");
}
