using UnityEngine;

public partial class HandPoseIkController
{
    // User-specified reference configurations, in J1..J6 order, degrees.
    private static readonly float[][] SpecifiedJointReferences =
    {
        new float[] { 0, 0, 0, 90, 90, 0 },
        new float[] { 0, -90, 0, 90, 90, 0 },
        new float[] { -90, -90, 0, 90, 90, 0 }
    };

    private bool BuildSpecifiedRobotReferences(Vector3[] pivots, Vector3[] axes, float[] reference, Pose tip)
    {
        Vector3 humanU = referenceHuman[1] - referenceHuman[0];
        Vector3 humanV = referenceHuman[2] - referenceHuman[0];
        if (humanU.magnitude < .05f || humanV.magnitude < .05f ||
            Vector3.Cross(humanU.normalized, humanV.normalized).magnitude < .1f) return false;
        for (int pose = 0; pose < 3; pose++)
        {
            float[] q = new float[6];
            for (int j = 0; j < 6; j++)
            {
                q[j] = SpecifiedJointReferences[pose][j] * Mathf.Deg2Rad;
                // Reject rather than silently changing the user's reference configuration.
                if (q[j] < rangeLower[j] || q[j] > rangeUpper[j]) return false;
            }
            ArmPoseSolver.Forward(pivots, axes, reference, q, tip, out referenceRobot[pose], out _, out _);
            if (!Valid(referenceRobot[pose].position) || !Valid(referenceRobot[pose].rotation)) return false;
        }
        Vector3 robotU = referenceRobot[1].position - referenceRobot[0].position;
        Vector3 robotV = referenceRobot[2].position - referenceRobot[0].position;
        if (robotU.magnitude < .01f || robotV.magnitude < .01f ||
            Vector3.Cross(robotU.normalized, robotV.normalized).magnitude < .1f) return false;
        robotMinimum = robotMaximum = referenceRobot[0].position - toolOrigin.position;
        for (int i = 1; i < 3; i++)
        {
            robotMinimum = Vector3.Min(robotMinimum, referenceRobot[i].position - toolOrigin.position);
            robotMaximum = Vector3.Max(robotMaximum, referenceRobot[i].position - toolOrigin.position);
        }
        referencesReady = true;
        return true;
    }

    // Three anchors determine an affine map in their plane. The normal direction is
    // extended using the mean edge-length ratio; it is a design choice, not a measured limit.
    public static Vector3 MapThreeReferencePositions(Vector3 point, Vector3 h0, Vector3 h1, Vector3 h2,
        Vector3 r0, Vector3 r1, Vector3 r2, out Vector3 weights)
    {
        Vector3 u = h1 - h0, v = h2 - h0, offset = point - h0;
        float uu = Vector3.Dot(u, u), uv = Vector3.Dot(u, v), vv = Vector3.Dot(v, v);
        float determinant = uu * vv - uv * uv;
        weights = Vector3.zero;
        if (determinant < 1e-8f) return new Vector3(float.NaN, float.NaN, float.NaN);
        float du = Vector3.Dot(offset, u), dv = Vector3.Dot(offset, v);
        float a = (du * vv - dv * uv) / determinant;
        float b = (dv * uu - du * uv) / determinant;
        weights = new Vector3(1f - a - b, a, b);
        Vector3 ru = r1 - r0, rv = r2 - r0;
        float scale = (ru.magnitude + rv.magnitude) / (u.magnitude + v.magnitude);
        float normalDistance = Vector3.Dot(offset, Vector3.Cross(u, v).normalized);
        return r0 + a * ru + b * rv + Vector3.Cross(ru, rv).normalized * normalDistance * scale;
    }

    private bool TryGetReferenceTarget(out Pose target)
    {
        target = default;
        if (!referencesReady || hand == null) return false;
        Vector3 point = inputToWorld * (hand.position - measurementHandOrigin);
        Vector3 position = MapThreeReferencePositions(point, referenceHuman[0], referenceHuman[1], referenceHuman[2],
            referenceRobot[0].position, referenceRobot[1].position, referenceRobot[2].position, out Vector3 weights);
        // Pose orientation is anchored to the specified robot configurations.
        // Fixed-orientation mode deliberately uses the calibrated orientation instead.
        Vector3 positive = new Vector3(Mathf.Max(0, weights.x), Mathf.Max(0, weights.y), Mathf.Max(0, weights.z));
        float sum = positive.x + positive.y + positive.z;
        if (sum <= 0 || !Valid(position)) return false;
        positive /= sum;
        Quaternion orientation = referenceRobot[0].rotation;
        Quaternion expectedHand = referenceHandRotations[0];
        float firstTwo = positive.x + positive.y;
        if (firstTwo > .00001f)
        {
            orientation = Quaternion.Slerp(referenceRobot[0].rotation, referenceRobot[1].rotation, positive.y / firstTwo);
            expectedHand = Quaternion.Slerp(referenceHandRotations[0], referenceHandRotations[1], positive.y / firstTwo);
        }
        orientation = Quaternion.Slerp(orientation, referenceRobot[2].rotation, positive.z);
        expectedHand = Quaternion.Slerp(expectedHand, referenceHandRotations[2], positive.z);
        orientation = inputToWorld * (hand.rotation * Quaternion.Inverse(expectedHand)) * Quaternion.Inverse(inputToWorld) * orientation;
        target = new Pose(position, followOrientation ? orientation : toolOrigin.rotation);
        return Valid(target.rotation);
    }
    private bool TrySolveFromReferences(Vector3[] pivots, Vector3[] axes, float[] current,
        float[] lower, float[] upper, Pose actual, Pose target, out float[] solution)
    {
        solution = null;
        for (int index = 0; index < 3; index++)
        {
            var seed = new float[6];
            for (int j = 0; j < 6; j++) seed[j] = SpecifiedJointReferences[index][j] * Mathf.Deg2Rad;
            var candidate = ArmPoseSolver.Solve(pivots, axes, current, seed, lower, upper, actual, target, 70);
            ArmPoseSolver.Forward(pivots, axes, current, candidate, actual, out Pose reached, out _, out _);
            if (!PoseWithinTolerance(reached, target)) continue;
            solution = candidate;
            return true;
        }
        return false;
    }
}
