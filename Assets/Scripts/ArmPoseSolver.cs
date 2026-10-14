using UnityEngine;

/// <summary>Numerical IK on a virtual copy of the articulation geometry; never edits link transforms.</summary>
public static class ArmPoseSolver
{
    public static Vector3 RotationError(Quaternion target, Quaternion actual)
    {
        Quaternion delta = target * Quaternion.Inverse(actual);
        if (delta.w < 0f) delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
        delta.ToAngleAxis(out float angle, out Vector3 axis);
        return angle < 0.0001f ? Vector3.zero : axis.normalized * (angle * Mathf.Deg2Rad);
    }

    // All q values are radians. Pivots/axes describe the geometry at referenceQ.
    public static void Forward(Vector3[] pivots, Vector3[] axes, float[] referenceQ, float[] q,
        Pose referenceTip, out Pose tip, out Vector3[] movedPivots, out Vector3[] movedAxes)
    {
        movedPivots = (Vector3[])pivots.Clone();
        movedAxes = (Vector3[])axes.Clone();
        tip = referenceTip;
        for (int i = 0; i < q.Length; i++)
        {
            Quaternion r = Quaternion.AngleAxis((q[i] - referenceQ[i]) * Mathf.Rad2Deg, movedAxes[i]);
            tip.position = movedPivots[i] + r * (tip.position - movedPivots[i]);
            tip.rotation = r * tip.rotation;
            for (int j = i + 1; j < q.Length; j++)
            {
                movedPivots[j] = movedPivots[i] + r * (movedPivots[j] - movedPivots[i]);
                movedAxes[j] = r * movedAxes[j];
            }
        }
    }

    public static float[] Solve(Vector3[] pivots, Vector3[] axes, float[] referenceQ,
        float[] seed, float[] lower, float[] upper, Pose referenceTip, Pose target, int iterations = 40)
    {
        float[] q = (float[])seed.Clone();
        const float weight = 0.15f, damping = 0.025f;
        for (int iteration = 0; iteration < iterations; iteration++)
        {
            Forward(pivots, axes, referenceQ, q, referenceTip, out Pose tip, out Vector3[] p, out Vector3[] a);
            Vector3 ep = target.position - tip.position;
            Vector3 er = RotationError(target.rotation, tip.rotation);
            if (ep.magnitude < 0.001f && er.magnitude < 0.015f) break;
            float[] e = { ep.x, ep.y, ep.z, er.x * weight, er.y * weight, er.z * weight };
            float[,] jac = new float[6, 6];
            for (int j = 0; j < 6; j++)
            {
                Vector3 v = Vector3.Cross(a[j], tip.position - p[j]);
                jac[0,j] = v.x; jac[1,j] = v.y; jac[2,j] = v.z;
                jac[3,j] = a[j].x * weight; jac[4,j] = a[j].y * weight; jac[5,j] = a[j].z * weight;
            }
            float[,] system = new float[6, 7];
            for (int r = 0; r < 6; r++)
            {
                for (int c = 0; c < 6; c++)
                {
                    for (int k = 0; k < 6; k++) system[r,c] += jac[r,k] * jac[c,k];
                    if (r == c) system[r,c] += damping * damping;
                }
                system[r,6] = e[r];
            }
            // Positive definite damped normal system, with partial pivoting.
            for (int k = 0; k < 6; k++)
            {
                int best = k;
                for (int r = k + 1; r < 6; r++) if (Mathf.Abs(system[r,k]) > Mathf.Abs(system[best,k])) best = r;
                for (int c = k; c <= 6; c++) { float t = system[k,c]; system[k,c] = system[best,c]; system[best,c] = t; }
                float divisor = system[k,k];
                for (int c = k; c <= 6; c++) system[k,c] /= divisor;
                for (int r = 0; r < 6; r++) if (r != k)
                {
                    float factor = system[r,k];
                    for (int c = k; c <= 6; c++) system[r,c] -= factor * system[k,c];
                }
            }
            float[] candidate = new float[6];
            float oldCost = ep.sqrMagnitude + weight * weight * er.sqrMagnitude;
            bool accepted = false;
            for (float step = 1f; step >= 0.0625f; step *= 0.5f)
            {
                for (int j = 0; j < 6; j++)
                {
                    float dq = 0;
                    for (int r = 0; r < 6; r++) dq += jac[r,j] * system[r,6];
                    candidate[j] = Mathf.Clamp(q[j] + step * Mathf.Clamp(dq, -0.15f, 0.15f), lower[j], upper[j]);
                }
                Forward(pivots, axes, referenceQ, candidate, referenceTip, out Pose next, out _, out _);
                float cost = (target.position - next.position).sqrMagnitude + weight * weight * RotationError(target.rotation, next.rotation).sqrMagnitude;
                if (cost < oldCost) { q = (float[])candidate.Clone(); accepted = true; break; }
            }
            if (!accepted) break;
        }
        return q;
    }
}
