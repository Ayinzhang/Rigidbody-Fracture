using UnityEngine;
using Unity.Mathematics;

public class RigidbodySimulation : MonoBehaviour
{
    public bool applyGravity = true;
    public enum SimulationType { Impulse, ShapeMatching } [SerializeField] SimulationType type;
    public SimulationType Type { get => type; set { type = value; Start(); } }
    [HideInInspector] public int iterations = 3;
    [HideInInspector] public float vDecay = 0.99f, wDecay = 0.98f, restitution = 0.2f, friction = 0.9f;

    Mesh mesh; float3 vc, g = new float3(0, -9.8f, 0); // Public parameters
    float3x3 I_ref, R_inv; float3 p, v, w; float3[] verts; // Impulse parameters
    float3 c; float3[] x, y, r, vi; quaternion R; // Shape matching parameters

    void Start()
    {
        mesh = GetComponentInChildren<MeshFilter>().sharedMesh;
        verts = new float3[mesh.vertexCount];
        for (int i = 0; i < verts.Length; i++) { verts[i] = mesh.vertices[i]; vc += verts[i]; }
        vc /= verts.Length;
        if (Type == 0)
        {
            // Cal inertia tensor
            for (int i = 0; i < verts.Length; i++)
            {
                float3 vr = verts[i] - vc;
                float x2 = vr.x * vr.x, y2 = vr.y * vr.y, z2 = vr.z * vr.z;
                I_ref[0] += new float3(y2 + z2, -vr.x * vr.y, -vr.x * vr.z);
                I_ref[1] += new float3(-vr.x * vr.y, x2 + z2, -vr.y * vr.z);
                I_ref[2] += new float3(-vr.x * vr.z, -vr.y * vr.z, x2 + y2);
            }
        }
        else
        {
            x = new float3[mesh.vertexCount]; y = new float3[mesh.vertexCount]; 
            r = new float3[mesh.vertexCount]; vi = new float3[mesh.vertexCount];

            float3x3 R = new float3x3(transform.rotation);
            for (int i = 0; i < mesh.vertexCount; i++)
            {
                r[i] = verts[i] - vc;
                x[i] = (float3)transform.position + math.mul(R, r[i]);
            }

            // Cal R_inv = (¡Ær_i * r_i^T)^-1 
            float3x3 RR = float3x3.zero;
            for (int i = 0; i < mesh.vertexCount; i++)
            {
                RR.c0 += r[i] * r[i].x;
                RR.c1 += r[i] * r[i].y;
                RR.c2 += r[i] * r[i].z;
            }
            R_inv = math.inverse(RR);
        }
    }

    void FixedUpdate()
    {
        if (Type == 0)
        {
            // Update velocity and mass center
            if (applyGravity) v += g * Time.fixedDeltaTime;
            p = transform.position; v *= vDecay; w *= wDecay;
            // Handle collision
            HandleCollision(float3.zero, new float3(0, 1, 0));
            // Update position and rotation
            transform.position += (Vector3)v * Time.fixedDeltaTime;
            transform.rotation = Quaternion.Euler(w * 180 / math.PI * Time.fixedDeltaTime) * transform.rotation;
        }
        else
        {
            // Free update
            for (int i = 0; i < verts.Length; i++)
            {
                if (applyGravity) vi[i] += g * Time.fixedDeltaTime;
                vi[i] *= vDecay; y[i] = x[i] + vi[i] * Time.fixedDeltaTime;
            }

            for (int it = 0; it < iterations; it++)
            {
                HandleCollision(float3.zero, new float3(0, 1, 0));

                c = float3.zero;
                for (int i = 0; i < verts.Length; i++) c += y[i]; c /= verts.Length;

                // Cal A = (¡Æ(y_i - c) * r_i^T) * inv(¡Ær_i * r_i^T)
                float3x3 A = float3x3.zero;
                for (int i = 0; i < verts.Length; i++)
                {
                    float3 p_ = y[i] - c;
                    A.c0 += p_ * r[i].x;
                    A.c1 += p_ * r[i].y;
                    A.c2 += p_ * r[i].z;
                }
                A = math.mul(A, R_inv);
                R = svd.svdRotation(A);

                // Projection
                for (int i = 0; i < verts.Length; i++)
                {
                    float3 goal = c + math.mul(R, r[i]);
                    y[i] = math.lerp(y[i], goal, restitution);
                }
            }

            // Update
            for (int i = 0; i < verts.Length; i++)
            {
                vi[i] = (y[i] - x[i]) / Time.fixedDeltaTime;
                x[i] = y[i];
            }
            transform.position = c;
            transform.rotation = R;
        }
    }

    void HandleCollision(float3 pos, float3 normal)
    {
        if (Type == 0)
        {
            // Cal virtual collision point
            int num = 0; float3 sum = new float3(0, 0, 0); float3x3 R = new float3x3(transform.rotation);
            for (int i = 0; i < verts.Length; i++)
            {
                float3 vp = p + math.mul(R, verts[i]).xyz, vr = math.cross(w, math.mul(R, verts[i] - vc).xyz);
                if (math.dot(vp - pos, normal) < 0 && math.dot(v + vr, normal) < 0) { num++; sum += vp; }
            }
            if (num == 0) return; sum /= num;
            // Cal impulse by K * j = ¦¤v
            float3 r = sum - p - math.mul(R, vc).xyz, vn = math.dot(v + math.cross(w, r), normal) * normal,
                vt = v + math.cross(w, r) - vn, vnn = -vn * restitution,
                vtn = vt * math.clamp(1 - friction * (1 + restitution) * math.length(vn) / (math.length(vt) + float.Epsilon), 0, 1);
            float3x3 Rri = new float3x3(new float3(0, -r.z, r.y), new float3(r.z, 0, -r.x), new float3(-r.y, r.x, 0)),
                I_w = R * I_ref * math.transpose(R), I_inv = math.determinant(I_w) < 1e-5f ? 1e-3f * float3x3.identity : math.inverse(I_w),
                K = float3x3.identity / verts.Length + math.mul(Rri, math.mul(I_inv, math.transpose(Rri)));
            float3 j = math.mul(math.inverse(K), vnn + vtn - vn - vt).xyz;
            v += j / verts.Length; w += math.mul(I_inv, math.cross(r, j)).xyz;
        }
        else
        {
            for (int i = 0; i < y.Length; i++)
            {
                float3 toSurface = y[i] - pos; float d = math.dot(toSurface, normal);

                if (d < 0)
                {
                    // Push out and bounce
                    y[i] -= normal * d; float vn = math.dot(vi[i], normal);
                    if (vn < 0)
                    {
                        float3 vNormal = vn * normal, vTangent = vi[i] - vNormal;
                        vNormal *= -restitution; vTangent *= (1 - friction); vi[i] = vNormal + vTangent;
                    }
                }
            }
        }
    }
}