using UnityEngine;
using Unity.Mathematics;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(Rigidbody))]
public class RigidbodyFracture : MonoBehaviour
{
    public int fractureCount = 3, reFractureCount = 2;
    public float2 collisionVel = new float2(1, 100);
    public float2 sliceTilt = new float2(15, 30);
    [HideInInspector] public GameObject fragmentRoot;

    Material mat; Rigidbody rb; Collider[] colliders; MeshData meshData; 
    List<MeshData> pendingHalfSlices = new List<MeshData>();
    int fragmentCount = 0; bool hasSplit = false;
    float remainMass; float3 point, normal;

    void Start()
    {
        mat = GetComponent<MeshRenderer>().material;
        rb = GetComponent<Rigidbody>(); remainMass = rb.mass;
        meshData = new MeshData(GetComponent<MeshFilter>().mesh);
        colliders = GetComponentsInChildren<Collider>();
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.relativeVelocity.magnitude > collisionVel.x)
            StartCoroutine(FractureAsync(collision));
    }

    IEnumerator FractureAsync(Collision collision)
    {
        for (int i = 0; i < colliders.Length; i++)  colliders[i].enabled = false;

        point = collision.GetContact(0).point;
        normal = collision.relativeVelocity.normalized;
        float sliceRate = 0.5f + 0.2f * (math.clamp((collision.relativeVelocity.magnitude - collisionVel.x) / (collisionVel.y - collisionVel.x), 0, 1) - 0.5f);

        while (fractureCount-- > 0)
        {
            MeshProjector.GetSlice(meshData, transform, point, normal, sliceRate, sliceTilt,
                out var sliceNormal, out var sliceOrigin, out var isFullSlice);

            if (fractureCount == 0 && !isFullSlice && !hasSplit) { isFullSlice = true; sliceNormal = -sliceNormal; }

            MeshSlicer.Slice(meshData, sliceNormal, sliceOrigin, out var topData, out var bottomData);

            if (isFullSlice)
            {
                float bottomMass = remainMass * (1 - sliceRate);
                CreateSingleFragment(collision.collider, bottomData.ToMesh(), bottomMass);
                remainMass *= sliceRate;
                hasSplit = true;
            }
            else
            {
                pendingHalfSlices.Add(bottomData);
                remainMass *= sliceRate;
            }

            meshData = topData;

            yield return null;
        }

        if (pendingHalfSlices.Count > 0)
        {
            MeshData merged = new MeshData(pendingHalfSlices);
            CreateMergedFragment(collision.collider, merged, remainMass * 0.5f);
            remainMass *= 0.5f;
        }

        CreateSingleFragment(collision.collider, meshData.ToMesh(), remainMass);
        gameObject.SetActive(false);
    }

    void CreateSingleFragment(Collider collider, Mesh mesh, float mass)
    {
        GameObject frag = CreateFragmentRoot(mesh, mass);
        MeshCollider col = frag.AddComponent<MeshCollider>();
        Physics.IgnoreCollision(collider, col);
        col.sharedMesh = mesh; col.convex = true; 
        if (reFractureCount > 0)
        {
            RigidbodyFracture fracScript = frag.AddComponent<RigidbodyFracture>();
            fracScript.reFractureCount = reFractureCount - 1; fracScript.fragmentRoot = fragmentRoot;
        }
    }

    void CreateMergedFragment(Collider collider, MeshData mergedData, float mass)
    {
        Mesh mergedMesh = mergedData.ToMesh();
        GameObject frag = CreateFragmentRoot(mergedMesh, mass);

        int subFragIndex = 0;
        foreach (MeshData sub in pendingHalfSlices)
        {
            Mesh subMesh = sub.ToMesh();

            GameObject colliderChild = new GameObject($"FragCollider{subFragIndex++}");
            colliderChild.transform.parent = frag.transform;
            colliderChild.transform.localPosition = Vector3.zero;
            colliderChild.transform.localRotation = Quaternion.identity;
            colliderChild.transform.localScale = Vector3.one;

            MeshCollider col = colliderChild.AddComponent<MeshCollider>();
            Physics.IgnoreCollision(collider, col);
            col.sharedMesh = subMesh;
            col.convex = true;
        }

        if (reFractureCount > 0)
        {
            RigidbodyFracture fracScript = frag.AddComponent<RigidbodyFracture>();
            fracScript.reFractureCount = reFractureCount - 1; fracScript.fragmentRoot = fragmentRoot;
        }

        pendingHalfSlices.Clear();
    }

    GameObject CreateFragmentRoot(Mesh mesh, float mass)
    {
        if (fragmentRoot == null)
        {
            fragmentRoot = new GameObject($"{name}_frags");
            fragmentRoot.transform.SetPositionAndRotation(transform.position, transform.rotation);
            fragmentRoot.transform.localScale = Vector3.one;
            fragmentRoot.transform.parent = transform.parent;
        }

        GameObject frag = new GameObject($"{name}_frag{fragmentCount++}");
        frag.transform.parent = fragmentRoot.transform;
        frag.transform.localPosition = Vector3.zero;
        frag.transform.localRotation = Quaternion.identity;
        frag.transform.localScale = transform.localScale;

        MeshFilter mf = frag.AddComponent<MeshFilter>();
        mf.mesh = mesh;

        MeshRenderer mr = frag.AddComponent<MeshRenderer>();
        Material[] materials = new Material[mesh.subMeshCount];
        for (int i = 0; i < materials.Length; i++) materials[i] = mat;
        mr.materials = materials;

        Rigidbody newRb = frag.AddComponent<Rigidbody>();
        newRb.mass = mass;
        newRb.useGravity = true;

        return frag;
    }
}
