using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(Rigidbody))]
public class RigidbodyFracture : MonoBehaviour
{
    public int fractureCount = 3;
    public float2 collisionVel = new float2(1, 100), sliceTilt = new float2(15, 30);

    GameObject fragmentRoot; Rigidbody rb; MeshData meshData, remainData; List<Mesh> meshes;
    int fragmentCount; float sliceRate, topMass, bottomMass, remainMass; float3 point, normal;

    void Start()
    {
        rb = GetComponent<Rigidbody>(); remainMass = rb.mass;
        meshData = new MeshData(GetComponent<MeshFilter>().mesh);
        meshes = new List<Mesh>();
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.relativeVelocity.magnitude > collisionVel.x)
        {
            point = collision.GetContact(0).point; normal = collision.relativeVelocity.normalized; gameObject.SetActive(false);
            sliceRate = 0.2f + 0.3f * math.clamp((collision.relativeVelocity.magnitude - collisionVel.x) / 
                (collisionVel.y - collisionVel.x), 0, 1);
            while (fractureCount-- > 0)
            {
                MeshProjector.GetSlice(meshData, transform, point, normal, sliceRate, sliceTilt, 
                    out var sliceNormal, out var sliceOrigin, out var isFullSlice);
                MeshSlicer.Slice(meshData, sliceNormal, sliceOrigin, out var topData, out var bottomData);
                meshes.Add(bottomData.ToMesh());
                topMass = remainMass * (isFullSlice ? 1 - sliceRate: sliceRate); 
                bottomMass += remainMass - topMass; remainMass = topMass;
                if (isFullSlice)
                {
                    if (remainData == null) remainData = bottomData;
                    else remainData.AddMeshData(bottomData);
                    CreatFragment(bottomMass); remainData = null; bottomMass = 0;
                }
                else if (remainData == null) remainData = bottomData;
                else remainData.AddMeshData(bottomData);
                meshData = topData;
            }
            if (remainData != null) CreatFragment(bottomMass);
            remainData = meshData; meshes.Add(remainData.ToMesh()); CreatFragment(remainMass);
        }
    }

    void CreatFragment(float mass)
    {
        if (fragmentRoot == null)
        {
            fragmentRoot = new GameObject($"{name}_frags");
            fragmentRoot.transform.parent = transform.parent;
            fragmentRoot.transform.position = transform.position;
            fragmentRoot.transform.rotation = transform.rotation;
        }
        GameObject fragment = new GameObject($"{name}_frag{fragmentCount++}");
        fragment.transform.parent = fragmentRoot.transform; fragment.transform.localPosition = float3.zero;
        fragment.transform.localRotation = quaternion.identity; fragment.transform.localScale = transform.localScale;
        MeshFilter mf = fragment.AddComponent<MeshFilter>(); mf.mesh = remainData.ToMesh();
        MeshRenderer mr = fragment.AddComponent<MeshRenderer>(); mr.materials = GetComponent<MeshRenderer>().materials;
        Rigidbody rb = fragment.AddComponent<Rigidbody>(); rb.mass = mass; rb.useGravity = true;
        foreach (var mesh in meshes)
        {
            MeshCollider meshCollider = fragment.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = mesh; meshCollider.convex = true;
        }
        meshes.Clear();
    }
}