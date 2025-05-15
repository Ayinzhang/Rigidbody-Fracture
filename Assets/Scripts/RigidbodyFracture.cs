using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;
using System.Collections;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(Rigidbody))]
public class RigidbodyFracture : MonoBehaviour
{
    public int fractureCount = 3;
    public float2 collisionVel = new float2(1, 100), sliceTilt = new float2(15, 30);

    Material mat;
    GameObject fragmentRoot; Rigidbody rb; MeshData meshData, remainData; List<Mesh> meshes;
    int fragmentCount; float sliceRate, topMass, bottomMass, remainMass; float3 point, normal;

    void Start()
    {
        mat = GetComponent<MeshRenderer>().material;
        rb = GetComponent<Rigidbody>(); remainMass = rb.mass;
        meshData = new MeshData(GetComponent<MeshFilter>().mesh);
        meshes = new List<Mesh>();
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.relativeVelocity.magnitude > collisionVel.x) StartCoroutine(FractureAsync(collision));
    }

    IEnumerator FractureAsync(Collision collision)
    {
        point = collision.GetContact(0).point; normal = collision.relativeVelocity.normalized;
        sliceRate = 0.5f + 0.2f * (math.clamp((collision.relativeVelocity.magnitude - collisionVel.x) / (collisionVel.y - collisionVel.x), 0, 1) - 0.5f);
        while (fractureCount-- > 0)
        {
            MeshProjector.GetSlice(meshData, transform, point, normal, sliceRate, sliceTilt,
                out var sliceNormal, out var sliceOrigin, out var isFullSlice);
            MeshSlicer.Slice(meshData, sliceNormal, sliceOrigin, out var topData, out var bottomData);
            meshes.Add(bottomData.ToMesh());
            topMass = remainMass * (isFullSlice ? 1 - sliceRate : sliceRate);
            bottomMass += remainMass - topMass; remainMass = topMass;
            if (isFullSlice) CreatFragment(bottomMass); meshData = topData;
            yield return null;
        }
        if (meshes.Count > 0) CreatFragment(bottomMass); gameObject.SetActive(false);
        remainData = meshData; meshes.Add(remainData.ToMesh()); CreatFragment(remainMass);
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
        Rigidbody rb = fragment.AddComponent<Rigidbody>(); rb.mass = mass; rb.useGravity = true;
        for (int i = 0; i < meshes.Count; i++)
        {
            GameObject subFrag = new GameObject($"{name}_subfrag{i}");
            subFrag.transform.parent = fragment.transform; subFrag.transform.localPosition = float3.zero;
            subFrag.transform.localRotation = quaternion.identity; subFrag.transform.localScale = new float3(1, 1, 1);
            MeshFilter mf = subFrag.AddComponent<MeshFilter>(); mf.mesh = meshes[i];
            MeshRenderer mr = subFrag.AddComponent<MeshRenderer>(); Material[] materials = new Material[mf.mesh.subMeshCount];
            for (int j = 0; j < materials.Length; j++) materials[j] = mat; mr.materials = materials;
            MeshCollider meshCollider = fragment.AddComponent<MeshCollider>();
            meshCollider.sharedMesh = meshes[i]; meshCollider.convex = true;
        }
        meshes.Clear(); bottomMass = 0;
    }
}