using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(Rigidbody))]
public class RigidbodyFracture : MonoBehaviour
{
    public int fractureCount = 3;
    public float2 collisionVel = new float2(1, 100), sliceTilt = new float2(15, 30);

    GameObject fragmentRoot; Rigidbody rb; MeshData meshData;
    int fragmentCount; float sliceRate, remainMass; float3 point, normal;

    void Start()
    {
        rb = GetComponent<Rigidbody>(); remainMass = rb.mass;
        meshData = new MeshData(GetComponent<MeshFilter>().mesh);
    }

    void OnCollisionEnter(Collision collision)
    {
        if (collision.relativeVelocity.magnitude > collisionVel.x)
        {
            point = collision.GetContact(0).point; normal = collision.relativeVelocity.normalized; gameObject.SetActive(false);
            while (fractureCount-- > 0)
            {
                MeshProjection meshProjection = new MeshProjection(meshData, transform, point, normal,
                    sliceRate = 0.49f * (collision.relativeVelocity.magnitude - collisionVel.x) / (collisionVel.y - collisionVel.x));
                Plane plane = meshProjection.GetPlaneData(sliceTilt);
                Fracture(plane);
            }
            CreatFragment(meshData, remainMass);
        }
    }

    void Fracture(Plane plane)
    {
        MeshSlicer.Slice(meshData, plane.normal, plane.ClosestPointOnPlane(point),
                             new float2(1,1), float2.zero, out var topSlice, out var bottomSlice);
        CreatFragment(bottomSlice, sliceRate * remainMass);
        meshData = topSlice; remainMass *= (1 - sliceRate);
    }

    void CreatFragment(MeshData meshData, float mass)
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
        MeshFilter mf = fragment.AddComponent<MeshFilter>(); mf.mesh = meshData.ToMesh();
        MeshRenderer mr = fragment.AddComponent<MeshRenderer>(); mr.materials = GetComponent<MeshRenderer>().materials;
        Rigidbody rb = fragment.AddComponent<Rigidbody>(); rb.mass = mass; rb.useGravity = true;
        MeshCollider collider = fragment.AddComponent<MeshCollider>(); collider.convex = true;
    }
}