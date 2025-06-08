using UnityEngine;
using Unity.Mathematics;
using System.Collections;
using System.Collections.Generic;

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer), typeof(Rigidbody))]
public class RigidbodyFracture : MonoBehaviour
{
    public int fractureCount = 3, reFractureCount = 2;
    public float halfSliceRate = 0.7f;
    public float2 collisionVel = new float2(1, 100), sliceTilt = new float2(5, 15);
    [HideInInspector] public GameObject fragmentRoot;

    Material mat; Rigidbody rb; MeshFilter mf;
    Collider colObj; MeshData meshData; 
    int fragmentCount = 0; bool isFracturing; float3 point, normal;

    void Start()
    {
        mat = GetComponent<MeshRenderer>().material; 
        rb = GetComponent<Rigidbody>(); mf = GetComponent<MeshFilter>();
        meshData = new MeshData(mf.mesh, rb.mass);
    }

    public void OnCollisionEnter(Collision collision)
    {
        if (!isFracturing && reFractureCount > 0 && collision.relativeVelocity.magnitude > collisionVel.x) 
            StartCoroutine(Fracture(collision));
    }

    IEnumerator Fracture(Collision collision)
    {
        isFracturing = true;
        colObj = collision.collider; point = collision.GetContact(0).point; normal = collision.relativeVelocity.normalized;
        float sliceRate = 0.5f + 0.2f * (math.clamp((collision.relativeVelocity.magnitude - collisionVel.x) / (collisionVel.y - collisionVel.x), 0, 1) - 0.5f);

        MeshProjector.GetSliceType(meshData, transform, point, normal, halfSliceRate, out var isFullSlice);

        if (isFullSlice)
        {
            Queue<MeshData> meshDatas = new Queue<MeshData>(); meshDatas.Enqueue(meshData);

            while (fragmentCount++ < fractureCount)
            {
                meshData = meshDatas.Dequeue();
                MeshProjector.GetSlice(meshData, transform, point, normal, sliceRate, sliceTilt,
                    out var sliceNormal, out var sliceOrigin);
                MeshSlicer.Slice(meshData, sliceNormal, sliceOrigin, out var topData, out var bottomData);
                topData.mass = meshData.mass * sliceRate; bottomData.mass = meshData.mass * (1 - sliceRate);
                meshDatas.Enqueue(topData); meshDatas.Enqueue(bottomData); yield return null;
            }

            while (meshDatas.Count > 0) CreateFragment(meshDatas.Dequeue()); gameObject.SetActive(false);
        }
        else
        {
            List<MeshData> meshDatas = new List<MeshData>();

            while (fragmentCount++ < fractureCount)
            {
                MeshProjector.GetSlice(meshData, transform, point, normal, sliceRate, sliceTilt,
                    out var sliceNormal, out var sliceOrigin);
                MeshSlicer.Slice(meshData, sliceNormal, sliceOrigin, out var topData, out var bottomData);
                topData.mass = meshData.mass * sliceRate; bottomData.mass = meshData.mass * (1 - sliceRate);
                meshData = topData; meshDatas.Add(bottomData); yield return null;
            }

            CreateFragment(meshData); meshData = new MeshData(meshDatas);
            mf.mesh = meshData.ToMesh(); rb.mass = meshData.mass; 
            for (int i = 0; i < gameObject.transform.childCount; i++) 
                gameObject.transform.GetChild(i).gameObject.SetActive(false);
            if (TryGetComponent<Collider>(out var col)) col.enabled = false;
            foreach(var meshData in meshDatas) CreateFragment(meshData, true);
            fragmentCount = 0; reFractureCount--;
        }
        isFracturing = false;
    }

    void CreateFragment(MeshData meshData, bool isSelfChild = false)
    {
        if (!isSelfChild && fragmentRoot == null)
        {
            fragmentRoot = new GameObject($"{name}_frags");
            fragmentRoot.transform.SetPositionAndRotation(transform.position, transform.rotation);
            fragmentRoot.transform.localScale = Vector3.one;
            fragmentRoot.transform.parent = transform.parent;
        }

        GameObject frag = new GameObject($"{name}_frag{fragmentCount}");
        frag.transform.parent = isSelfChild ? gameObject.transform: fragmentRoot.transform;
        frag.transform.position = transform.position;   
        frag.transform.localScale = isSelfChild ? Vector3.one: transform.localScale;

        MeshFilter mf = frag.AddComponent<MeshFilter>();
        mf.mesh = meshData.ToMesh();

        MeshCollider mc = frag.AddComponent<MeshCollider>();
        mc.sharedMesh = mf.mesh; mc.convex = true;

        if (isSelfChild) { frag.AddComponent<CollisionRelay>(); return; }
        Physics.IgnoreCollision(colObj, mc);

        MeshRenderer mr = frag.AddComponent<MeshRenderer>();
        Material[] materials = new Material[mf.mesh.subMeshCount];
        for (int i = 0; i < materials.Length; i++) materials[i] = mat;
        mr.materials = materials;

        Rigidbody newRb = frag.AddComponent<Rigidbody>();
        newRb.mass = meshData.mass; newRb.useGravity = true;

        if (reFractureCount > 0) 
        {
            RigidbodyFracture rbFracture = frag.AddComponent<RigidbodyFracture>();
            rbFracture.reFractureCount = reFractureCount - 1; rbFracture.halfSliceRate = halfSliceRate; 
            rbFracture.collisionVel = collisionVel; rbFracture.sliceTilt = sliceTilt; 
            rbFracture.fragmentRoot = fragmentRoot; rbFracture.meshData = meshData;
        }
    }
}
