using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using Unity.Mathematics;

public enum MeshType { Default = 0, CutFace = 1}

public struct MeshVertex
{
    public float3 position, normal;
    public float2 uv;

    public MeshVertex(float3 position, float3 normal, float2 uv)
    {
        this.position = position; this.normal = normal; this.uv = uv;
    }
}

public class MeshEdge
{
    public int v1, v2, t1, t2, t1Edge;

    public MeshEdge(int v1, int v2)
    {
        this.v1 = v1; this.v2 = v2; t1 = -1; t2 = -1;
    }

    public MeshEdge(int v1, int v2, int t1, int t2, int edge1)
    {
        this.v1 = v1; this.v2 = v2; this.t1 = t1; this.t2 = t2; t1Edge = edge1;
    }
}

public class MeshData
{
    public List<MeshVertex> vertices, cutVertices;
    public List<int>[] triangles; public List<MeshEdge> constraints;
    public int[] indexMap;
    public int vertexCount { get => vertices.Count + cutVertices.Count; }
    public int triangleCount
    {
        get
        {
            int count = 0;
            for (int i = 0; i < triangles.Length; i++) count += triangles[i].Count;
            return count;
        }
    }

    public MeshData(int vertexCount, int triangleCount)
    {
        vertices = new List<MeshVertex>(vertexCount);
        cutVertices = new List<MeshVertex>(vertexCount / 10);
        triangles = new List<int>[] { new List<int>(triangleCount), new List<int>(triangleCount / 10) };
        constraints = new List<MeshEdge>();
        indexMap = new int[vertexCount];
    }

    public MeshData(Mesh mesh)
    {
        vertices = new List<MeshVertex>(mesh.vertexCount);
        cutVertices = new List<MeshVertex>(mesh.vertexCount / 10);
        constraints = new List<MeshEdge>();
        indexMap = new int[mesh.vertices.Length];

        for (int i = 0; i < mesh.vertices.Length; i++) vertices.Add(new MeshVertex(mesh.vertices[i], mesh.normals[i], mesh.uv[i]));

        triangles = new List<int>[2];
        triangles[0] = new List<int>(mesh.GetTriangles(0));

        if (mesh.subMeshCount >= 2) triangles[1] = new List<int>(mesh.GetTriangles(1));
        else triangles[1] = new List<int>(mesh.triangles.Length / 10);
    }

    public void AddCutFaceVertex(float3 position, float3 normal, float2 uv)
    {
        MeshVertex vertex = new MeshVertex(position, normal, uv);
        vertices.Add(vertex); cutVertices.Add(vertex);
    }

    public void AddMappedVertex(MeshVertex vertex, int sourceIndex)
    {
        vertices.Add(vertex);
        indexMap[sourceIndex] = vertices.Count - 1;
    }

    public void AddTriangle(int v1, int v2, int v3, MeshType meshType)
    {
        triangles[(int)meshType].Add(v1);
        triangles[(int)meshType].Add(v2);
        triangles[(int)meshType].Add(v3);
    }

    public void AddMappedTriangle(int v1, int v2, int v3, MeshType meshType)
    {
        triangles[(int)meshType].Add(indexMap[v1]);
        triangles[(int)meshType].Add(indexMap[v2]);
        triangles[(int)meshType].Add(indexMap[v3]);
    }

    public void AddMeshData(MeshData other)
    {
        int vertOffset = vertices.Count, cutOffset = cutVertices.Count;
        int subMeshCount = triangles.Length; int[] triOffset = new int[subMeshCount];
        for (int s = 0; s < subMeshCount; s++)
            triOffset[s] = triangles[s].Count / 3;

        vertices.AddRange(other.vertices); cutVertices.AddRange(other.cutVertices);

        int oldMapLen = indexMap.Length;
        System.Array.Resize(ref indexMap, oldMapLen + other.indexMap.Length);
        for (int i = 0; i < other.indexMap.Length; i++)
            indexMap[oldMapLen + i] = other.indexMap[i] + vertOffset;

        for (int s = 0; s < subMeshCount; s++)
        {
            List<int> destTris = triangles[s], srcTris = other.triangles[s];
            for (int i = 0; i < srcTris.Count; i++)
                destTris.Add(srcTris[i] + vertOffset);
        }

        foreach (var e in other.constraints)
        {
            int newV1 = e.v1 + cutOffset, newV2 = e.v2 + cutOffset,
                newT1 = e.t1 >= 0 ? e.t1 + triOffset[0] : -1, newT2 = e.t2 >= 0 ? e.t2 + triOffset[0] : -1;
            var mergedEdge = new MeshEdge(newV1, newV2, newT1, newT2, e.t1Edge);
            constraints.Add(mergedEdge);
        }
    }

    public void WeldCutFaceVertices()
    {
        List<MeshVertex> weldedVerts = new List<MeshVertex>(cutVertices.Count);

        int[] indexMap = new int[cutVertices.Count]; int k = 0;

        for (int i = 0; i < cutVertices.Count; i++)
        {
            bool duplicate = false;
            for (int j = 0; j < weldedVerts.Count; j++)
            {
                if (Equals(cutVertices[i].position, weldedVerts[j].position))
                {
                    indexMap[i] = j;
                    duplicate = true;
                    break;
                }
            }

            if (!duplicate)
            {
                weldedVerts.Add(cutVertices[i]);
                indexMap[i] = k++;
            }
        }

        for (int i = 0; i < constraints.Count; i++)
        {
            MeshEdge edge = constraints[i];
            edge.v1 = indexMap[edge.v1];
            edge.v2 = indexMap[edge.v2];
        }

        weldedVerts.TrimExcess();
        cutVertices = new List<MeshVertex>(weldedVerts);
    }

    public Mesh ToMesh()
    {
        Mesh mesh = new Mesh(); int indexStart = 0;

        VertexAttributeDescriptor[] layout = new[]
        {
            new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3),
            new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3),
            new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2),
        };

        mesh.subMeshCount = triangles.Length;
        mesh.SetIndexBufferParams(triangleCount, IndexFormat.UInt32);
        mesh.SetVertexBufferParams(vertexCount, layout);
        mesh.SetVertexBufferData(vertices, 0, 0, vertices.Count);
        mesh.SetVertexBufferData(cutVertices, 0, vertices.Count, cutVertices.Count);

        for (int i = 0; i < triangles.Length; i++)
        {
            var subMeshIndexBuffer = triangles[i];
            mesh.SetIndexBufferData(subMeshIndexBuffer, 0, indexStart, subMeshIndexBuffer.Count);
            mesh.SetSubMesh(i, new SubMeshDescriptor(indexStart, subMeshIndexBuffer.Count));
            indexStart += subMeshIndexBuffer.Count;
        }

        mesh.RecalculateBounds();
        return mesh;
    }
}
