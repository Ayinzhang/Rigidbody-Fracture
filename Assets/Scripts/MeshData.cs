using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using Unity.Mathematics;
using System;

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

    public MeshData(List<MeshData> meshDatas)
    {
        vertices = new List<MeshVertex>();
        cutVertices = new List<MeshVertex>();
        triangles = new List<int>[] { new List<int>(), new List<int>() };
        constraints = new List<MeshEdge>();
        indexMap = new int[0];

        Dictionary<float3, int> vertexLookup = new Dictionary<float3, int>(new Float3Comparer());

        foreach (var data in meshDatas)
        {
            foreach (var v in data.vertices)
            {
                if (!vertexLookup.ContainsKey(v.position))
                {
                    vertexLookup[v.position] = vertices.Count;
                    vertices.Add(v);
                }
            }

            foreach (var v in data.cutVertices)
            {
                if (!vertexLookup.ContainsKey(v.position))
                {
                    vertexLookup[v.position] = vertices.Count;
                    vertices.Add(v);
                    cutVertices.Add(v);
                }
            }

            for (int sub = 0; sub < 2; sub++)
            {
                List<int> tris = data.triangles[sub];
                for (int i = 0; i < tris.Count; i += 3)
                {
                    var v1 = GetGlobalIndex(data, tris[i], vertexLookup);
                    var v2 = GetGlobalIndex(data, tris[i + 1], vertexLookup);
                    var v3 = GetGlobalIndex(data, tris[i + 2], vertexLookup);
                    triangles[sub].Add(v1);
                    triangles[sub].Add(v2);
                    triangles[sub].Add(v3);
                }
            }
        }
    }

    private int GetGlobalIndex(MeshData src, int index, Dictionary<float3, int> vertexLookup)
    {
        MeshVertex v = index < src.vertices.Count ? src.vertices[index] : src.cutVertices[index - src.vertices.Count];
        return vertexLookup[v.position];
    }

    class Float3Comparer : IEqualityComparer<float3>
    {
        public bool Equals(float3 a, float3 b) => math.distancesq(a, b) < 1e-6f;
        public int GetHashCode(float3 obj) => obj.GetHashCode();
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

    public void WeldCutFaceVertices()
    {
        List<MeshVertex> weldedVerts = new List<MeshVertex>(cutVertices.Count);

        int[] indexMap = new int[cutVertices.Count]; int k = 0;

        for (int i = 0; i < cutVertices.Count; i++)
        {
            bool duplicate = false;
            for (int j = 0; j < weldedVerts.Count; j++)
            {
                if (math.length(cutVertices[i].position - weldedVerts[j].position) < 1e-5)
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
        Mesh mesh = new Mesh();

        var layout = new[]
        {
            new VertexAttributeDescriptor(VertexAttribute.Position, VertexAttributeFormat.Float32, 3),
            new VertexAttributeDescriptor(VertexAttribute.Normal, VertexAttributeFormat.Float32, 3),
            new VertexAttributeDescriptor(VertexAttribute.TexCoord0, VertexAttributeFormat.Float32, 2),
        };

        mesh.SetIndexBufferParams(triangleCount, IndexFormat.UInt32);
        mesh.SetVertexBufferParams(vertexCount, layout);
        mesh.SetVertexBufferData(vertices, 0, 0, vertices.Count);
        mesh.SetVertexBufferData(cutVertices, 0, vertices.Count, cutVertices.Count);

        mesh.subMeshCount = triangles.Length;
        int indexStart = 0;
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

public struct Quad
{
    public int q1, q2, q3, q4, t1, t2, t1L, t1R, t2L, t2R;

    public Quad(int q1, int q2, int q3, int q4, int t1, int t2, int t1L, int t1R, int t2L, int t2R)
    {
        this.q1 = q1; this.q2 = q2; this.q3 = q3; this.q4 = q4; this.t1 = t1; this.t2 = t2; 
        this.t1L = t1L; this.t1R = t1R; this.t2L = t2L; this.t2R = t2R;
    }
}

public interface IBinSortable
{
    int bin { get; set; }
}

public class BinSort
{
    public static int GetBinNumber(int i, int j, int n)
    {
        return (i % 2 == 0) ? (i * n) + j : (i + 1) * n - j - 1;
    }

    public static T[] Sort<T>(T[] input, int lastIndex, int binCount) where T : IBinSortable
    {
        int[] count = new int[binCount];
        T[] output = new T[input.Length];

        if (binCount <= 1) return input;
        if (lastIndex > input.Length) lastIndex = input.Length;

        for (int i = 0; i < lastIndex; i++)
        {
            int j = input[i].bin;
            count[j] += 1;
        }

        for (int i = 1; i < binCount; i++) count[i] += count[i - 1];

        for (int i = lastIndex - 1; i >= 0; i--)
        {
            int j = input[i].bin;
            count[j] -= 1; output[count[j]] = input[i];
        }

        for (int i = lastIndex; i < output.Length; i++) output[i] = input[i];
        return output;
    }

}

public class TriangulationPoint : IBinSortable
{
    public float2 coords;
    public int bin { get; set; }
    public int index = 0;

    public TriangulationPoint(int index, float2 coords)
    {
        this.index = index;
        this.coords = coords;
    }
}