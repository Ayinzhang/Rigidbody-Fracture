using UnityEngine;
using UnityEngine.Rendering;
using System.Collections.Generic;
using Unity.Mathematics;

public enum MeshType
{
    Default = 0,
    CutFace = 1
}

public struct MeshVertex
{
    public Vector3 position, normal;
    public Vector2 uv;

    public MeshVertex(Vector3 position)
    {
        this.position = position; normal = Vector3.zero; uv = Vector2.zero;
    }

    public MeshVertex(Vector3 position, Vector3 normal, Vector2 uv)
    {
        this.position = position; this.normal = normal; this.uv = uv;
    }

    public override bool Equals(object obj)
    {
        if (!(obj is MeshVertex)) return false;

        return ((MeshVertex)obj).position.Equals(this.position);
    }

    public static bool operator ==(MeshVertex lhs, MeshVertex rhs)
    {
        return lhs.Equals(rhs);
    }

    public static bool operator !=(MeshVertex lhs, MeshVertex rhs)
    {
        return !lhs.Equals(rhs);
    }

    public override int GetHashCode()
    {
        return position.GetHashCode();
    }

    public override string ToString()
    {
        return $"Position = {position}, Normal = {normal}, UV = {uv}";
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

    public override bool Equals(object obj)
    {
        if (obj is MeshEdge)
        {
            MeshEdge other = (MeshEdge)obj;
            return (v1 == other.v1 && v2 == other.v2) || (v1 == other.v2 && v2 == other.v1);
        }
        return false;
    }

    public override int GetHashCode()
    {
        return new { v1, v2 }.GetHashCode() + new { v2, v1 }.GetHashCode();
    }

    public static bool operator ==(MeshEdge lhs, MeshEdge rhs)
    {
        return lhs.Equals(rhs);
    }

    public static bool operator !=(MeshEdge lhs, MeshEdge rhs)
    {
        return !lhs.Equals(rhs);
    }
}

public class MeshData
{
    public List<MeshVertex> vertices, cutVertices;
    public List<int>[] triangles; public List<MeshEdge> constraints;

    public int[] indexMap; public Bounds bounds;
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
        Vector3[] positions = mesh.vertices;
        Vector3[] normals = mesh.normals;
        Vector2[] uv = mesh.uv;

        vertices = new List<MeshVertex>(mesh.vertexCount);
        cutVertices = new List<MeshVertex>(mesh.vertexCount / 10);
        constraints = new List<MeshEdge>();
        indexMap = new int[positions.Length];

        for (int i = 0; i < positions.Length; i++) vertices.Add(new MeshVertex(positions[i], normals[i], uv[i]));

        triangles = new List<int>[2];
        triangles[0] = new List<int>(mesh.GetTriangles(0));

        if (mesh.subMeshCount >= 2) triangles[1] = new List<int>(mesh.GetTriangles(1));
        else triangles[1] = new List<int>(mesh.triangles.Length / 10);

        CalculateBounds();
    }

    public void AddCutFaceVertex(Vector3 position, Vector3 normal, Vector2 uv)
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
                if (cutVertices[i].position == weldedVerts[j].position)
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

    public int[] GetTriangles(int subMeshIndex)
    {
        return triangles[subMeshIndex].ToArray();
    }

    public void CalculateBounds()
    {
        float vertexCount = (float)vertices.Count;
        Vector3 min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

        foreach (MeshVertex vertex in vertices)
        {
            if (vertex.position.x < min.x) min.x = vertex.position.x;
            if (vertex.position.y < min.y) min.y = vertex.position.y;
            if (vertex.position.z < min.z) min.z = vertex.position.z;
            if (vertex.position.x > max.x) max.x = vertex.position.x;
            if (vertex.position.y > max.y) max.y = vertex.position.y;
            if (vertex.position.z > max.z) max.z = vertex.position.z;
        }

        bounds = new Bounds((max + min) / 2f, max - min);
    }

    public Mesh ToMesh()
    {
        Mesh mesh = new Mesh();

        VertexAttributeDescriptor[] layout = new[]
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
