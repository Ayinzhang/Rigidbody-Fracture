using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

public class MeshTriangulator
{
    public struct TriangulationPoint { public int index; public float2 coords; }

    List<MeshVertex> vertices;

    public TriangulationPoint[] points;
    public float normalizationScaleFactor;

    public MeshTriangulator(List<MeshVertex> cutVertices, List<MeshEdge> constraints, float3 normal)
    {
        vertices = cutVertices;
        int N = cutVertices.Count; points = new TriangulationPoint[N];

        float3 e1 = math.cross(normal, new float3(0, 1, 0));
        if (math.sqrt(e1.x * e1.x + e1.y * e1.y + e1.z * e1.z) < 1e-6f) e1 = math.cross(normal, new float3(1, 0, 0));
        float3 e2 = math.cross(normal, e1);

        float minX = float.MaxValue, maxX = float.MinValue;
        float minY = float.MaxValue, maxY = float.MinValue;
        float2[] raw = new float2[N];
        for (int i = 0; i < N; i++)
        {
            float3 p = vertices[i].position;
            float2 coord = new float2(math.dot(p, e1), math.dot(p, e2));
            raw[i] = coord;
            if (coord.x < minX) minX = coord.x;
            if (coord.x > maxX) maxX = coord.x;
            if (coord.y < minY) minY = coord.y;
            if (coord.y > maxY) maxY = coord.y;
        }

        float dx = maxX - minX, dy = maxY - minY;
        normalizationScaleFactor = Mathf.Max(dx, dy);
        if (normalizationScaleFactor < 1e-6f) normalizationScaleFactor = 1f;

        for (int i = 0; i < N; i++)
        {
            float2 c = (raw[i] - new float2(minX, minY)) / normalizationScaleFactor;
            points[i] = new TriangulationPoint { index = i, coords = c };
        }
    }

    public int[] Triangulate()
    {
        int N = points.Length;
        if (N < 3) return new int[0];

        List<int> indexList = new List<int>(N);
        for (int i = 0; i < N; i++) indexList.Add(i);

        List<int> tris = new List<int>();

        int guard = 0;
        while (indexList.Count > 3 && guard++ < N * N)
        {
            bool clipped = false;
            for (int j = 0; j < indexList.Count; j++)
            {
                int i0 = indexList[(j + indexList.Count - 1) % indexList.Count];
                int i1 = indexList[j];
                int i2 = indexList[(j + 1) % indexList.Count];
                float2 p0 = points[i0].coords;
                float2 p1 = points[i1].coords;
                float2 p2 = points[i2].coords;

                // Check convex (CCW)
                if (((p1.x - p0.x) * (p2.y - p0.y) - (p1.y - p0.y) * (p2.x - p0.x)) <= 0) continue;

                // Check no other point inside triangle
                bool anyInside = false;
                for (int k = 0; k < indexList.Count; k++)
                {
                    int ik = indexList[k];
                    if (ik == i0 || ik == i1 || ik == i2) continue;
                    if (PointInTriangle(points[ik].coords, p0, p1, p2)) { anyInside = true; break; }
                }
                if (anyInside) continue;

                // Clip ear
                tris.Add(i0);
                tris.Add(i1);
                tris.Add(i2);
                indexList.RemoveAt(j);
                clipped = true;
                break;
            }
            if (!clipped) break;
        }
        // Final triangle
        if (indexList.Count == 3)
        {
            tris.Add(indexList[0]); tris.Add(indexList[1]); tris.Add(indexList[2]);
        }
        return tris.ToArray();
    }

    private bool PointInTriangle(float2 p, float2 a, float2 b, float2 c)
    {
        float areaOrig = Mathf.Abs((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
        float area1 = Mathf.Abs((a.x - p.x) * (b.y - p.y) - (a.y - p.y) * (b.x - p.x));
        float area2 = Mathf.Abs((b.x - p.x) * (c.y - p.y) - (b.y - p.y) * (c.x - p.x));
        float area3 = Mathf.Abs((c.x - p.x) * (a.y - p.y) - (c.y - p.y) * (a.x - p.x));
        return Mathf.Abs(area1 + area2 + area3 - areaOrig) < 0;
    }
}

public static class MeshProjector
{
    static float2[] points; static float2 minLeft, maxRight, projPoint, center2d;
    static float3 center, worldNormal, tangent, bitangent;

    public static void GetSlice(MeshData meshData, Transform trans, float3 point, float3 normal, 
        float sliceRate, float2 sliceTilt, out float3 sliceNormal, out float3 sliceOrigin, out bool isFullSlice)
    {
        points = new float2[meshData.vertices.Count];
        minLeft = new float2(float.MaxValue, float.MaxValue);
        maxRight = new float2(float.MinValue, float.MinValue);
        point = trans.InverseTransformPoint(point); worldNormal = normal;
        normal = trans.InverseTransformDirection(normal).normalized;
        tangent = math.normalize(math.cross(normal, new float3(0, 1, 0)));
        bitangent = math.normalize(math.cross(normal, tangent));
        for (int i = 0; i < meshData.vertices.Count; i++)
            center += meshData.vertices[i].position; center /= meshData.vertices.Count;
        float3 offset = point - center; projPoint = new float2(math.dot(offset, tangent), math.dot(offset, bitangent));
        for (int i = 0; i < meshData.vertices.Count; i++)
        {
            offset = meshData.vertices[i].position - center;
            points[i] = new float2(math.dot(offset, tangent), math.dot(offset, bitangent)); center2d += points[i];
            minLeft.x = math.min(minLeft.x, points[i].x); minLeft.y = math.min(minLeft.y, points[i].y);
            maxRight.x = math.max(maxRight.x, points[i].x); maxRight.y = math.max(maxRight.y, points[i].y);
        }
        center2d /= meshData.vertices.Count;

        float halfWidth = (maxRight.x - minLeft.x) * 0.5f, halfHeight = (maxRight.y - minLeft.y) * 0.5f;
        float2 areaCenter2d = new float2((minLeft.x + maxRight.x) * 0.5f, (minLeft.y + maxRight.y) * 0.5f);
        isFullSlice = math.length(areaCenter2d - minLeft) >  2 * math.length(areaCenter2d - projPoint);
        int l = 0, r = 180, m = (l + r) / 2; float angle = math.radians(m);
        while (l < r) 
        {
            if (angle - math.sin(angle) >= 2 * sliceRate * math.PI) r = m;
            else l = m + 1;
            m = (l + r) / 2; angle = math.radians(m);
        }
        float theta = UnityEngine.Random.Range(0f, 2 * math.PI);
        float2 lineP0 = center2d + new float2(math.cos(theta) * halfWidth, math.sin(theta) * halfHeight);
        float sign = (isFullSlice ? 1 : -1) * math.sign((lineP0.y - center2d.y) * (projPoint.x - center2d.x) -
            (lineP0.x - center2d.x) * (projPoint.y - center2d.y));
        float2 lineP1 = center2d + new float2(math.cos(theta + sign * angle) * halfWidth, math.sin(theta + sign * angle) * halfHeight);

        float3 P0 = sliceOrigin = center + tangent * lineP0.x + bitangent * lineP0.y, 
               P1 = center + tangent * lineP1.x + bitangent * lineP1.y,
               worldP0 = math.mul(trans.localToWorldMatrix, new float4(P0, 1)).xyz, 
               worldP1 = math.mul(trans.localToWorldMatrix, new float4(P1, 1)).xyz;

        float3 worldDir = math.normalize(worldP1 - worldP0);
        float3 baseNormal = math.normalize(math.cross(worldDir, worldNormal));

        float tilt = UnityEngine.Random.Range(sliceTilt.x, sliceTilt.y);
        quaternion q = quaternion.AxisAngle(worldDir, (isFullSlice ? 1 : -1) * math.radians(tilt));
        sliceNormal = math.normalize(math.mul(new float4(math.mul(q, baseNormal), 0), trans.localToWorldMatrix)).xyz;

        if (math.dot(point - P0, sliceNormal) < 0) sliceNormal = -sliceNormal;
    }
}

public static class MeshSlicer
{
    public static void Slice(MeshData meshData, float3 sliceNormal, float3 sliceOrigin, out MeshData topSlice, out MeshData bottomSlice)
    {
        topSlice = new MeshData(meshData.vertexCount, meshData.triangleCount);
        bottomSlice = new MeshData(meshData.vertexCount, meshData.triangleCount);

        // Keep track of what side of the cutting plane each vertex is on
        bool[] side = new bool[meshData.vertexCount];

        // Go through and identify which vertices are above/below the split plane
        for (int i = 0; i < meshData.vertices.Count; i++)
        {
            var vertex = meshData.vertices[i];
            side[i] = math.dot(vertex.position - sliceOrigin, sliceNormal) >= 0;
            var slice = side[i] ? topSlice : bottomSlice;
            slice.AddMappedVertex(vertex, i);
        }

        int offset = meshData.vertices.Count;
        for (int i = 0; i < meshData.cutVertices.Count; i++)
        {
            var vertex = meshData.cutVertices[i];
            side[i + offset] = math.dot(vertex.position - sliceOrigin, sliceNormal) >= 0;
            var slice = side[i + offset] ? topSlice : bottomSlice;
            slice.AddMappedVertex(vertex, i + offset);
        }

        SplitTriangles(meshData, topSlice, bottomSlice, sliceNormal, sliceOrigin, side, MeshType.Default);
        SplitTriangles(meshData, topSlice, bottomSlice, sliceNormal, sliceOrigin, side, MeshType.CutFace);

        FillCutFaces(topSlice, bottomSlice, -sliceNormal);
    }

    /// <summary>
    /// Fills the cut faces for each sliced mesh. The `sliceNormal` is the normal for the plane and points
    /// in the direction of `topMeshData`
    /// </summary>
    /// <param name="topSlice">Fragment mesh data for slice above the slice plane</param>
    /// <param name="bottomSlice">Fragment mesh data for slice above the slice plane</param>
    /// <param name="sliceNormal">Normal of the slice plane (points towards the top slice)</param>
    /// <param name="textureScale">Scale factor to apply to UV coordinates</param>
    /// <param name="textureOffset">Offset to apply to UV coordinates</param>
    private static void FillCutFaces(MeshData topSlice,
                                     MeshData bottomSlice,
                                     float3 sliceNormal)
    {
        // Since the topSlice and bottomSlice both share the same cut face, we only need to calculate it
        // once. Then the same vertex/triangle data for the face will be used for both slices, except
        // with the normals reversed.

        // First need to weld the coincident vertices for the triangulation to work properly
        topSlice.WeldCutFaceVertices();

        // Need at least 3 vertices to triangulate
        if (topSlice.cutVertices.Count < 3) return;

        // Triangulate the cut face
        var triangulator = new MeshTriangulator(topSlice.cutVertices, topSlice.constraints, sliceNormal);
        int[] triangles = triangulator.Triangulate();

        // Update normal and UV for the cut face vertices
        for (int i = 0; i < topSlice.cutVertices.Count; i++)
        {
            var vertex = topSlice.cutVertices[i];
            var point = triangulator.points[i];

            // UV coordinates are based off of the 2D coordinates used for triangulation
            // During triangulation, coordinates are normalized to [0,1], so need to multiply
            // by normalization scale factor to get back to the appropritate scale
            float2 uv = new float2(triangulator.normalizationScaleFactor * point.coords.x,
                triangulator.normalizationScaleFactor * point.coords.y);

            // Update normals and UV coordinates for the cut vertices
            var topVertex = vertex;
            topVertex.normal = sliceNormal;
            topVertex.uv = uv;

            var bottomVertex = vertex;
            bottomVertex.normal = -sliceNormal;
            bottomVertex.uv = uv;

            topSlice.cutVertices[i] = topVertex;
            bottomSlice.cutVertices[i] = bottomVertex;
        }

        // Add the new triangles to the top/bottom slices
        int offsetTop = topSlice.vertices.Count;
        int offsetBottom = bottomSlice.vertices.Count;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            topSlice.AddTriangle(
                offsetTop + triangles[i],
                offsetTop + triangles[i + 1],
                offsetTop + triangles[i + 2],
                MeshType.CutFace);

            bottomSlice.AddTriangle(
                offsetBottom + triangles[i],
                offsetBottom + triangles[i + 2], // Swap two vertices so triangles are wound CW
                offsetBottom + triangles[i + 1],
                MeshType.CutFace);
        }
    }

    /// <summary>
    /// Identifies triangles that are intersected by the slice plane and splits them in two
    /// </summary>
    /// <param name="meshData"></param>
    /// <param name="topSlice">Fragment mesh data for slice above the slice plane</param>
    /// <param name="bottomSlice">Fragment mesh data for slice above the slice plane</param>
    /// <param name="sliceNormal">The normal of the slice plane (points towards the top slice)</param>
    /// <param name="sliceOrigin">The origin of the slice plane</param>
    /// <param name="side">Array mapping each vertex to either the top/bottom slice</param>
    /// <param name="type">Index of the sub mesh</param>
    private static void SplitTriangles(MeshData meshData,
                                       MeshData topSlice,
                                       MeshData bottomSlice,
                                       float3 sliceNormal,
                                       float3 sliceOrigin,
                                       bool[] side,
                                       MeshType type)
    {
        int[] triangles = meshData.triangles[(int)type].ToArray();

        // Keep track of vertices that lie on the intersection plane
        int a, b, c;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            // Get vertex indexes for this triangle
            a = triangles[i];
            b = triangles[i + 1];
            c = triangles[i + 2];

            // Triangle is contained completely within mesh A
            if (side[a] && side[b] && side[c])
            {
                topSlice.AddMappedTriangle(a, b, c, type);
            }
            // Triangle is contained completely within mesh B
            else if (!side[a] && !side[b] && !side[c])
            {
                bottomSlice.AddMappedTriangle(a, b, c, type);
            }
            // Triangle is intersected by the slicing plane. Need to subdivide it
            else
            {
                // In these cases, two vertices of the triangle are above the cut plane and one vertex is below
                if (side[b] && side[c] && !side[a])
                {
                    SplitTriangle(b, c, a, sliceNormal, sliceOrigin, meshData, topSlice, bottomSlice, type, true);
                }
                else if (side[c] && side[a] && !side[b])
                {
                    SplitTriangle(c, a, b, sliceNormal, sliceOrigin, meshData, topSlice, bottomSlice, type, true);
                }
                else if (side[a] && side[b] && !side[c])
                {
                    SplitTriangle(a, b, c, sliceNormal, sliceOrigin, meshData, topSlice, bottomSlice, type, true);
                }
                // In these cases, two vertices of the triangle are below the cut plane and one vertex is above
                else if (!side[b] && !side[c] && side[a])
                {
                    SplitTriangle(b, c, a, sliceNormal, sliceOrigin, meshData, topSlice, bottomSlice, type, false);
                }
                else if (!side[c] && !side[a] && side[b])
                {
                    SplitTriangle(c, a, b, sliceNormal, sliceOrigin, meshData, topSlice, bottomSlice, type, false);
                }
                else if (!side[a] && !side[b] && side[c])
                {
                    SplitTriangle(a, b, c, sliceNormal, sliceOrigin, meshData, topSlice, bottomSlice, type, false);
                }
            }
        }
    }

    /// <summary>
    /// Splits triangle defined by the points (v1,v2,v3)
    /// </summary>
    /// <param name="v1_idx">Index of first vertex in triangle</param>
    /// <param name="v2_idx">Index of second vertex in triangle<</param>
    /// <param name="v3_idx">Index of third vertex in triangle<</param>
    /// <param name="sliceNormal">The normal of the slice plane (points towards the top slice)</param>
    /// <param name="sliceOrigin">The origin of the slice plane</param>
    /// <param name="meshData">Original mesh data</param>
    /// <param name="topSlice">Mesh data for top slice</param>
    /// <param name="bottomSlice">Mesh data for bottom slice</param>
    /// <param name="type">Index of the submesh that the triangle belongs to</param>
    /// <param name="v3BelowCutPlane">Boolean indicating whether v3 is above or below the slice plane.</param>                                             
    private static void SplitTriangle(int v1_idx,
                                      int v2_idx,
                                      int v3_idx,
                                      float3 sliceNormal,
                                      float3 sliceOrigin,
                                      MeshData meshData,
                                      MeshData topSlice,
                                      MeshData bottomSlice,
                                      MeshType type,
                                      bool v3BelowCutPlane)
    {
        // - `v1`, `v2`, `v3` are the indexes of the triangle relative to the original mesh data
        // - `v1` and `v2` are on the the side of split plane that belongs to meshA
        // - `v3` is on the side of the split plane that belongs to meshB
        // - `vertices`, `normals`, `uv` are the original mesh data used for interpolation  
        //      
        // v3BelowCutPlane = true
        // ======================
        //                                
        //     v1 *_____________* v2   .
        //         \           /      /|\  cutNormal
        //          \         /        |
        //       ----*-------*---------*--
        //        v13 \     /  v23       cutOrigin
        //             \   /
        //              \ /
        //               *  v3         triangle normal out of screen                                                                                  
        //    
        // v3BelowCutPlane = false
        // =======================
        //
        //               *  v3         .                                             
        //              / \           /|\  cutNormal  
        //         v23 /   \ v13       |                    
        //       -----*-----*----------*--
        //           /       \         cut origin                                
        //          /         \                                                                  
        //      v2 *___________* v1    triangle normal out of screen
        //                 

        float s13;
        float s23;
        float3 v13;
        float3 v23;

        MeshVertex v1 = v1_idx < meshData.vertices.Count ? meshData.vertices[v1_idx] : meshData.cutVertices[v1_idx - meshData.vertices.Count];
        MeshVertex v2 = v2_idx < meshData.vertices.Count ? meshData.vertices[v2_idx] : meshData.cutVertices[v2_idx - meshData.vertices.Count];
        MeshVertex v3 = v3_idx < meshData.vertices.Count ? meshData.vertices[v3_idx] : meshData.cutVertices[v3_idx - meshData.vertices.Count];

        if (LinePlaneIntersection(v1.position, v3.position, sliceNormal, sliceOrigin, out v13, out s13) &&
            LinePlaneIntersection(v2.position, v3.position, sliceNormal, sliceOrigin, out v23, out s23))
        {
            // Interpolate normals and UV coordinates
            var norm13 = math.normalize(v1.normal + s13 * (v3.normal - v1.normal));
            var norm23 = math.normalize(v2.normal + s23 * (v3.normal - v2.normal));
            var uv13 = v1.uv + s13 * (v3.uv - v1.uv);
            var uv23 = v2.uv + s23 * (v3.uv - v2.uv);

            // Add vertices/normals/uv for the intersection points to each mesh
            topSlice.AddCutFaceVertex(v13, norm13, uv13);
            topSlice.AddCutFaceVertex(v23, norm23, uv23);
            bottomSlice.AddCutFaceVertex(v13, norm13, uv13);
            bottomSlice.AddCutFaceVertex(v23, norm23, uv23);

            // Indices for the intersection vertices (for the original mesh data)
            int index13_A = topSlice.vertices.Count - 2;
            int index23_A = topSlice.vertices.Count - 1;
            int index13_B = bottomSlice.vertices.Count - 2;
            int index23_B = bottomSlice.vertices.Count - 1;

            if (v3BelowCutPlane)
            {
                // Triangle slice above the cutting plane is a quad, so divide into two triangles
                topSlice.AddTriangle(index23_A, index13_A, topSlice.indexMap[v2_idx], type);
                topSlice.AddTriangle(index13_A, topSlice.indexMap[v1_idx], topSlice.indexMap[v2_idx], type);

                // One triangle must be added to mesh 2
                bottomSlice.AddTriangle(bottomSlice.indexMap[v3_idx], index13_B, index23_B, type);

                // When looking at the cut-face, the edges should wind counter-clockwise
                topSlice.constraints.Add(new MeshEdge(topSlice.cutVertices.Count - 2, topSlice.cutVertices.Count - 1));
                bottomSlice.constraints.Add(new MeshEdge(bottomSlice.cutVertices.Count - 1, bottomSlice.cutVertices.Count - 2));
            }
            else
            {
                // Triangle slice above the cutting plane is a simple triangle
                topSlice.AddTriangle(index13_A, index23_A, topSlice.indexMap[v3_idx], type);

                // Triangle slice below the cutting plane is a quad, so divide into two triangles
                bottomSlice.AddTriangle(bottomSlice.indexMap[v1_idx], bottomSlice.indexMap[v2_idx], index13_B, type);
                bottomSlice.AddTriangle(bottomSlice.indexMap[v2_idx], index23_B, index13_B, type);

                // When looking at the cut-face, the edges should wind counter-clockwise
                topSlice.constraints.Add(new MeshEdge(topSlice.cutVertices.Count - 1, topSlice.cutVertices.Count - 2));
                bottomSlice.constraints.Add(new MeshEdge(bottomSlice.cutVertices.Count - 2, bottomSlice.cutVertices.Count - 1));
            }
        }
    }

    private static bool LinePlaneIntersection(float3 a,
                                             float3 b,
                                             float3 n,
                                             float3 p0,
                                             out float3 x,
                                             out float s)
    {
        // Initialize out params
        s = 0;
        x = float3.zero;

        // Handle degenerate cases
        if (Equals(a, b))
        {
            return false;
        }
        else if (Equals(n, float3.zero))
        {
            return false;
        }

        // `s` is the parameter for the line segment a -> b where 0.0 <= s <= 1.0
        s = math.dot(p0 - a, n) / math.dot(b - a, n);

        if (s >= 0 && s <= 1)
        {
            x = a + (b - a) * s;
            return true;
        }

        return false;
    }
}