using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

public static class MathUtils
{
    public static bool IsQuadConvex(float2 a1, float2 a2, float2 b1, float2 b2)
    {
        return LinesIntersectInternal(a1, a2, b1, b2, true);
    }

    public static bool LinesIntersect(float2 a1, float2 a2, float2 b1, float2 b2)
    {
        return LinesIntersectInternal(a1, a2, b1, b2, false);
    }

    private static bool LinesIntersectInternal(float2 a1, float2 a2, float2 b1, float2 b2, bool includeSharedEndpoints)
    {
        float2 a12 = new float2(a2.x - a1.x, a2.y - a1.y);
        float2 b12 = new float2(b2.x - b1.x, b2.y - b1.y);

        if (math.length(a1 - b1) < 1e-5 || math.length(a1 - b2) < 1e-5 || math.length(a2 - b1) < 1e-5 || math.length(a2 - b2) < 1e-5)
        {
            return includeSharedEndpoints;
        }
        else
        {
            float a1xb = (a1.x - b1.x) * b12.y - (a1.y - b1.y) * b12.x;
            float a2xb = (a2.x - b1.x) * b12.y - (a2.y - b1.y) * b12.x;
            float b1xa = (b1.x - a1.x) * a12.y - (b1.y - a1.y) * a12.x;
            float b2xa = (b2.x - a1.x) * a12.y - (b2.y - a1.y) * a12.x;

            return ((a1xb >= 0 && a2xb <= 0) || (a1xb <= 0 && a2xb >= 0)) &&
                   ((b1xa >= 0 && b2xa <= 0) || (b1xa <= 0 && b2xa >= 0));
        }
    }

    public static bool LinePlaneIntersection(float3 a,
                                             float3 b,
                                             float3 n,
                                             float3 p0,
                                             out float3 x,
                                             out float s)
    {
        s = 0;
        x = float3.zero;

        if (math.length(a - b) < 1e-5)
        {
            return false;
        }
        else if (math.length(n) < 1e-5)
        {
            return false;
        }

        s = math.dot(p0 - a, n) / math.dot(b - a, n);

        if (s >= 0 && s <= 1)
        {
            x = a + (b - a) * s;
            return true;
        }

        return false;
    }

    public static bool IsPointOnRightSideOfLine(float2 a, float2 b, float2 c)
    {
        return ((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)) <= 0;
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
        float2 normPoint = projPoint - center2d; normPoint.x /= halfWidth; normPoint.y /= halfHeight;
        isFullSlice = math.length(normPoint) < 0.7f;
        
        float angle = UnityEngine.Random.Range(0, 2 * math.PI);
        float2 dir = new float2(Mathf.Cos(angle), Mathf.Sin(angle));
        float2 tan = new float2(-dir.y, dir.x);
        int index = -1; float maxDis = 0;
        for (int i = 0; i < points.Length; i++) 
        {
            float dis = math.dot(points[i] - projPoint, tan);
            if (math.abs(dis) > math.abs(maxDis)) { maxDis = dis; index = i; }
        }
        float2 lineP0 = projPoint + tan * maxDis * (1 - sliceRate), lineP1 = lineP0 + dir;

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
        //if(!isFullSlice) sliceNormal = -sliceNormal;
    }
}

public static class MeshTriangulator
{
    const int V1 = 0, V2 = 1, V3 = 2, E12 = 3, E23 = 4, E31 = 5, SUPERTRIANGLE = 0, OUT_OF_BOUNDS = -1;
    static int N, triangleCount;
    static int[] vertexTriangles, edgeVertex1, edgeVertex2, oppositePoint, nextEdge, previousEdge;
    static int[,] triangulation;
    static bool[] visited, skipTriangle; static List<MeshEdge> constraints;

    static public TriangulationPoint[] points; static public float normalizationScaleFactor = 1f;

    static public int[] Triangulate(List<MeshVertex> inputPoints, List<MeshEdge> constraints, float3 normal)
    {
        MeshTriangulator.constraints = constraints;
        if (inputPoints == null || inputPoints.Count < 3) return null;

        N = inputPoints.Count;
        triangleCount = 2 * N + 1;
        triangulation = new int[triangleCount, 6];
        skipTriangle = new bool[triangleCount];
        points = new TriangulationPoint[N + 3];
        edgeVertex1 = new int[] { 0, 0, 0, V1, V2, V3 };
        edgeVertex2 = new int[] { 0, 0, 0, V2, V3, V1 };
        oppositePoint = new int[] { 0, 0, 0, V3, V1, V2 };
        nextEdge = new int[] { 0, 0, 0, E23, E31, E12 };
        previousEdge = new int[] { 0, 0, 0, E31, E12, E23 };

        float3 e1 = math.normalize(inputPoints[0].position - inputPoints[1].position);
        float3 e2 = math.normalize(normal);
        float3 e3 = math.normalize(math.cross(e1, e2));

        for (int i = 0; i < N; i++)
        {
            var position = inputPoints[i].position;
            var coords = new float2(math.dot(position, e1), math.dot(position, e3));
            points[i] = new TriangulationPoint(i, coords);
        }

        if (N < 3) return null;

        AddSuperTriangle();
        NormalizeCoordinates();
        ComputeTriangulation();

        if (constraints.Count > 0)
        {
            ApplyConstraints();
            DiscardTrianglesViolatingConstraints();
        }

        DiscardTrianglesWithSuperTriangleVertices();

        List<int> triangles = new List<int>(3 * triangleCount);
        for (int i = 0; i < triangleCount; i++)
        {
            if (!skipTriangle[i])
            {
                triangles.Add(triangulation[i, V1]);
                triangles.Add(triangulation[i, V2]);
                triangles.Add(triangulation[i, V3]);
            }
        }

        return triangles.ToArray();
    }

    static void NormalizeCoordinates()
    {
        float xMin = float.MaxValue;
        float xMax = float.MinValue;
        float yMin = float.MaxValue;
        float yMax = float.MinValue;

        for (int i = 0; i < N; i++)
        {
            var point = points[i];
            if (point.coords.x < xMin) xMin = point.coords.x;
            if (point.coords.y < yMin) yMin = point.coords.y;
            if (point.coords.x > xMax) xMax = point.coords.x;
            if (point.coords.y > yMax) yMax = point.coords.y;
        }

        normalizationScaleFactor = Mathf.Max(xMax - xMin, yMax - yMin);

        for (int i = 0; i < N; i++)
        {
            var point = points[i];
            var normalizedPos = new float2(
                (point.coords.x - xMin) / normalizationScaleFactor,
                (point.coords.y - yMin) / normalizationScaleFactor);

            points[i].coords = normalizedPos;
        }
    }

    static TriangulationPoint[] SortPointsIntoBins()
    {
        int n = Mathf.RoundToInt(Mathf.Pow(N, 0.25f));
        int binCount = n * n;

        for (int k = 0; k < N; k++)
        {
            var point = points[k];
            int i = (int)(0.99f * n * point.coords.y);
            int j = (int)(0.99f * n * point.coords.x);
            point.bin = BinSort.GetBinNumber(i, j, n);
        }

        return BinSort.Sort<TriangulationPoint>(points, N, binCount);
    }

    static bool ComputeTriangulation()
    {
        int tSearch = 0;
        int tLast = 0;

        var sortedPoints = SortPointsIntoBins();

        for (int i = 0; i < N; i++)
        {
            TriangulationPoint point = sortedPoints[i];

            int counter = 0;
            bool pointInserted = false;
            while (!pointInserted)
            {
                if (counter++ > tLast || tSearch == OUT_OF_BOUNDS)
                {
                    break;
                }

                var v1 = points[triangulation[tSearch, V1]].coords;
                var v2 = points[triangulation[tSearch, V2]].coords;
                var v3 = points[triangulation[tSearch, V3]].coords;

                if (!MathUtils.IsPointOnRightSideOfLine(v1, v2, point.coords))
                {
                    tSearch = triangulation[tSearch, E12];
                }
                else if (!MathUtils.IsPointOnRightSideOfLine(v2, v3, point.coords))
                {
                    tSearch = triangulation[tSearch, E23];
                }
                else if (!MathUtils.IsPointOnRightSideOfLine(v3, v1, point.coords))
                {
                    tSearch = triangulation[tSearch, E31];
                }
                else
                {
                    InsertPointIntoTriangle(point, tSearch, tLast);
                    tLast += 2;
                    tSearch = tLast;
                    pointInserted = true;
                }
            }
        }

        return true;
    }

    static void AddSuperTriangle()
    {
        points[N] = new TriangulationPoint(N, new float2(-100f, -100f));
        points[N + 1] = new TriangulationPoint(N + 1, new float2(0f, 100f));
        points[N + 2] = new TriangulationPoint(N + 2, new float2(100f, -100f));

        triangulation[SUPERTRIANGLE, V1] = N;
        triangulation[SUPERTRIANGLE, V2] = N + 1;
        triangulation[SUPERTRIANGLE, V3] = N + 2;
        triangulation[SUPERTRIANGLE, E12] = OUT_OF_BOUNDS;
        triangulation[SUPERTRIANGLE, E23] = OUT_OF_BOUNDS;
        triangulation[SUPERTRIANGLE, E31] = OUT_OF_BOUNDS;
    }

    static void InsertPointIntoTriangle(TriangulationPoint p, int t, int triangleCount)
    {
        int t1 = t;
        int t2 = triangleCount + 1;
        int t3 = triangleCount + 2;

        triangulation[t2, V1] = p.index;
        triangulation[t2, V2] = triangulation[t, V2];
        triangulation[t2, V3] = triangulation[t, V3];

        triangulation[t2, E12] = t3;
        triangulation[t2, E23] = triangulation[t, E23];
        triangulation[t2, E31] = t1;

        triangulation[t3, V1] = p.index;
        triangulation[t3, V2] = triangulation[t, V1];
        triangulation[t3, V3] = triangulation[t, V2];

        triangulation[t3, E12] = t1;
        triangulation[t3, E23] = triangulation[t, E12];
        triangulation[t3, E31] = t2;

        UpdateAdjacency(triangulation[t, E12], t, t3);
        UpdateAdjacency(triangulation[t, E23], t, t2);

        triangulation[t1, V2] = triangulation[t, V3];
        triangulation[t1, V3] = triangulation[t, V1];
        triangulation[t1, V1] = p.index;

        triangulation[t1, E23] = triangulation[t, E31];
        triangulation[t1, E12] = t2;
        triangulation[t1, E31] = t3;

        RestoreDelauneyTriangulation(p, t1, t2, t3);
    }

    static void RestoreDelauneyTriangulation(TriangulationPoint p, int t1, int t2, int t3)
    {
        int t4;
        Stack<(int, int)> s = new Stack<(int, int)>();

        s.Push((t1, triangulation[t1, E23]));
        s.Push((t2, triangulation[t2, E23]));
        s.Push((t3, triangulation[t3, E23]));

        while (s.Count > 0)
        {
            (t1, t2) = s.Pop();

            if (t2 == OUT_OF_BOUNDS) continue;
            else if (SwapQuadDiagonalIfNeeded(p.index, t1, t2, out t3, out t4))
            {
                s.Push((t1, t3));
                s.Push((t2, t4));
            }
        }
    }

    static bool SwapQuadDiagonalIfNeeded(int p, int t1, int t2, out int t3, out int t4)
    {
        int q4 = p;
        int q1, q2, q3;
        if (triangulation[t2, E12] == t1)
        {
            q1 = triangulation[t2, V2];
            q2 = triangulation[t2, V1];
            q3 = triangulation[t2, V3];

            t3 = triangulation[t2, E23];
            t4 = triangulation[t2, E31];
        }
        else if (triangulation[t2, E23] == t1)
        {
            q1 = triangulation[t2, V3];
            q2 = triangulation[t2, V2];
            q3 = triangulation[t2, V1];

            t3 = triangulation[t2, E31];
            t4 = triangulation[t2, E12];
        }
        else
        {
            q1 = triangulation[t2, V1];
            q2 = triangulation[t2, V3];
            q3 = triangulation[t2, V2];

            t3 = triangulation[t2, E12];
            t4 = triangulation[t2, E23];
        }

        if (SwapTest(points[q1].coords, points[q2].coords, points[q3].coords, points[q4].coords))
        {
            UpdateAdjacency(t3, t2, t1);
            UpdateAdjacency(triangulation[t1, E31], t1, t2);

            triangulation[t1, V1] = q4;
            triangulation[t1, V2] = q1;
            triangulation[t1, V3] = q3;

            triangulation[t2, V1] = q4;
            triangulation[t2, V2] = q3;
            triangulation[t2, V3] = q2;

            triangulation[t2, E12] = t1;
            triangulation[t2, E23] = t4;
            triangulation[t2, E31] = triangulation[t1, E31];

            triangulation[t1, E23] = t3;
            triangulation[t1, E31] = t2;

            return true;
        }
        else
        {
            return false;
        }
    }

    static void DiscardTrianglesWithSuperTriangleVertices()
    {
        for (int i = 0; i < triangleCount; i++)
        {
            if (TriangleContainsVertex(i, N) ||
                TriangleContainsVertex(i, N + 1) ||
                TriangleContainsVertex(i, N + 2))
            {
                skipTriangle[i] = true;
            }
        }
    }

    static bool SwapTest(float2 v1, float2 v2, float2 v3, float2 v4)
    {
        float x13 = v1.x - v3.x;
        float x23 = v2.x - v3.x;
        float y13 = v1.y - v3.y;
        float y23 = v2.y - v3.y;
        float x14 = v1.x - v4.x;
        float x24 = v2.x - v4.x;
        float y14 = v1.y - v4.y;
        float y24 = v2.y - v4.y;

        float cosA = x13 * x23 + y13 * y23;
        float cosB = x24 * x14 + y24 * y14;

        if (cosA >= 0 && cosB >= 0)
        {
            return false;
        }
        else if (cosA < 0 && cosB < 0)
        {
            return true;
        }
        else
        {
            float sinA = (x13 * y23 - x23 * y13);
            float sinB = (x24 * y14 - x14 * y24);
            float sinAB = sinA * cosB + sinB * cosA;
            return sinAB < 0;
        }
    }

    static bool TriangleContainsVertex(int t, int v)
    {
        return triangulation[t, V1] == v || triangulation[t, V2] == v || triangulation[t, V3] == v;
    }

    static void UpdateAdjacency(int t, int tOld, int tNew)
    {
        int sharedEdge;
        if (t == OUT_OF_BOUNDS) return;
        else if (FindSharedEdge(t, tOld, out sharedEdge)) triangulation[t, sharedEdge] = tNew;
    }

    static bool FindSharedEdge(int tOrigin, int tAdjacent, out int edgeIndex)
    {
        edgeIndex = 0;

        if (tOrigin == OUT_OF_BOUNDS)
        {
            return false;
        }
        else if (triangulation[tOrigin, E12] == tAdjacent)
        {
            edgeIndex = E12;
            return true;
        }
        else if (triangulation[tOrigin, E23] == tAdjacent)
        {
            edgeIndex = E23;
            return true;
        }
        else if (triangulation[tOrigin, E31] == tAdjacent)
        {
            edgeIndex = E31;
            return true;
        }
        else
        {
            return false;
        }
    }

    static void ApplyConstraints()
    {
        visited = new bool[triangulation.GetLength(0)];

        vertexTriangles = new int[N + 3];
        for (int i = 0; i < triangulation.GetLength(0); i++)
        {
            vertexTriangles[triangulation[i, V1]] = i;
            vertexTriangles[triangulation[i, V2]] = i;
            vertexTriangles[triangulation[i, V3]] = i;
        }

        foreach (MeshEdge constraint in constraints)
        {
            if (constraint.v1 == constraint.v2) continue;
            Queue<MeshEdge> intersectingEdges = FindIntersectingEdges(constraint, vertexTriangles);
            RemoveIntersectingEdges(constraint, intersectingEdges);
        }
    }

    static Queue<MeshEdge> FindIntersectingEdges(MeshEdge constraint, int[] vertexTriangles)
    {
        Queue<MeshEdge> intersectingEdges = new Queue<MeshEdge>();

        MeshEdge startEdge;
        if (FindStartingEdge(vertexTriangles, constraint, out startEdge))
        {
            intersectingEdges.Enqueue(startEdge);
        }
        else
        {
            return intersectingEdges;
        }

        int t = startEdge.t1;
        int edgeIndex = startEdge.t1Edge;
        int lastTriangle = t;
        bool finalTriangleFound = false;
        while (!finalTriangleFound)
        {
            lastTriangle = t;
            t = triangulation[t, edgeIndex];

            float2 v_i = points[constraint.v1].coords;
            float2 v_j = points[constraint.v2].coords;
            float2 v1 = points[triangulation[t, V1]].coords;
            float2 v2 = points[triangulation[t, V2]].coords;
            float2 v3 = points[triangulation[t, V3]].coords;

            if (TriangleContainsVertex(t, constraint.v2))
            {
                finalTriangleFound = true;
            }
            else if ((triangulation[t, E12] != lastTriangle) && MathUtils.LinesIntersect(v_i, v_j, v1, v2))
            {
                edgeIndex = E12;
                var edge = new MeshEdge(triangulation[t, V1], triangulation[t, V2], t, triangulation[t, E12], edgeIndex);
                intersectingEdges.Enqueue(edge);
            }
            else if ((triangulation[t, E23] != lastTriangle) && MathUtils.LinesIntersect(v_i, v_j, v2, v3))
            {
                edgeIndex = E23;
                var edge = new MeshEdge(triangulation[t, V2], triangulation[t, V3], t, triangulation[t, E23], edgeIndex);
                intersectingEdges.Enqueue(edge);
            }
            else if ((triangulation[t, E31] != lastTriangle) && MathUtils.LinesIntersect(v_i, v_j, v3, v1))
            {
                edgeIndex = E31;
                var edge = new MeshEdge(triangulation[t, V3], triangulation[t, V1], t, triangulation[t, E31], edgeIndex);
                intersectingEdges.Enqueue(edge);
            }
            else
            {
                Debug.LogWarning("Failed to find final triangle, exiting early.");
                break;
            }
        }

        return intersectingEdges;
    }

    static bool FindStartingEdge(int[] vertexTriangles, MeshEdge constraint, out MeshEdge startingEdge)
    {
        startingEdge = new MeshEdge(-1, -1);

        int v_i = constraint.v1;
        int v_j = constraint.v2;

        int tSearch = vertexTriangles[v_i];

        for (int i = 0; i < visited.Length; i++)
        {
            visited[i] = false;
        }

        bool intersectionFound = false;
        bool noCandidatesFound = false;
        int intersectingEdgeIndex = E12;
        int tE12, tE23, tE31;
        while (!intersectionFound && !noCandidatesFound)
        {
            visited[tSearch] = true;

            if (TriangleContainsConstraint(tSearch, constraint))
            {
                return false;
            }
            else if (EdgeConstraintIntersectsTriangle(tSearch, constraint, out intersectingEdgeIndex))
            {
                intersectionFound = true;
                break;
            }

            tE12 = triangulation[tSearch, E12];
            tE23 = triangulation[tSearch, E23];
            tE31 = triangulation[tSearch, E31];

            if (tE12 != OUT_OF_BOUNDS && !visited[tE12] && TriangleContainsVertex(tE12, v_i))
            {
                tSearch = tE12;
            }
            else if (tE23 != OUT_OF_BOUNDS && !visited[tE23] && TriangleContainsVertex(tE23, v_i))
            {
                tSearch = tE23;
            }
            else if (tE31 != OUT_OF_BOUNDS && !visited[tE31] && TriangleContainsVertex(tE31, v_i))
            {
                tSearch = tE31;
            }
            else
            {
                noCandidatesFound = true;
                break;
            }
        }

        if (intersectionFound)
        {
            int v_k = triangulation[tSearch, edgeVertex1[intersectingEdgeIndex]];
            int v_l = triangulation[tSearch, edgeVertex2[intersectingEdgeIndex]];
            int triangle2 = triangulation[tSearch, intersectingEdgeIndex];
            startingEdge = new MeshEdge(v_k, v_l, tSearch, triangle2, intersectingEdgeIndex);

            return true;
        }
        else
        {
            return false;
        }
    }

    static void RemoveIntersectingEdges(MeshEdge constraint, Queue<MeshEdge> intersectingEdges)
    {
        List<MeshEdge> newEdges = new List<MeshEdge>();
        MeshEdge edge, newEdge;
        int counter = 0;
        while (intersectingEdges.Count > 0 && counter <= intersectingEdges.Count)
        {
            edge = intersectingEdges.Dequeue();

            Quad quad;
            if (FindQuadFromSharedEdge(edge.t1, edge.t1Edge, out quad))
            {
                if (MathUtils.LinesIntersect(points[quad.q4].coords,
                        points[quad.q3].coords,
                        points[quad.q1].coords,
                        points[quad.q2].coords))
                {
                    SwapQuadDiagonal(quad, intersectingEdges, newEdges, constraints);
                    newEdge = new MeshEdge(quad.q3, quad.q4, quad.t1, quad.t2, E31);
                    if (MathUtils.LinesIntersect(points[constraint.v1].coords,
                            points[constraint.v2].coords,
                            points[quad.q3].coords,
                            points[quad.q4].coords))
                    {
                        intersectingEdges.Enqueue(newEdge);
                    }
                    else
                    {
                        counter = 0;
                        newEdges.Add(newEdge);
                    }
                }
                else
                {
                    intersectingEdges.Enqueue(edge);
                }
            }

            counter++;
        }

        if (newEdges.Count > 0)
        {
            RestoreConstrainedDelauneyTriangulation(constraint, newEdges);
        }
    }

    static void RestoreConstrainedDelauneyTriangulation(MeshEdge constraint, List<MeshEdge> newEdges)
    {
        bool swapOccurred = true;
        int counter = 0;
        while (swapOccurred)
        {
            counter++;
            swapOccurred = false;

            for (int i = 0; i < newEdges.Count; i++)
            {
                MeshEdge edge = newEdges[i];

                if (edge == constraint)
                {
                    continue;
                }

                Quad quad;
                if (FindQuadFromSharedEdge(edge.t1, edge.t1Edge, out quad))
                {
                    if (SwapTest(points[quad.q1].coords, points[quad.q2].coords, points[quad.q3].coords, points[quad.q4].coords))
                    {
                        SwapQuadDiagonal(quad, newEdges, constraints, null);

                        int v_m = quad.q3;
                        int v_n = quad.q4;
                        newEdges[i] = new MeshEdge(v_m, v_n, quad.t1, quad.t2, E31);

                        swapOccurred = true;
                    }
                }
            }
        }
    }

    static void DiscardTrianglesViolatingConstraints()
    {
        for (int i = 0; i < triangleCount; i++)
        {
            skipTriangle[i] = true;
        }

        HashSet<(int, int)> boundaries = new HashSet<(int, int)>();
        for (int i = 0; i < constraints.Count; i++)
        {
            MeshEdge constraint = constraints[i];
            boundaries.Add((constraint.v1, constraint.v2));
        }

        for (int i = 0; i < visited.Length; i++)
        {
            visited[i] = false;
        }

        Queue<int> frontier = new Queue<int>();

        int v1, v2, v3;
        bool boundaryE12, boundaryE23, boundaryE31;
        for (int i = 0; i < triangleCount; i++)
        {
            if (visited[i]) continue;

            v1 = triangulation[i, V1];
            v2 = triangulation[i, V2];
            v3 = triangulation[i, V3];
            boundaryE12 = boundaries.Contains((v1, v2));
            boundaryE23 = boundaries.Contains((v2, v3));
            boundaryE31 = boundaries.Contains((v3, v1));

            if (boundaryE12 || boundaryE23 || boundaryE31)
            {
                skipTriangle[i] = false;
                frontier.Clear();
                if (!boundaryE12)
                {
                    frontier.Enqueue(triangulation[i, E12]);
                }
                if (!boundaryE23)
                {
                    frontier.Enqueue(triangulation[i, E23]);
                }
                if (!boundaryE31)
                {
                    frontier.Enqueue(triangulation[i, E31]);
                }

                while (frontier.Count > 0)
                {
                    int k = frontier.Dequeue();

                    if (k == OUT_OF_BOUNDS || visited[k])
                    {
                        continue;
                    }

                    skipTriangle[k] = false;
                    visited[k] = true;

                    v1 = triangulation[k, V1];
                    v2 = triangulation[k, V2];
                    v3 = triangulation[k, V3];

                    if (!boundaries.Contains((v1, v2)))
                    {
                        frontier.Enqueue(triangulation[k, E12]);
                    }
                    if (!boundaries.Contains((v2, v3)))
                    {
                        frontier.Enqueue(triangulation[k, E23]);
                    }
                    if (!boundaries.Contains((v3, v1)))
                    {
                        frontier.Enqueue(triangulation[k, E31]);
                    }
                }
            }
        }
    }

    static bool TriangleContainsConstraint(int t, MeshEdge constraint)
    {
        return (triangulation[t, V1] == constraint.v1 || triangulation[t, V2] == constraint.v1 || triangulation[t, V3] == constraint.v1) &&
               (triangulation[t, V1] == constraint.v2 || triangulation[t, V2] == constraint.v2 || triangulation[t, V3] == constraint.v2);
    }

    static bool EdgeConstraintIntersectsTriangle(int t, MeshEdge constraint, out int intersectingEdgeIndex)
    {
        float2 v_i = points[constraint.v1].coords;
        float2 v_j = points[constraint.v2].coords;
        float2 v1 = points[triangulation[t, V1]].coords;
        float2 v2 = points[triangulation[t, V2]].coords;
        float2 v3 = points[triangulation[t, V3]].coords;

        if (MathUtils.LinesIntersect(v_i, v_j, v1, v2))
        {
            intersectingEdgeIndex = E12;
            return true;
        }
        else if (MathUtils.LinesIntersect(v_i, v_j, v2, v3))
        {
            intersectingEdgeIndex = E23;
            return true;
        }
        else if (MathUtils.LinesIntersect(v_i, v_j, v3, v1))
        {
            intersectingEdgeIndex = E31;
            return true;
        }
        else
        {
            intersectingEdgeIndex = -1;
            return false;
        }
    }

    static bool FindQuadFromSharedEdge(int t1, int t1SharedEdge, out Quad quad)
    {
        int q1, q2, q3, q4;
        int t1L, t1R, t2L, t2R;

        int t2 = triangulation[t1, t1SharedEdge];
        int t2SharedEdge;
        if (FindSharedEdge(t2, t1, out t2SharedEdge))
        {
            if (t2SharedEdge == E12)
            {
                q2 = triangulation[t2, V1];
                q1 = triangulation[t2, V2];
                q3 = triangulation[t2, V3];
            }
            else if (t2SharedEdge == E23)
            {
                q2 = triangulation[t2, V2];
                q1 = triangulation[t2, V3];
                q3 = triangulation[t2, V1];
            }
            else
            {
                q2 = triangulation[t2, V3];
                q1 = triangulation[t2, V1];
                q3 = triangulation[t2, V2];
            }

            q4 = triangulation[t1, oppositePoint[t1SharedEdge]];

            t1L = triangulation[t1, previousEdge[t1SharedEdge]];
            t1R = triangulation[t1, nextEdge[t1SharedEdge]];
            t2L = triangulation[t2, nextEdge[t2SharedEdge]];
            t2R = triangulation[t2, previousEdge[t2SharedEdge]];

            quad = new Quad(q1, q2, q3, q4, t1, t2, t1L, t1R, t2L, t2R);

            return true;
        }

        quad = new Quad();

        return false;
    }

    static void SwapQuadDiagonal(Quad quad, IEnumerable<MeshEdge> edges1, IEnumerable<MeshEdge> edges2, IEnumerable<MeshEdge> edges3)
    {
        int t1 = quad.t1;
        int t2 = quad.t2;
        int t1R = quad.t1R;
        int t1L = quad.t1L;
        int t2R = quad.t2R;
        int t2L = quad.t2L;

        triangulation[t1, V1] = quad.q4;
        triangulation[t1, V2] = quad.q1;
        triangulation[t1, V3] = quad.q3;

        triangulation[t2, V1] = quad.q4;
        triangulation[t2, V2] = quad.q3;
        triangulation[t2, V3] = quad.q2;

        triangulation[t1, E12] = t1L;
        triangulation[t1, E23] = t2L;
        triangulation[t1, E31] = t2;

        triangulation[t2, E12] = t1;
        triangulation[t2, E23] = t2R;
        triangulation[t2, E31] = t1R;

        UpdateAdjacency(t2L, t2, t1);
        UpdateAdjacency(t1R, t1, t2);

        UpdateEdgesAfterSwap(edges1, t1, t2, t1L, t1R, t2L, t2R);
        UpdateEdgesAfterSwap(edges2, t1, t2, t1L, t1R, t2L, t2R);
        UpdateEdgesAfterSwap(edges3, t1, t2, t1L, t1R, t2L, t2R);

        vertexTriangles[quad.q1] = t1;
        vertexTriangles[quad.q2] = t2;
    }

    static void UpdateEdgesAfterSwap(IEnumerable<MeshEdge> edges, int t1, int t2, int t1L, int t1R, int t2L, int t2R)
    {
        if (edges == null)
        {
            return;
        }

        foreach (MeshEdge edge in edges)
        {
            if (edge.t1 == t1 && edge.t2 == t1R)
            {
                edge.t1 = t2;
                edge.t2 = t1R;
                edge.t1Edge = E31;
            }
            else if (edge.t1 == t1 && edge.t2 == t1L)
            {
                edge.t1Edge = E12;
            }
            else if (edge.t1 == t1R && edge.t2 == t1)
            {
                edge.t2 = t2;
            }
            else if (edge.t1 == t2 && edge.t2 == t2R)
            {
                edge.t1Edge = E23;
            }
            else if (edge.t1 == t2 && edge.t2 == t2L)
            {
                edge.t1 = t1;
                edge.t2 = t2L;
                edge.t1Edge = E23;
            }
            else if (edge.t1 == t2L && edge.t2 == t2)
            {
                edge.t2 = t1;
            }
        }
    }
}

public static class MeshSlicer
{
    public static void Slice(MeshData meshData,
                             float3 sliceNormal,
                             float3 sliceOrigin,
                             out MeshData topSlice,
                             out MeshData bottomSlice)
    {
        topSlice = new MeshData(meshData.vertexCount, meshData.triangleCount);
        bottomSlice = new MeshData(meshData.vertexCount, meshData.triangleCount);

        bool[] side = new bool[meshData.vertexCount];

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

    private static void FillCutFaces(MeshData topSlice,
                                     MeshData bottomSlice,
                                     float3 sliceNormal)
    {
        topSlice.WeldCutFaceVertices();
        if (topSlice.cutVertices.Count < 3) return;
        int[] triangles = MeshTriangulator.Triangulate(topSlice.cutVertices, topSlice.constraints, sliceNormal);
        for (int i = 0; i < topSlice.cutVertices.Count; i++)
        {
            var vertex = topSlice.cutVertices[i];
            var point = MeshTriangulator.points[i];

            float2 uv = new float2(
                MeshTriangulator.normalizationScaleFactor * point.coords.x,
                MeshTriangulator.normalizationScaleFactor * point.coords.y);

            var topVertex = vertex;
            topVertex.normal = sliceNormal;
            topVertex.uv = uv;

            var bottomVertex = vertex;
            bottomVertex.normal = -sliceNormal;
            bottomVertex.uv = uv;

            topSlice.cutVertices[i] = topVertex;
            bottomSlice.cutVertices[i] = bottomVertex;
        }

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
                offsetBottom + triangles[i + 2],
                offsetBottom + triangles[i + 1],
                MeshType.CutFace);
        }
    }

    private static void SplitTriangles(MeshData meshData,
                                       MeshData topSlice,
                                       MeshData bottomSlice,
                                       float3 sliceNormal,
                                       float3 sliceOrigin,
                                       bool[] side,
                                       MeshType type)
    {
        int[] triangles = meshData.triangles[(int)type].ToArray();
        int a, b, c;
        for (int i = 0; i < triangles.Length; i += 3)
        {
            a = triangles[i];
            b = triangles[i + 1];
            c = triangles[i + 2];

            if (side[a] && side[b] && side[c])
            {
                topSlice.AddMappedTriangle(a, b, c, type);
            }
            else if (!side[a] && !side[b] && !side[c])
            {
                bottomSlice.AddMappedTriangle(a, b, c, type);
            }
            else
            {
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
        float s13;
        float s23;
        float3 v13;
        float3 v23;

        MeshVertex v1 = v1_idx < meshData.vertices.Count ? meshData.vertices[v1_idx] : meshData.cutVertices[v1_idx - meshData.vertices.Count];
        MeshVertex v2 = v2_idx < meshData.vertices.Count ? meshData.vertices[v2_idx] : meshData.cutVertices[v2_idx - meshData.vertices.Count];
        MeshVertex v3 = v3_idx < meshData.vertices.Count ? meshData.vertices[v3_idx] : meshData.cutVertices[v3_idx - meshData.vertices.Count];

        if (MathUtils.LinePlaneIntersection(v1.position, v3.position, sliceNormal, sliceOrigin, out v13, out s13) &&
            MathUtils.LinePlaneIntersection(v2.position, v3.position, sliceNormal, sliceOrigin, out v23, out s23))
        {
            var norm13 = math.normalize(v1.normal + s13 * (v3.normal - v1.normal));
            var norm23 = math.normalize(v2.normal + s23 * (v3.normal - v2.normal));
            var uv13 = v1.uv + s13 * (v3.uv - v1.uv);
            var uv23 = v2.uv + s23 * (v3.uv - v2.uv);

            topSlice.AddCutFaceVertex(v13, norm13, uv13);
            topSlice.AddCutFaceVertex(v23, norm23, uv23);
            bottomSlice.AddCutFaceVertex(v13, norm13, uv13);
            bottomSlice.AddCutFaceVertex(v23, norm23, uv23);

            int index13_A = topSlice.vertices.Count - 2;
            int index23_A = topSlice.vertices.Count - 1;
            int index13_B = bottomSlice.vertices.Count - 2;
            int index23_B = bottomSlice.vertices.Count - 1;

            if (v3BelowCutPlane)
            {
                topSlice.AddTriangle(index23_A, index13_A, topSlice.indexMap[v2_idx], type);
                topSlice.AddTriangle(index13_A, topSlice.indexMap[v1_idx], topSlice.indexMap[v2_idx], type);
                bottomSlice.AddTriangle(bottomSlice.indexMap[v3_idx], index13_B, index23_B, type);
                topSlice.constraints.Add(new MeshEdge(topSlice.cutVertices.Count - 2, topSlice.cutVertices.Count - 1));
                bottomSlice.constraints.Add(new MeshEdge(bottomSlice.cutVertices.Count - 1, bottomSlice.cutVertices.Count - 2));
            }
            else
            {
                topSlice.AddTriangle(index13_A, index23_A, topSlice.indexMap[v3_idx], type);
                bottomSlice.AddTriangle(bottomSlice.indexMap[v1_idx], bottomSlice.indexMap[v2_idx], index13_B, type);
                bottomSlice.AddTriangle(bottomSlice.indexMap[v2_idx], index23_B, index13_B, type);
                topSlice.constraints.Add(new MeshEdge(topSlice.cutVertices.Count - 1, topSlice.cutVertices.Count - 2));
                bottomSlice.constraints.Add(new MeshEdge(bottomSlice.cutVertices.Count - 2, bottomSlice.cutVertices.Count - 1));
            }
        }
    }
}