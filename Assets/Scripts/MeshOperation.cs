using UnityEngine;
using Unity.Mathematics;
using System.Collections.Generic;

public static class MathUtils
{
    /// <summary>
    /// Returns true if the quad specified by the two diagonals a1->a2 and b1->b2 is convex
    /// Quad is convex if a1->a2 and b1->b2 intersect each other
    /// </summary>
    /// <param name="a1">Start point of diagonal A</param>
    /// <param name="a2">End point of diagonal A</param>
    /// <param name="b1">Start point of diagonal B</param>
    /// <param name="b2">End point of diagonal B</param>
    /// <returns></returns>
    public static bool IsQuadConvex(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2)
    {
        return LinesIntersectInternal(a1, a2, b1, b2, true);
    }

    /// <summary>
    /// Returns true lines a1->a2 and b1->b2 is intersect
    /// </summary>
    /// <param name="a1">Start point of line A</param>
    /// <param name="a2">End point of line A</param>
    /// <param name="b1">Start point of line B</param>
    /// <param name="b2">End point of line B</param>
    /// <returns></returns>
    public static bool LinesIntersect(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2)
    {
        return LinesIntersectInternal(a1, a2, b1, b2, false);
    }

    /// <summary>
    /// Returns true lines a1->a2 and b1->b2 is intersect
    /// </summary>
    /// <param name="a1">Start point of line A</param>
    /// <param name="a2">End point of line A</param>
    /// <param name="b1">Start point of line B</param>
    /// <param name="b2">End point of line B</param>
    /// <returns></returns>
    private static bool LinesIntersectInternal(Vector2 a1, Vector2 a2, Vector2 b1, Vector2 b2, bool includeSharedEndpoints)
    {
        Vector2 a12 = new Vector2(a2.x - a1.x, a2.y - a1.y);
        Vector2 b12 = new Vector2(b2.x - b1.x, b2.y - b1.y);

        // If any of the vertices are shared between the two diagonals,
        // the quad collapses into a triangle and is convex by default.
        if (a1 == b1 || a1 == b2 || a2 == b1 || a2 == b2)
        {
            return includeSharedEndpoints;
        }
        else
        {
            // Compute cross product between each point and the opposite diagonal
            // Look at sign of the Z component to see which side of line point is on
            float a1xb = (a1.x - b1.x) * b12.y - (a1.y - b1.y) * b12.x;
            float a2xb = (a2.x - b1.x) * b12.y - (a2.y - b1.y) * b12.x;
            float b1xa = (b1.x - a1.x) * a12.y - (b1.y - a1.y) * a12.x;
            float b2xa = (b2.x - a1.x) * a12.y - (b2.y - a1.y) * a12.x;

            // Check that the points for each diagonal lie on opposite sides of the other
            // diagonal. Quad is also convex if a1/a2 lie on b1->b2 (and vice versa) since
            // the shape collapses into a triangle (hence >= instead of >)
            return ((a1xb >= 0 && a2xb <= 0) || (a1xb <= 0 && a2xb >= 0)) &&
                   ((b1xa >= 0 && b2xa <= 0) || (b1xa <= 0 && b2xa >= 0));
        }
    }

    /// <summary>
    /// Determines the intersection between the line segment a->b and the plane defined by the specified normal and origin point. If an intersection point exists, it is returned via the out parameter `intersection`. The parameter `s` is defined below and is used to properly interpolate normals/uvs for intersection vertices.
    /// </summary>
    /// <param name="a">Start point of line</param>
    /// <param name="b">End point of line</param>
    /// <param name="n">Plane normal</param>
    /// <param name="p0">Plane origin</param>
    /// <param name="x">If intersection exists, intersection point return as out parameter.</param>
    /// <param name="s">Returns the parameterization of the intersection where x = a + (b - a) * s</param>
    /// <returns></returns>
    public static bool LinePlaneIntersection(Vector3 a,
                                             Vector3 b,
                                             Vector3 n,
                                             Vector3 p0,
                                             out Vector3 x,
                                             out float s)
    {
        // Initialize out params
        s = 0;
        x = Vector3.zero;

        // Handle degenerate cases
        if (a == b)
        {
            return false;
        }
        else if (n == Vector3.zero)
        {
            return false;
        }

        // `s` is the parameter for the line segment a -> b where 0.0 <= s <= 1.0
        s = Vector3.Dot(p0 - a, n) / Vector3.Dot(b - a, n);

        if (s >= 0 && s <= 1)
        {
            x = a + (b - a) * s;
            return true;
        }

        return false;
    }

    /// <summary>
    /// Returns true of the point `p` is on the left side of the directed line segment `i` -> `j`
    /// Use for checking if a point is inside of a triangle. Since triangle vertices oriented
    /// CCW, a point on the left side of a triangle edge is "inside" that edge of the triangle.
    /// </summary>
    /// <param name="p">Index of test point in `points` array</param>
    /// <param name="i">Index of first vertex of the edge in the `points` array</param>
    /// /// <param name="j">Index of second vertex of the edge in the `points` array</param>
    /// <returns>True if the point `p` is on the left side of the line `i`->`j`</returns>
    public static bool IsPointOnRightSideOfLine(Vector2 a, Vector2 b, Vector2 c)
    {
        // The <= is essential; if it is <, the whole thing falls apart
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
        float2 areaCenter2d = new float2((minLeft.x + maxRight.x) * 0.5f, (minLeft.y + maxRight.y) * 0.5f);
        isFullSlice = math.length(areaCenter2d - minLeft) >  2 * math.length(areaCenter2d - projPoint);
        int l = 0, r = 180, m = (l + r) / 2; float angle = math.radians(m);
        while (l < r) 
        {
            if (angle - math.sin(angle) >= 2 * sliceRate * math.PI) r = m;
            else l = m + 1;
            m = (l + r) / 2; angle = math.radians(m);
        }
        float2 lineP0, lineP1;
        if (isFullSlice)
        {
            float theta = UnityEngine.Random.Range(0f, 2 * math.PI);
            lineP0 = center2d + new float2(math.cos(theta) * halfWidth, math.sin(theta) * halfHeight);
            float sign = math.sign((lineP0.y - center2d.y) * (projPoint.x - center2d.x) - (lineP0.x - center2d.x) * (projPoint.y - center2d.y));
            lineP1 = center2d + new float2(math.cos(theta + sign * angle) * halfWidth, math.sin(theta + sign * angle) * halfHeight);
        }
        else
        {
            float pointAngle = math.atan2(projPoint.y - center2d.y, projPoint.x - center2d.x);
            float2 projectPoint1 = projPoint - center2d; projectPoint1.x /= halfWidth; projectPoint1.y /= halfHeight;
            float halfAngle = math.acos(math.length(projectPoint1)), delta = UnityEngine.Random.Range(0f, angle - 2 * halfAngle);
            lineP0 = center2d + new float2(math.cos(pointAngle - halfAngle - delta) * halfWidth, math.sin(pointAngle - halfAngle - delta) * halfHeight);
            lineP1 = center2d + new float2(math.cos(pointAngle + angle - halfAngle - delta) * halfWidth, math.sin(pointAngle + angle - halfAngle - delta) * halfHeight);
        }
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

public static class MeshTriangulator
{
    const int V1 = 0, V2 = 1, V3 = 2, E12 = 3, E23 = 4, E31 = 5, SUPERTRIANGLE = 0, OUT_OF_BOUNDS = -1;
    static int N, triangleCount;
    static int[] vertexTriangles, edgeVertex1, edgeVertex2, oppositePoint, nextEdge, previousEdge;
    static int[,] triangulation;
    static bool[] visited, skipTriangle; static List<MeshEdge> constraints;

    static public TriangulationPoint[] points; static public float normalizationScaleFactor = 1f;

    static public int[] Triangulate(List<MeshVertex> inputPoints, List<MeshEdge> constraints, Vector3 normal)
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

        Vector3 e1 = math.normalize(inputPoints[0].position - inputPoints[1].position);
        Vector3 e2 = normal.normalized;
        Vector3 e3 = Vector3.Cross(e1, e2).normalized;

        for (int i = 0; i < N; i++)
        {
            var position = inputPoints[i].position;
            var coords = new Vector2(Vector3.Dot(position, e1), Vector3.Dot(position, e3));
            points[i] = new TriangulationPoint(i, coords);
        }
        // Need at least 3 vertices to triangulate
        if (N < 3) return null;

        AddSuperTriangle();
        NormalizeCoordinates();
        ComputeTriangulation();

        if (constraints.Count > 0)
        {
            ApplyConstraints();
            DiscardTrianglesViolatingConstraints();
        }

        MeshTriangulator.DiscardTrianglesWithSuperTriangleVertices();

        List<int> triangles = new List<int>(3 * triangleCount);
        for (int i = 0; i < triangleCount; i++)
        {
            // Add all triangles that don't contain a super-triangle vertex
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
            var normalizedPos = new Vector2(
                (point.coords.x - xMin) / normalizationScaleFactor,
                (point.coords.y - yMin) / normalizationScaleFactor);

            points[i].coords = normalizedPos;
        }
    }

    /// <summary>
    /// Sorts the points into bins using an ordered grid
    /// </summary>
    /// <returns>Returns the array of sorted points</returns>
    static TriangulationPoint[] SortPointsIntoBins()
    {
        // Compute the number of bins along each axis
        int n = Mathf.RoundToInt(Mathf.Pow(N, 0.25f));

        // Total bin count
        int binCount = n * n;

        // Assign bin numbers to each point by taking the normalized coordinates
        // and dividing them into a n x n grid.
        for (int k = 0; k < N; k++)
        {
            var point = MeshTriangulator.points[k];
            int i = (int)(0.99f * n * point.coords.y);
            int j = (int)(0.99f * n * point.coords.x);
            point.bin = BinSort.GetBinNumber(i, j, n);
        }

        return BinSort.Sort<TriangulationPoint>(MeshTriangulator.points, N, binCount);
    }

    /// <summary>
    /// Computes the triangulation of the point set.
    /// </summary>
    /// <returns>Returns true if the triangulation was successful</returns>
    static bool ComputeTriangulation()
    {
        // Index of the current triangle being searched
        int tSearch = 0;
        // Index of the last triangle formed
        int tLast = 0;

        var sortedPoints = SortPointsIntoBins();

        // Loop through each point and insert it into the triangulation
        for (int i = 0; i < N; i++)
        {
            TriangulationPoint point = sortedPoints[i];

            // Insert new point into the triangulation. Start by finding the triangle that contains the point `p`
            // Keep track of how many triangles we visited in case search fails and we get stuck in a loop
            int counter = 0;
            bool pointInserted = false;
            while (!pointInserted)
            {
                if (counter++ > tLast || tSearch == OUT_OF_BOUNDS)
                {
                    break;
                }

                // Get coordinates of triangle vertices
                var v1 = MeshTriangulator.points[triangulation[tSearch, V1]].coords;
                var v2 = MeshTriangulator.points[triangulation[tSearch, V2]].coords;
                var v3 = MeshTriangulator.points[triangulation[tSearch, V3]].coords;

                // Verify that point is on the correct side of each edge of the triangle.
                // If a point is on the left side of an edge, move to the adjacent triangle and check again. The search
                // continues until a containing triangle is found or the point is outside of all triangles
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
                // If it is on the right  side of all three edges, it is contained within the triangle (Unity uses CW winding). 
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

    /// <summary>
    /// Initializes the triangulation by inserting the super triangle
    /// </summary>
    static void AddSuperTriangle()
    {
        // Add new points to the end of the points array
        MeshTriangulator.points[N] = new TriangulationPoint(N, new Vector2(-100f, -100f));
        MeshTriangulator.points[N + 1] = new TriangulationPoint(N + 1, new Vector2(0f, 100f));
        MeshTriangulator.points[N + 2] = new TriangulationPoint(N + 2, new Vector2(100f, -100f));

        // Store supertriangle in the first column of the vertex and adjacency data
        triangulation[SUPERTRIANGLE, V1] = N;
        triangulation[SUPERTRIANGLE, V2] = N + 1;
        triangulation[SUPERTRIANGLE, V3] = N + 2;

        // Zeros signify boundary edges
        triangulation[SUPERTRIANGLE, E12] = OUT_OF_BOUNDS;
        triangulation[SUPERTRIANGLE, E23] = OUT_OF_BOUNDS;
        triangulation[SUPERTRIANGLE, E31] = OUT_OF_BOUNDS;
    }

    /// <summary>
    /// Inserts the point `p` into triangle `t`, replacing it with three new triangles
    /// </summary>
    /// <param name="p">The index of the point to insert</param>
    /// <param name="t">The index of the triangle</param>
    /// <param name="triangleCount">Total number of triangles created so far</param>
    static void InsertPointIntoTriangle(TriangulationPoint p, int t, int triangleCount)
    {
        //                         V1
        //                         *
        //                        /|\
        //                       /3|2\
        //                      /  |  \
        //                     /   |   \
        //                    /    |    \
        //                   /     |     \
        //                  /  t1  |  t3  \
        //                 /       |       \
        //                /      1 * 1      \
        //               /      __/1\__      \
        //              /    __/       \__    \
        //             / 2__/     t2      \__3 \
        //            / _/3                 2\_ \
        //           *---------------------------*
        //         V3                             V2

        int t1 = t;
        int t2 = triangleCount + 1;
        int t3 = triangleCount + 2;

        // Add the vertex & adjacency information for the two new triangles
        // New vertex is set to first vertex of each triangle to help with
        // restoring the triangulation later on
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

        // Triangle index remains the same for E12, no need to update adjacency
        UpdateAdjacency(triangulation[t, E12], t, t3);
        UpdateAdjacency(triangulation[t, E23], t, t2);

        // Replace existing triangle `t` with `t1`
        triangulation[t1, V2] = triangulation[t, V3];
        triangulation[t1, V3] = triangulation[t, V1];
        triangulation[t1, V1] = p.index;

        triangulation[t1, E23] = triangulation[t, E31];
        triangulation[t1, E12] = t2;
        triangulation[t1, E31] = t3;

        // After the triangles have been inserted, restore the Delauney triangulation
        RestoreDelauneyTriangulation(p, t1, t2, t3);
    }

    /// <summary>
    /// Restores the triangulation to a Delauney triangulation after new triangles have been added.
    /// </summary>
    /// <param name="p">Index of the inserted point</param>
    /// <param name="t1">Index of first triangle to check</param>
    /// <param name="t2">Index of second triangle to check</param>
    /// <param name="t3">Index of third triangle to check</param>
    static void RestoreDelauneyTriangulation(TriangulationPoint p, int t1, int t2, int t3)
    {
        int t4;
        Stack<(int, int)> s = new Stack<(int, int)>();

        s.Push((t1, triangulation[t1, E23]));
        s.Push((t2, triangulation[t2, E23]));
        s.Push((t3, triangulation[t3, E23]));

        while (s.Count > 0)
        {
            // Pop next triangle and its adjacent triangle off the stack
            // t1 contains the newly added vertex at V1
            // t2 is adjacent to t1 along the opposite edge of V1
            (t1, t2) = s.Pop();

            if (t2 == OUT_OF_BOUNDS)
            {
                continue;
            }
            // If t2 circumscribes p, the quadrilateral formed by t1+t2 has the
            // diagonal drawn in the wrong direction and needs to be swapped
            else if (SwapQuadDiagonalIfNeeded(p.index, t1, t2, out t3, out t4))
            {
                // Push newly formed triangles onto the stack to see if their diagonals
                // need to be swapped
                s.Push((t1, t3));
                s.Push((t2, t4));
            }
        }
    }

    /// <summary>
    /// Swaps the diagonal of the quadrilateral formed by triangle `t` and the
    /// triangle adjacent to the edge that is opposite of the newly added point
    /// </summary>
    /// <param name="p">The index of the inserted point</param>
    /// <param name="t1">Index of the triangle containing p</param>
    /// <param name="t2">Index of the triangle opposite t1 that shares edge E23 with t1</param>
    /// <param name="t3">Index of triangle adjacent to t1 after swap</param>
    /// <param name="t4">Index of triangle adjacent to t2 after swap</param>
    /// <returns>Returns true if the swap was performed. If the swap was not
    /// performed (e.g. returns false), t3 and t4 are unused.
    /// </returns>
    static bool SwapQuadDiagonalIfNeeded(int p, int t1, int t2, out int t3, out int t4)
    {
        // 1) Form quadrilateral from t1 + t2 (q0->q1->q2->q3)
        // 2) Swap diagonal between q1->q3 to q0->q2
        //
        //               BEFORE                            AFTER
        //  
        //                 q3                                q3
        //    *-------------*-------------*    *-------------*-------------*
        //     \           / \           /      \           /|\           / 
        //      \   t3    /   \   t4    /        \   t3    /3|2\   t4    /  
        //       \       /     \       /          \       /  |  \       /   
        //        \     /       \     /            \     /   |   \     /    
        //         \   /   t2    \   /              \   /    |    \   /     
        //          \ /           \ /                \ /     |     \ /     
        //        q1 *-------------*  q2           q1 * 2 t1 | t2 3 * q2
        //            \2         3/                    \     |     /        
        //             \         /                      \    |    /         
        //              \  t1   /                        \   |   /          
        //               \     /                          \  |  /          
        //                \   /                            \1|1/            
        //                 \1/                              \|/             
        //                  *  q4 == p                       *  q4 == p   
        //

        // Get the vertices of the quad. The new vertex is always located at V1 of the triangle
        int q4 = p;
        int q1, q2, q3;

        // Since t2 might be oriented in any direction, find which edge is adjacent to `t`
        // The 4th vertex of the quad will be opposite this edge. We also need the two triangles
        // t3 and t3 that are adjacent to t2 along the other edges since the adjacency information
        // needs to be updated for those triangles.
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
        else // (triangulation[t2, E31] == t1)
        {
            q1 = triangulation[t2, V1];
            q2 = triangulation[t2, V3];
            q3 = triangulation[t2, V2];

            t3 = triangulation[t2, E12];
            t4 = triangulation[t2, E23];
        }

        // Perform test to see if p lies in the circumcircle of t2
        if (SwapTest(points[q1].coords, points[q2].coords, points[q3].coords, points[q4].coords))
        {
            // Update adjacency for triangles adjacent to t1 and t2
            UpdateAdjacency(t3, t2, t1);
            UpdateAdjacency(triangulation[t1, E31], t1, t2);

            // Perform the swap. As always, put the new vertex as the first vertex of the triangle
            triangulation[t1, V1] = q4;
            triangulation[t1, V2] = q1;
            triangulation[t1, V3] = q3;

            triangulation[t2, V1] = q4;
            triangulation[t2, V2] = q3;
            triangulation[t2, V3] = q2;

            // Update adjacency information (order of operations is important here since we
            // are overwriting data).
            triangulation[t2, E12] = t1;
            triangulation[t2, E23] = t4;
            triangulation[t2, E31] = triangulation[t1, E31];

            // triangulation[t1, E12] = t2;
            triangulation[t1, E23] = t3;
            triangulation[t1, E31] = t2;

            return true;
        }
        else
        {
            return false;
        }
    }

    /// <summary>
    /// Marks any triangles that contain super-triangle vertices as discarded
    /// </summary>
    static void DiscardTrianglesWithSuperTriangleVertices()
    {
        for (int i = 0; i < triangleCount; i++)
        {
            // Add all triangles that don't contain a super-triangle vertex
            if (TriangleContainsVertex(i, N) ||
                TriangleContainsVertex(i, N + 1) ||
                TriangleContainsVertex(i, N + 2))
            {
                skipTriangle[i] = true;
            }
        }
    }

    /// <summary>
    /// Checks to see if the triangle formed by points v1->v2->v3 circumscribes point vP
    /// </summary>
    /// <param name="v1">Coordinates of 1st vertex of triangle</param>
    /// <param name="v2">Coordinates of 2nd vertex of triangle</param>
    /// <param name="v3">Coordinates of 3rd vertex of triangle</param>
    /// <param name="v4">Coordinates of test point</param>
    /// <returns> Returns true if the triangle `t` circumscribes the point `p`</returns>
    static bool SwapTest(Vector2 v1, Vector2 v2, Vector2 v3, Vector2 v4)
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

    /// <summary>
    /// Checks if the triangle `t` contains the specified vertex
    /// </summary>
    /// <param name="t">The index of the triangle</param>
    /// <param name="v">The index of the vertex</param>
    /// <returns>Returns true if the triangle `t` contains the vertex `v`</returns>
    static bool TriangleContainsVertex(int t, int v)
    {
        return triangulation[t, V1] == v || triangulation[t, V2] == v || triangulation[t, V3] == v;
    }

    /// <summary>
    /// Updates the adjacency information in triangle `t`. Any references to `tOld are
    /// replaced with `tNew`
    /// </summary>
    /// <param name="t">The index of the triangle to update</param>
    /// <param name="tOld">The index to be replaced</param>
    /// <param name="tNew">The new index to replace with</param>
    static void UpdateAdjacency(int t, int tOld, int tNew)
    {
        // Boundary edge, no triangle exists
        int sharedEdge;
        if (t == OUT_OF_BOUNDS)
        {
            return;
        }
        else if (FindSharedEdge(t, tOld, out sharedEdge))
        {
            triangulation[t, sharedEdge] = tNew;
        }
    }

    /// <summary>
    /// Finds the edge index for triangle `tOrigin` that is adjacent to triangle `tAdjacent`
    /// </summary>
    /// <param name="tOrigin">The origin triangle to search</param>
    /// <param name="tAdjacent">The triangle index to search for</param>
    /// <param name="edgeIndex">Edge index returned as an out parameter</param>
    /// <returns>True if `tOrigin` is adjacent to `tAdjacent` and supplies the
    /// shared edge index via the out parameter. If `tOrigin` is an invalid index or
    /// `tAdjacent` is not adjacent to `tOrigin`, returns false.</returns>
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

    /// <summary>
    /// Applys the edge constraints to the triangulation
    /// </summary>
    static void ApplyConstraints()
    {
        visited = new bool[triangulation.GetLength(0)];

        // Map each vertex to a triangle that contains it
        vertexTriangles = new int[N + 3];
        for (int i = 0; i < triangulation.GetLength(0); i++)
        {
            vertexTriangles[triangulation[i, V1]] = i;
            vertexTriangles[triangulation[i, V2]] = i;
            vertexTriangles[triangulation[i, V3]] = i;
        }

        // Loop through each edge constraint
        foreach (MeshEdge constraint in constraints)
        {
            if (constraint.v1 == constraint.v2) continue;

            // We find the edges of the triangulation that intersect the constraint edge and remove them
            // For each intersecting edge, we identify the triangles that share that edge (which form a quad)
            // The diagonal of this quad is flipped.
            Queue<MeshEdge> intersectingEdges = FindIntersectingEdges(constraint, vertexTriangles);
            RemoveIntersectingEdges(constraint, intersectingEdges);
        }
    }

    /// <summary>
    /// Searches through the triangulation to find intersecting edges
    /// </summary>
    /// <param name="intersectingEdges"></param>
    static Queue<MeshEdge> FindIntersectingEdges(MeshEdge constraint, int[] vertexTriangles)
    {
        Queue<MeshEdge> intersectingEdges = new Queue<MeshEdge>();

        // Need to find the first edge that the constraint crosses.
        MeshEdge startEdge;
        if (FindStartingEdge(vertexTriangles, constraint, out startEdge))
        {
            intersectingEdges.Enqueue(startEdge);
        }
        else
        {
            return intersectingEdges;
        }

        // Search for all triangles that intersect the constraint. Stop when we find a triangle that contains v_j
        int t = startEdge.t1;
        int edgeIndex = startEdge.t1Edge;
        int lastTriangle = t;
        bool finalTriangleFound = false;
        while (!finalTriangleFound)
        {
            // Cross the last intersecting edge and inspect the next triangle
            lastTriangle = t;
            t = triangulation[t, edgeIndex];

            // Get coordinates of constraint end points and triangle vertices
            Vector2 v_i = points[constraint.v1].coords;
            Vector2 v_j = points[constraint.v2].coords;
            Vector2 v1 = points[triangulation[t, V1]].coords;
            Vector2 v2 = points[triangulation[t, V2]].coords;
            Vector2 v3 = points[triangulation[t, V3]].coords;

            // If triangle contains the endpoint of the constraint, the search is done
            if (TriangleContainsVertex(t, constraint.v2))
            {
                finalTriangleFound = true;
            }
            // Otherwise, the constraint must intersect one edge of this triangle. Ignore the edge that we entered from
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
                // Shouldn't reach this point
                Debug.LogWarning("Failed to find final triangle, exiting early.");
                break;
            }
        }

        return intersectingEdges;
    }

    /// <summary>
    /// Finds the starting edge for the search to find all edges that intersect the constraint
    /// </summary>
    /// <param name="constraint">The constraint being used to check for intersections</param>
    static bool FindStartingEdge(int[] vertexTriangles, MeshEdge constraint, out MeshEdge startingEdge)
    {
        // Initialize out parameter to default value
        startingEdge = new MeshEdge(-1, -1);

        // v_i->v_j are the start/end points of the constraint, respectively
        int v_i = constraint.v1;
        int v_j = constraint.v2;

        // Start the search with an initial triangle that contains v_i
        int tSearch = vertexTriangles[v_i];

        // Reset visited states
        for (int i = 0; i < visited.Length; i++)
        {
            visited[i] = false;
        }

        // Circle v_i until we find a triangle that contains an edge which intersects the constraint edge
        // This will be the starting triangle in the search for finding all triangles that intersect the constraint
        bool intersectionFound = false;
        bool noCandidatesFound = false;
        int intersectingEdgeIndex = E12;
        int tE12, tE23, tE31;
        while (!intersectionFound && !noCandidatesFound)
        {
            visited[tSearch] = true;

            // Triangulation already contains the constraint so we ignore the constraint
            if (TriangleContainsConstraint(tSearch, constraint))
            {
                return false;
            }
            // Check if the constraint intersects any edges of this triangle
            else if (EdgeConstraintIntersectsTriangle(tSearch, constraint, out intersectingEdgeIndex))
            {
                intersectionFound = true;
                break;
            }

            tE12 = triangulation[tSearch, E12];
            tE23 = triangulation[tSearch, E23];
            tE31 = triangulation[tSearch, E31];

            // If constraint does not intersect this triangle, check adjacent triangles by crossing edges that have v_i as a vertex
            // Avoid triangles that we have previously visited in the search
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

    /// <summary>
    /// Remove the edges from the triangulation that intersect the constraint. Find two triangles that
    /// share the intersecting edge, swap the diagonal and repeat until no edges intersect the constraint.
    /// </summary>
    /// <param name="constraint">The constraint to check against</param>
    /// <param name="intersectingEdges">A queue containing the previously found edges that intersect the constraint</param>
    static void RemoveIntersectingEdges(MeshEdge constraint, Queue<MeshEdge> intersectingEdges)
    {
        // Remove intersecting edges. Keep track of the new edges that we create
        List<MeshEdge> newEdges = new List<MeshEdge>();
        MeshEdge edge, newEdge;

        // Mark the number of times we have been through the loop. If no new edges
        // have been added after all edges have been visited, stop the loop. Every 
        // time an edge is added to newEdges, reset the counter.
        int counter = 0;

        // Loop through all intersecting edges until they have been properly resolved
        // or they have all been visited with no diagonal swaps.
        while (intersectingEdges.Count > 0 && counter <= intersectingEdges.Count)
        {
            edge = intersectingEdges.Dequeue();

            Quad quad;
            if (FindQuadFromSharedEdge(edge.t1, edge.t1Edge, out quad))
            {
                // If the quad is convex, we swap the diagonal (a quad is convex if the diagonals intersect)
                // Otherwise push it back into the queue so we can swap the diagonal later on.
                if (MathUtils.LinesIntersect(points[quad.q4].coords,
                        points[quad.q3].coords,
                        points[quad.q1].coords,
                        points[quad.q2].coords))
                {
                    // Swap diagonals of the convex quads whose diagonals intersect the constraint
                    SwapQuadDiagonal(quad, intersectingEdges, newEdges, constraints);

                    // The new diagonal is between Q3 and Q4
                    newEdge = new MeshEdge(quad.q3, quad.q4, quad.t1, quad.t2, E31);

                    // If the new diagonal still intersects the constraint edge v_i->v_j,
                    // put back on the list of intersecting eddges
                    if (MathUtils.LinesIntersect(points[constraint.v1].coords,
                            points[constraint.v2].coords,
                            points[quad.q3].coords,
                            points[quad.q4].coords))
                    {
                        intersectingEdges.Enqueue(newEdge);
                    }
                    // Otherwise record in list of new edges
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

        // If any new edges were formed due to a diagonal being swapped, restore the Delauney condition
        // of the triangulation while respecting the constraints
        if (newEdges.Count > 0)
        {
            RestoreConstrainedDelauneyTriangulation(constraint, newEdges);
        }
    }

    /// <summary>
    /// Restores the Delauney triangulation after the constraint has been inserted
    /// </summary>
    /// <param name="constraint">The constraint that was added to the triangulation</param>
    /// <param name="newEdges">The list of new edges that were added</param>
    static void RestoreConstrainedDelauneyTriangulation(MeshEdge constraint, List<MeshEdge> newEdges)
    {
        // Iterate over the list of newly created edges and swap non-constraint diagonals until no more swaps take place
        bool swapOccurred = true;
        int counter = 0;
        while (swapOccurred)
        {
            counter++;
            swapOccurred = false;

            for (int i = 0; i < newEdges.Count; i++)
            {
                MeshEdge edge = newEdges[i];

                // If newly added edge is equal to constraint, we don't want to flip this edge so skip it
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

                        // Enqueue the new diagonal
                        int v_m = quad.q3;
                        int v_n = quad.q4;
                        newEdges[i] = new MeshEdge(v_m, v_n, quad.t1, quad.t2, E31);

                        swapOccurred = true;
                    }
                }
            }
        }
    }

    /// <summary>
    /// Discards triangles that violate the any of the edge constraints
    /// </summary>
    static void DiscardTrianglesViolatingConstraints()
    {
        // Initialize to all triangles being skipped
        for (int i = 0; i < triangleCount; i++)
        {
            skipTriangle[i] = true;
        }

        // Identify the boundary edges
        HashSet<(int, int)> boundaries = new HashSet<(int, int)>();
        for (int i = 0; i < MeshTriangulator.constraints.Count; i++)
        {
            MeshEdge constraint = MeshTriangulator.constraints[i];
            boundaries.Add((constraint.v1, constraint.v2));
        }

        // Reset visited states
        for (int i = 0; i < visited.Length; i++)
        {
            visited[i] = false;
        }

        // Search frontier
        Queue<int> frontier = new Queue<int>();

        int v1, v2, v3;
        bool boundaryE12, boundaryE23, boundaryE31;
        for (int i = 0; i < triangleCount; i++)
        {
            // If we've already visited this triangle, skip it
            if (visited[i])
            {
                continue;
            }

            v1 = triangulation[i, V1];
            v2 = triangulation[i, V2];
            v3 = triangulation[i, V3];
            boundaryE12 = boundaries.Contains((v1, v2));
            boundaryE23 = boundaries.Contains((v2, v3));
            boundaryE31 = boundaries.Contains((v3, v1));

            // If this triangle has a boundary edge, start searching for adjacent triangles
            if (boundaryE12 || boundaryE23 || boundaryE31)
            {
                skipTriangle[i] = false;

                // Search along edges that are not boundary edges
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

                // Recursively search along all non-boundary edges, marking the
                // adjacent triangles as "keep"
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

                    // Continue searching along non-boundary edges
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

    /// <summary>
    /// Determines if the triangle contains the edge constraint
    /// </summary>
    /// <param name="t">The triangle to test</param>
    /// <param name="constraint">The edge constraint</param>
    /// <returns>True if the triangle contains one or both of the endpoints of the constraint</returns>
    static bool TriangleContainsConstraint(int t, MeshEdge constraint)
    {
        return (triangulation[t, V1] == constraint.v1 || triangulation[t, V2] == constraint.v1 || triangulation[t, V3] == constraint.v1) &&
               (triangulation[t, V1] == constraint.v2 || triangulation[t, V2] == constraint.v2 || triangulation[t, V3] == constraint.v2);
    }

    /// <summary>
    /// Returns true if the edge constraint intersects an edge of triangle `t`
    /// </summary>
    /// <param name="t">The triangle to test</param>
    /// <param name="constraint">The edge constraint</param>
    /// <param name="intersectingEdgeIndex">The index of the intersecting edge (E12, E23, E31)</param>
    /// <returns>Returns true if an intersection is found, otherwise false.</returns>
    static bool EdgeConstraintIntersectsTriangle(int t, MeshEdge constraint, out int intersectingEdgeIndex)
    {
        Vector2 v_i = points[constraint.v1].coords;
        Vector2 v_j = points[constraint.v2].coords;
        Vector2 v1 = points[triangulation[t, V1]].coords;
        Vector2 v2 = points[triangulation[t, V2]].coords;
        Vector2 v3 = points[triangulation[t, V3]].coords;

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

    /// <summary>
    /// Returns the quad formed by triangle `t1` and the other triangle that shares the intersecting edge
    /// </summary>
    /// <param name="t1">Base triangle</param>
    /// <param name="intersectingEdge">Edge index that is being intersected</param>
    static bool FindQuadFromSharedEdge(int t1, int t1SharedEdge, out Quad quad)
    {
        //               q3        
        //      *---------*---------*
        //       \       / \       /
        //        \ t2L /   \ t2R /
        //         \   /     \   /
        //          \ /   t2  \ /
        //        q1 *---------* q2 
        //          / \   t1  / \    
        //         /   \     /   \     
        //        / t1L \   / t1R \   
        //       /       \ /       \  
        //      *---------*---------*
        //               q4             

        int q1, q2, q3, q4;
        int t1L, t1R, t2L, t2R;

        // t2 is adjacent to t1 along t1Edge
        int t2 = triangulation[t1, t1SharedEdge];
        int t2SharedEdge;
        if (FindSharedEdge(t2, t1, out t2SharedEdge))
        {
            // Get the top 3 vertices of the quad from t2
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
            else // (t2SharedEdge == E31)
            {
                q2 = triangulation[t2, V3];
                q1 = triangulation[t2, V1];
                q3 = triangulation[t2, V2];
            }

            // q4 is the point in t1 opposite of the shared edge
            q4 = triangulation[t1, oppositePoint[t1SharedEdge]];

            // Get the adjacent triangles to make updating adjacency easier
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

    /// <summary>
    /// Swaps the diagonal of the quadrilateral q0->q1->q2->q3 formed by t1 and t2
    /// </summary>
    /// <param name="">The quad that will have its diagonal swapped</param>
    static void SwapQuadDiagonal(Quad quad, IEnumerable<MeshEdge> edges1, IEnumerable<MeshEdge> edges2, IEnumerable<MeshEdge> edges3)
    {
        // BEFORE
        //               q3        
        //      *---------*---------*
        //       \       / \       /
        //        \ t2L /   \ t2R /
        //         \   /     \   /
        //          \ /   t2  \ /
        //        q1 *---------* q2 
        //          / \   t1  / \    
        //         /   \     /   \     
        //        / t1L \   / t1R \   
        //       /       \ /       \  
        //      *---------*---------*
        //               q4           

        // AFTER
        //               q3        
        //      *---------*---------*
        //       \       /|\       /
        //        \ t2L / | \ t2R /
        //         \   /  |  \   /
        //          \ /   |   \ /
        //        q1 * t1 | t2 * q2 
        //          / \   |   / \    
        //         /   \  |  /   \     
        //        / t1L \ | / t1R \   
        //       /       \|/       \  
        //      *---------*---------*
        //               q4      

        int t1 = quad.t1;
        int t2 = quad.t2;
        int t1R = quad.t1R;
        int t1L = quad.t1L;
        int t2R = quad.t2R;
        int t2L = quad.t2L;

        // Perform the swap. As always, put the new vertex as the first vertex of the triangle
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

        // Update adjacency for the adjacent triangles
        UpdateAdjacency(t2L, t2, t1);
        UpdateAdjacency(t1R, t1, t2);

        // Now that triangles have moved, need to update edges as well
        UpdateEdgesAfterSwap(edges1, t1, t2, t1L, t1R, t2L, t2R);
        UpdateEdgesAfterSwap(edges2, t1, t2, t1L, t1R, t2L, t2R);
        UpdateEdgesAfterSwap(edges3, t1, t2, t1L, t1R, t2L, t2R);

        // Also need to update the vertexTriangles array since the vertices q1 and q2
        // may have been referencing t2/t1 respectively and they are no longer.
        vertexTriangles[quad.q1] = t1;
        vertexTriangles[quad.q2] = t2;
    }

    /// <summary>
    /// Update the Edges
    /// </summary>
    /// <param name="edges"></param>
    /// <param name="t1"></param>
    /// <param name="t2"></param>
    /// <param name="t1L"></param>
    /// <param name="t1R"></param>
    /// <param name="t2L"></param>
    /// <param name="t2R"></param>
    static void UpdateEdgesAfterSwap(IEnumerable<MeshEdge> edges, int t1, int t2, int t1L, int t1R, int t2L, int t2R)
    {
        if (edges == null)
        {
            return;
        }

        // Update edges to reflect changes in triangles
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
                // Triangles stay the same
                edge.t1Edge = E12;
            }
            else if (edge.t1 == t1R && edge.t2 == t1)
            {
                edge.t2 = t2;
            }
            else if (edge.t1 == t1L && edge.t2 == t1)
            {
                // Unchanged
            }
            else if (edge.t1 == t2 && edge.t2 == t2R)
            {
                // Triangles stay the same
                edge.t1Edge = E23;
            }
            else if (edge.t1 == t2 && edge.t2 == t2L)
            {
                edge.t1 = t1;
                edge.t2 = t2L;
                edge.t1Edge = E23;
            }
            else if (edge.t1 == t2R && edge.t2 == t2)
            {
                // Unchanged
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

        // Fill in the cut plane for each mesh.
        // The slice normal points to the "above" mesh, so the face normal for the cut face
        // on the above mesh is opposite of the slice normal. Conversely, normal for the
        // cut face on the "below" mesh is in the direction of the slice normal
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
                                     Vector3 sliceNormal)
    {
        // Since the topSlice and bottomSlice both share the same cut face, we only need to calculate it
        // once. Then the same vertex/triangle data for the face will be used for both slices, except
        // with the normals reversed.

        // First need to weld the coincident vertices for the triangulation to work properly
        topSlice.WeldCutFaceVertices();

        // Need at least 3 vertices to triangulate
        if (topSlice.cutVertices.Count < 3) return;

        // Triangulate the cut face
        int[] triangles = MeshTriangulator.Triangulate(topSlice.cutVertices, topSlice.constraints, sliceNormal);

        // Update normal and UV for the cut face vertices
        for (int i = 0; i < topSlice.cutVertices.Count; i++)
        {
            var vertex = topSlice.cutVertices[i];
            var point = MeshTriangulator.points[i];

            // UV coordinates are based off of the 2D coordinates used for triangulation
            // During triangulation, coordinates are normalized to [0,1], so need to multiply
            // by normalization scale factor to get back to the appropritate scale
            Vector2 uv = new Vector2(
                MeshTriangulator.normalizationScaleFactor * point.coords.x,
                MeshTriangulator.normalizationScaleFactor * point.coords.y);

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
                                       Vector3 sliceNormal,
                                       Vector3 sliceOrigin,
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
                                      Vector3 sliceNormal,
                                      Vector3 sliceOrigin,
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
        Vector3 v13;
        Vector3 v23;

        MeshVertex v1 = v1_idx < meshData.vertices.Count ? meshData.vertices[v1_idx] : meshData.cutVertices[v1_idx - meshData.vertices.Count];
        MeshVertex v2 = v2_idx < meshData.vertices.Count ? meshData.vertices[v2_idx] : meshData.cutVertices[v2_idx - meshData.vertices.Count];
        MeshVertex v3 = v3_idx < meshData.vertices.Count ? meshData.vertices[v3_idx] : meshData.cutVertices[v3_idx - meshData.vertices.Count];

        if (MathUtils.LinePlaneIntersection(v1.position, v3.position, sliceNormal, sliceOrigin, out v13, out s13) &&
            MathUtils.LinePlaneIntersection(v2.position, v3.position, sliceNormal, sliceOrigin, out v23, out s23))
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
}