using System;
using Latios.Navigator.Components;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

namespace Latios.Navigator.Utils
{
    public static class NavUtils
    {
        /// <summary>
        ///     Finds the triangle containing a given point in the NavMeshSurfaceBlob.
        /// </summary>
        /// <param name="position">
        ///     The point to check for containment within a triangle.
        /// </param>
        /// <param name="navMeshSurfaceBlob">
        ///     The NavMeshSurfaceBlob containing the triangles to search.
        /// </param>
        /// <param name="containingTriangleIndex">
        ///     The index of the triangle that contains the point, if found. If no triangle contains the point, this will be -1.
        /// </param>
        /// <returns>
        ///     True if a triangle containing the point was found, false otherwise.
        /// </returns>
        public static bool TryFindTriangleContainingPoint(float3 position, ref NavMeshSurfaceBlob navMeshSurfaceBlob,
            out int containingTriangleIndex)
        {
            containingTriangleIndex = -1;
            for (var i = 0; i < navMeshSurfaceBlob.Triangles.Length; i++)
            {
                var triangle = navMeshSurfaceBlob.Triangles[i];
                if (TriMath.IsPointInTriangle(position, triangle))
                {
                    containingTriangleIndex = i;
                    return true;
                }
            }

            return false;
        }

        /// <summary>
        ///     Gets the triangle by index from the NavMeshSurfaceBlob.
        /// </summary>
        /// <param name="index">
        ///     The index of the triangle to retrieve.
        /// </param>
        /// <param name="navMeshSurfaceBlob">
        ///     The NavMeshSurfaceBlob containing the triangles.
        /// </param>
        /// <returns>
        ///     The triangle at the specified index.
        /// </returns>
        /// <exception cref="IndexOutOfRangeException">
        ///     Thrown when the index is out of range of the triangles array in the NavMeshSurfaceBlob.
        /// </exception>
        public static NavTriangle GetTriangleByIndex(int index, ref NavMeshSurfaceBlob navMeshSurfaceBlob)
        {
            if (index < 0 || index >= navMeshSurfaceBlob.Triangles.Length)
                throw new IndexOutOfRangeException("Triangle index is out of range.");

            return navMeshSurfaceBlob.Triangles[index];
        }

        /// <summary>
        ///     Tries to find the shared portal vertices between two triangles.
        /// </summary>
        /// <param name="t1">
        ///     The first triangle.
        /// </param>
        /// <param name="t2">
        ///     The second triangle.
        /// </param>
        /// <param name="p1">
        ///     The first shared vertex.
        /// </param>
        /// <param name="p2">
        ///     The second shared vertex.
        /// </param>
        /// <returns>
        ///     True if two shared vertices were found, false otherwise.
        /// </returns>
        public static bool TryGetSharedPortalVertices(in NavTriangle t1, in NavTriangle t2, out float3 p1,
            out float3 p2)
        {
            p1 = float3.zero;
            p2 = float3.zero;
            var foundCount = 0;
            var t1Indices = new NativeArray<int>(3, Allocator.Temp);
            t1Indices[0] = t1.Ia;
            t1Indices[1] = t1.Ib;
            t1Indices[2] = t1.Ic;
            var t1Coords = new NativeArray<float3>(3, Allocator.Temp);
            t1Coords[0] = t1.PointA;
            t1Coords[1] = t1.PointB;
            t1Coords[2] = t1.PointC;
            var t2Indices = new NativeArray<int>(3, Allocator.Temp);
            t2Indices[0] = t2.Ia;
            t2Indices[1] = t2.Ib;
            t2Indices[2] = t2.Ic;


            // Check for shared vertices between t1 and t2
            for (var i = 0; i < 3; i++)
            {
                var isShared = false;
                for (var j = 0; j < 3; j++)
                    if (t1Indices[i] == t2Indices[j])
                    {
                        isShared = true;
                        break;
                    }

                if (isShared)
                {
                    if (foundCount == 0)
                        p1                       = t1Coords[i];
                    else if (foundCount == 1) p2 = t1Coords[i];

                    foundCount++;
                    if (foundCount == 2) break;
                }
            }

            var currentCenter = t1.Centroid;
            if (TriMath.SignedArea2D(currentCenter, p1, p2) <= 0f) (p1, p2) = (p2, p1);
            t1Indices.Dispose();
            t1Coords.Dispose();
            t2Indices.Dispose();
            return foundCount == 2;
        }

        /// <summary>
        ///     Finds the closest triangle to a point in the NavMeshSurfaceBlob.
        /// </summary>
        /// <param name="point">
        ///     The point to find the closest triangle to.
        /// </param>
        /// <param name="nevMesh">
        ///     The NavMeshSurfaceBlob containing the triangles.
        /// </param>
        /// <param name="containingTriangleIndex">
        ///     The index of the closest triangle found. If no triangle is found, this will be -1.
        /// </param>
        /// <returns>
        ///     True if a triangle was found, false otherwise.
        /// </returns>
        public static bool FindClosestTriangleToPoint(float3 point, ref NavMeshSurfaceBlob nevMesh,
            out int containingTriangleIndex)
        {
            containingTriangleIndex = -1;
            var closestDistanceSq = float.MaxValue;

            for (var i = 0; i < nevMesh.Triangles.Length; i++)
            {
                var triangle = nevMesh.Triangles[i];
                var distanceSq = TriMath.DistanceToTriangleSq(point, triangle);
                if (distanceSq < closestDistanceSq)
                {
                    closestDistanceSq       = distanceSq;
                    containingTriangleIndex = i;
                }
            }

            return containingTriangleIndex != -1;
        }

        #region Debugging

        /// <summary>
        ///     Draws the triangles of a NavMeshSurfaceBlob for debugging purposes.
        /// </summary>
        /// <param name="navMeshSurfaceBlob">
        ///     The NavMeshSurfaceBlob containing the triangles to draw.
        /// </param>
        /// <param name="color">
        ///     The color to use for drawing the triangles.
        /// </param>
        public static void Debug(ref NavMeshSurfaceBlob navMeshSurfaceBlob, Color color)
        {
            for (var i = 0; i < navMeshSurfaceBlob.Triangles.Length; i++)
            {
                var triangle = navMeshSurfaceBlob.Triangles[i];
                UnityEngine.Debug.DrawLine(triangle.PointA, triangle.PointB, color);
                UnityEngine.Debug.DrawLine(triangle.PointB, triangle.PointC, color);
                UnityEngine.Debug.DrawLine(triangle.PointC, triangle.PointA, color);
            }
        }

        /// <summary>
        ///     Draws the adjacency information of a NavMeshSurfaceBlob for debugging purposes.
        /// </summary>
        /// <param name="navMeshSurfaceBlob">
        ///     The NavMeshSurfaceBlob containing the triangles and adjacency information to draw.
        /// </param>
        /// <param name="color">
        ///     The color to use for drawing the adjacency lines.
        /// </param>
        public static void DebugAdjacency(ref NavMeshSurfaceBlob navMeshSurfaceBlob, Color color)
        {
            for (var i = 0; i < navMeshSurfaceBlob.Triangles.Length; i++)
            {
                var offsets = navMeshSurfaceBlob.AdjacencyOffsets[i];
                var triangle = navMeshSurfaceBlob.Triangles[i];

                for (var j = offsets.x; j < offsets.x + offsets.y; j++)
                {
                    var adjacentIndex = navMeshSurfaceBlob.AdjacencyIndices[j];
                    var adjacentTriangle = navMeshSurfaceBlob.Triangles[adjacentIndex];

                    if (TryGetSharedPortalVertices(triangle, adjacentTriangle, out var p1, out var p2))
                        UnityEngine.Debug.DrawLine(p1, p2, color);
                }
            }
        }

        #endregion
    }
}