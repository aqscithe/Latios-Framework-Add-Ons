using Latios.Navigator.Components;
using Unity.Mathematics;

namespace Latios.Navigator.Utils
{
    public static class TriMath
    {
        /// <summary>
        ///     Determines if a point is inside a triangle in the XZ plane.
        /// </summary>
        /// <param name="point">
        ///     The point to check for containment within the triangle.
        /// </param>
        /// <param name="triangle">
        ///     The triangle defined by three vertices in 3D space.
        /// </param>
        /// <returns>
        ///     True if the point is inside the triangle, false otherwise.
        /// </returns>
        public static bool IsPointInTriangle(float3 point, NavTriangle triangle)
        {
            // Project to XZ plane
            var p_xz = new float2(point.x, point.z);
            var a_xz = new float2(triangle.PointA.x, triangle.PointA.z);
            var b_xz = new float2(triangle.PointB.x, triangle.PointB.z);
            var c_xz = new float2(triangle.PointC.x, triangle.PointC.z);

            // vectors from A to other vertices and point
            var v0_ac = c_xz - a_xz;
            var v1_ab = b_xz - a_xz;
            var v2_ap = p_xz - a_xz;

            // Compute dot products
            var dot00 = math.dot(v0_ac, v0_ac);
            var dot01 = math.dot(v0_ac, v1_ab);
            var dot02 = math.dot(v0_ac, v2_ap);
            var dot11 = math.dot(v1_ab, v1_ab);
            var dot12 = math.dot(v1_ab, v2_ap);

            // Barycentric coordinates
            var denominator = dot00 * dot11 - dot01 * dot01;

            // check for degenerate triangle
            if (math.abs(denominator) < 1e-6f) return false; // Degenerate triangle, cannot determine point inside

            var invDenom = 1f / denominator;
            var u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            var v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            // Check if point is inside the triangle
            return u >= 0 && v >= 0 && u + v <= 1;
        }

        /// <summary>
        ///     Calculates the signed area of a triangle in the XZ plane.
        /// </summary>
        /// <param name="a">
        ///     The first vertex of the triangle.
        /// </param>
        /// <param name="b">
        ///     The second vertex of the triangle.
        /// </param>
        /// <param name="c">
        ///     The third vertex of the triangle.
        /// </param>
        /// <returns>
        ///     The signed area of the triangle in the XZ plane.
        /// </returns>
        public static float SignedArea2D(float3 a, float3 b, float3 c)
        {
            // 2D signed area (XZ plane)
            var ax = b.x - a.x;
            var az = b.z - a.z;
            var bx = c.x - a.x;
            var bz = c.z - a.z;
            return bx * az - ax * bz;
        }

        /// <summary>
        ///     Calculates the squared distance from a point to a triangle in 3D space.
        /// </summary>
        /// <param name="point">
        ///     The point in 3D space from which the distance to the triangle is calculated.
        /// </param>
        /// <param name="triangle">
        ///     The triangle defined by three vertices in 3D space, represented by a <see cref="NavTriangle" /> structure.
        /// </param>
        /// <returns>
        ///     The squared distance from the point to the triangle. If the point is inside the triangle, the distance is 0.
        /// </returns>
        public static float DistanceToTriangleSq(float3 point, NavTriangle triangle)
        {
            var r = triangle.Radius;
            var d = math.distance(point, triangle.Centroid);

            if (d > r)
                // Point is outside the bounding radius of the triangle
                return math.distancesq(point, triangle.Centroid);

            // Point is within the bounding radius, check if it's inside the triangle
            if (IsPointInTriangle(point, triangle))
                // Point is inside the triangle
                return 0f;

            return float.PositiveInfinity;
        }


        /// <summary>
        ///     Calculates the bounding radius of a triangle defined by three points in 3D space.
        /// </summary>
        /// <param name="pA">
        ///     The first vertex of the triangle.
        /// </param>
        /// <param name="pB">
        ///     The second vertex of the triangle.
        /// </param>
        /// <param name="pC">
        ///     The third vertex of the triangle.
        /// </param>
        /// <returns>
        ///     The bounding radius of the triangle, which is the maximum distance from the centroid to any of the triangle's
        ///     vertices.
        /// </returns>
        public static float BoundingRadius(float3 pA, float3 pB, float3 pC)
        {
            var boundingRadius = 0f;
            var centroid = (pA + pB + pC) / 3f;
            var d = math.distance(centroid, pA);
            if (d > boundingRadius) boundingRadius = d;
            d = math.distance(centroid, pB);
            if (d > boundingRadius) boundingRadius = d;
            d = math.distance(centroid, pC);
            if (d > boundingRadius) boundingRadius = d;
            return boundingRadius;
        }
    }
}