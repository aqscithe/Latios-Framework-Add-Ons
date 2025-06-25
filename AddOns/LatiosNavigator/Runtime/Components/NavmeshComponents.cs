using Latios.Navigator.Utils;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.Navigator.Components
{
    /// <summary>
    ///     Represents a triangle in the navigation mesh.
    /// </summary>
    public struct NavTriangle
    {
        /// <summary>
        ///     The index of the first vertex in the <see cref="NavMeshSurfaceBlob" />
        /// </summary>
        public int Ia;

        /// <summary>
        ///     The index of the second vertex in the <see cref="NavMeshSurfaceBlob" />
        /// </summary>
        public int Ib;

        /// <summary>
        ///     The index of the third vertex in the <see cref="NavMeshSurfaceBlob" />
        /// </summary>
        public int Ic;

        /// <summary>
        ///     Position of the first vertex in the triangle.
        /// </summary>
        public float3 PointA;

        /// <summary>
        ///     Position of the second vertex in the triangle.
        /// </summary>
        public float3 PointB;

        /// <summary>
        ///     Position of the third vertex in the triangle.
        /// </summary>
        public float3 PointC;

        /// <summary>
        ///     Calculates the centroid of the triangle.
        /// </summary>
        public float3 Centroid => (PointA + PointB + PointC) / 3f;

        /// <summary>
        ///     Returns the radius of the bounding sphere that contains the triangle.
        /// </summary>
        public float Radius => TriMath.BoundingRadius(PointA, PointB, PointC);
    }


    /// <summary>
    ///     Represents a blob asset that contains the navigation mesh surface data.
    /// </summary>
    public struct NavMeshSurfaceBlob
    {
        public BlobArray<NavTriangle> Triangles;
        public BlobArray<int>         AdjacencyIndices;
        public BlobArray<int2>        AdjacencyOffsets; // x = start index, y = count
    }

    public struct NavMeshSurfaceBlobReference : IComponentData
    {
        public BlobAssetReference<NavMeshSurfaceBlob> NavMeshSurfaceBlob;
    }
}