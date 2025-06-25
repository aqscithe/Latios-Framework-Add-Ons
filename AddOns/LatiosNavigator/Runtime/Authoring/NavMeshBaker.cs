using System.Collections.Generic;
using Latios.Authoring;
using Latios.Navigator.Components;
using Latios.Navigator.Internal;
using Unity.AI.Navigation;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine.AI;

namespace Latios.Navigator.Authoring
{
    [DisableAutoCreation]
    internal class NavMeshBaker : SmartBaker<NavMeshSurface, NavMeshSurfaceBakeItem> { }

    [TemporaryBakingType]
    internal struct NavMeshSurfaceBakeItem : ISmartBakeItem<NavMeshSurface>
    {
        SmartBlobberHandle<NavMeshSurfaceBlob> m_navMeshSurfaceHandle;

        public bool Bake(NavMeshSurface authoring, IBaker baker)
        {
            var entity = baker.GetEntity(authoring, TransformUsageFlags.Dynamic);
            baker.AddComponent<NavMeshSurfaceBlobReference>(entity);
            m_navMeshSurfaceHandle = baker.RequestCreateBlobAsset(authoring);
            return true;
        }

        public void PostProcessBlobRequests(EntityManager entityManager, Entity entity)
        {
            var blobAsset = m_navMeshSurfaceHandle.Resolve(entityManager);

            entityManager.SetComponentData(entity, new NavMeshSurfaceBlobReference
            {
                NavMeshSurfaceBlob = blobAsset
            });
        }
    }


    internal struct NaveMeshSmartBlobberRequestFilter : ISmartBlobberRequestFilter<NavMeshSurfaceBlob>
    {
        public NavMeshSurface Surface;
        int3 ToQuantizedKey(float3 v, float precision = 0.001f) => (int3)math.round(v / precision);

        public bool Filter(IBaker baker, Entity blobBakingEntity)
        {
            if (Surface == null)
                return false;

            var triangulation = NavMesh.CalculateTriangulation();

            if (triangulation.vertices.Length == 0 || triangulation.indices.Length == 0)
                return false;


            // Remap vertices to unique quantized keys
            var uniqueVerts = new Dictionary<int3, int>();
            var remappedVerts = new List<float3>();
            var remappedTriangles = new List<Triangle>();
            foreach (var v in triangulation.vertices)
            {
                var key = ToQuantizedKey(v);
                if (!uniqueVerts.TryGetValue(key, out var index))
                {
                    index            = remappedVerts.Count;
                    uniqueVerts[key] = index;
                    remappedVerts.Add(v);
                }
            }

            for (var i = 0; i < triangulation.indices.Length; i += 3)
            {
                var a = uniqueVerts[ToQuantizedKey(triangulation.vertices[triangulation.indices[i]])];
                var b = uniqueVerts[ToQuantizedKey(triangulation.vertices[triangulation.indices[i + 1]])];
                var c = uniqueVerts[ToQuantizedKey(triangulation.vertices[triangulation.indices[i + 2]])];

                remappedTriangles.Add(new Triangle
                {
                    VertexA = a,
                    VertexB = b,
                    VertexC = c
                });
            }

            // Add the temporary baking component
            var buffer = baker.AddBuffer<TriangleElementInput>(blobBakingEntity);
            for (var i = 0; i < remappedTriangles.Count; i++)
                buffer.Add(new TriangleElementInput
                {
                    Ia     = remappedTriangles[i].VertexA,
                    Ib     = remappedTriangles[i].VertexB,
                    Ic     = remappedTriangles[i].VertexC,
                    PointA = remappedVerts[remappedTriangles[i].VertexA],
                    PointB = remappedVerts[remappedTriangles[i].VertexB],
                    PointC = remappedVerts[remappedTriangles[i].VertexC]
                });

            return true;
        }
    }
}