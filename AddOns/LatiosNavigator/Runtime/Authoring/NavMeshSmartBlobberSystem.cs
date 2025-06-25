using Latios.Authoring;
using Latios.Authoring.Systems;
using Latios.Navigator.Components;
using Latios.Navigator.Internal;
using Unity.AI.Navigation;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Entities.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Latios.Navigator.Authoring
{
    public static class NavMeshSmartBlobberBakerExtensions
    {
        public static SmartBlobberHandle<NavMeshSurfaceBlob>
            RequestCreateBlobAsset(this IBaker baker, NavMeshSurface authoring) =>
            baker.RequestCreateBlobAsset<NavMeshSurfaceBlob, NaveMeshSmartBlobberRequestFilter>(
                new NaveMeshSmartBlobberRequestFilter
                {
                    Surface = authoring
                });
    }

    [RequireMatchingQueriesForUpdate]
    [WorldSystemFilter(WorldSystemFilterFlags.BakingSystem)]
    [UpdateInGroup(typeof(SmartBlobberBakingGroup))]
    [BurstCompile]
    public partial struct NavMeshSmartBlobberSystem : ISystem
    {
        public void OnCreate(ref SystemState state)
        {
            new SmartBlobberTools<NavMeshSurfaceBlob>().Register(state.World);
        }

        [BurstCompile]
        public void OnUpdate(ref SystemState state)
        {
            new Job().ScheduleParallel();
        }

        [WithOptions(EntityQueryOptions.IncludeDisabledEntities | EntityQueryOptions.IncludePrefab)]
        [BurstCompile]
        partial struct Job : IJobEntity
        {
            void Execute(ref SmartBlobberResult result,
                in DynamicBuffer<TriangleElementInput> input)
            {
                var builder = new BlobBuilder(Allocator.Temp);
                ref var root = ref builder.ConstructRoot<NavMeshSurfaceBlob>();


                var trianglesArray = input.Reinterpret<NavTriangle>().AsNativeArray();
                builder.ConstructFromNativeArray(ref root.Triangles, trianglesArray);

                // --- Adjacency Calculation ---
                var edgeToTriangles =
                    new NativeParallelMultiHashMap<Edge, int>(trianglesArray.Length * 3, Allocator.Temp);

                for (var i = 0; i < trianglesArray.Length; i++)
                {
                    var t = trianglesArray[i]; // NavTriangle
                    edgeToTriangles.Add(new Edge { VertexA = math.min(t.Ia, t.Ib), VertexB = math.max(t.Ia, t.Ib) }, i);
                    edgeToTriangles.Add(new Edge { VertexA = math.min(t.Ib, t.Ic), VertexB = math.max(t.Ib, t.Ic) }, i);
                    edgeToTriangles.Add(new Edge { VertexA = math.min(t.Ic, t.Ia), VertexB = math.max(t.Ic, t.Ia) }, i);
                }

                var adjacencyIndicesList = new NativeList<int>(Allocator.Temp);
                var adjacencyOffsetsList = new NativeList<int2>(Allocator.Temp);

                var edges = new NativeArray<Edge>(3, Allocator.Temp);
                var neighborSet = new NativeHashSet<int>(3, Allocator.Temp); // Max 3 neighbors per triangle
                for (var i = 0; i < trianglesArray.Length; i++)
                {
                    var t = trianglesArray[i]; // NavTriangle


                    edges[0] = new Edge { VertexA = math.min(t.Ia, t.Ib), VertexB = math.max(t.Ia, t.Ib) };
                    edges[1] = new Edge { VertexA = math.min(t.Ib, t.Ic), VertexB = math.max(t.Ib, t.Ic) };
                    edges[2] = new Edge { VertexA = math.min(t.Ic, t.Ia), VertexB = math.max(t.Ic, t.Ia) };


                    foreach (var edge in edges)
                    {
                        NativeParallelMultiHashMapIterator<Edge> it;
                        if (edgeToTriangles.TryGetFirstValue(edge, out var neighborIdx, out it))
                            do
                            {
                                if (neighborIdx != i) // Don't add self as neighbor
                                    neighborSet.Add(neighborIdx);
                            } while (edgeToTriangles.TryGetNextValue(out neighborIdx, ref it));
                    }

                    var startIndex = adjacencyIndicesList.Length;
                    var count = neighborSet.Count;

                    foreach (var n in neighborSet)
                        adjacencyIndicesList.Add(n);

                    adjacencyOffsetsList.Add(new int2(startIndex, count));
                    neighborSet.Clear();
                }

                neighborSet.Dispose();
                edges.Dispose();

                builder.ConstructFromNativeArray(ref root.AdjacencyIndices, adjacencyIndicesList.AsArray());
                builder.ConstructFromNativeArray(ref root.AdjacencyOffsets, adjacencyOffsetsList.AsArray());

                adjacencyIndicesList.Dispose();
                adjacencyOffsetsList.Dispose();
                edgeToTriangles.Dispose();

                var typedBlob = builder.CreateBlobAssetReference<NavMeshSurfaceBlob>(Allocator.Persistent);
                result.blob = UnsafeUntypedBlobAssetReference.Create(typedBlob);
            }
        }
    }
}