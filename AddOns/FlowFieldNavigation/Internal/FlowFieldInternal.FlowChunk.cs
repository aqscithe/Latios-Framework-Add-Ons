using Latios.Unsafe;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Jobs;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    internal static partial class FlowFieldInternal
    {
        [BurstCompile]
        internal struct CalculateCostsForGoalChunksJob : IJob
        {
            const float HeapCapacityFactor = 0.35f;

            [ReadOnly] internal NativeArray<int> PassabilityMap;
            [ReadOnly] internal NativeHashSet<int2> GoalCells;
            internal int Width, Height;
            internal int2 ClusterSize;

            internal NativeList<int4> Chunks;

            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<float> Costs;

            public void Execute()
            {
                int initialHeapCapacity = math.max(64, (int)(ClusterSize.x * ClusterSize.y * HeapCapacityFactor));
                var queue = new NativePriorityQueue<CostEntry, CostComparer>(initialHeapCapacity, Allocator.Temp);
                var visited = new NativeBitArray(Width * Height, Allocator.Temp);

                var clusters = new NativeHashSet<int4>(4, Allocator.Temp);

                foreach (var goal in GoalCells)
                {
                    queue.Enqueue(new(goal, 0));
                    var cluster = FlowExtensions.GetCluster(goal, ClusterSize, Width, Height);
                    for (Grid.Direction i = Grid.Direction.Up; i <= Grid.Direction.Right; i++)
                    {
                        if (FlowExtensions.TryGetNeighbourCluster(cluster, i, ClusterSize, Width, Height, out var neighbour))
                        {
                            clusters.Add(neighbour);
                        }
                    }
                }

                foreach (var cluster in clusters)
                {
                    Chunks.Add(cluster);
                }

                FlowExtensions.CalculateCostsForCluster(Width, Height, ClusterSize, ref queue, ref Costs, ref PassabilityMap, ref visited);

                queue.Dispose();
                visited.Dispose();
            }
        }
        
        [BurstCompile]
        internal struct CalculateCostsForChunkJob : IJobParallelForDefer
        {
            const float HeapCapacityFactor = 0.35f;
            
            [ReadOnly] internal NativeArray<int> PassabilityMap;
            
            [ReadOnly] internal NativeList<int4> CurrentChunks;
            internal UnsafeParallelBlockList NextChunksStream;
            internal int Width, Height;
            
            internal int2 ClusterSize;
            
            [NativeDisableContainerSafetyRestriction]
            internal NativeArray<float> Costs;

            [NativeSetThreadIndex] int threadIndex;
            
            public void Execute(int clusterIndex)
            {
                var cluster = CurrentChunks[clusterIndex];
                var min = cluster.xy;
                var max = cluster.zw;
                
                var width = max.x - min.x;
                var height = max.y - min.y;

                int initialHeapCapacity = math.max(64, (int)(width * height * HeapCapacityFactor));
                var queue = new NativePriorityQueue<CostEntry, CostComparer>(initialHeapCapacity, Allocator.Temp);
                // var visited = new NativeBitArray(Width * Height, Allocator.Temp);

                for (Grid.Direction i = Grid.Direction.Up; i <= Grid.Direction.Right; i++)
                {
                    if (FlowExtensions.TryGetNeighbourCluster(cluster, i, ClusterSize, Width, Height, out var neighbour))
                    {
                        NextChunksStream.Write(neighbour, threadIndex);
                    }
                }

                // Top edge (y = Max.y + 1) - включая углы
                for (int x = min.x - 1; x <= max.x + 1; x++)
                    TryAddCell(new int2(x, max.y + 1), ref queue);
                // Bottom edge (y = Min.y - 1) - включая углы
                for (int x = min.x - 1; x <= max.x + 1; x++)
                    TryAddCell(new int2(x, min.y - 1), ref queue);
                // Left edge (x = Min.x - 1) - исключая уже обработанные углы
                for (int y = min.y; y <= max.y; y++)
                    TryAddCell(new int2(min.x - 1, y), ref queue);
                // Right edge (x = Max.x + 1) - исключая уже обработанные углы
                for (int y = min.y; y <= max.y; y++)
                    TryAddCell(new int2(max.x + 1, y), ref queue);

                FlowExtensions.CalculateCostsForCluster(Width, Height, new int4(min, max), ref queue, ref Costs, ref PassabilityMap);//, ref visited);

                queue.Dispose();
                // visited.Dispose();
            }

            bool TryAddCell(int2 pos, ref NativePriorityQueue<CostEntry, CostComparer> queue)
            {
                if (!Grid.IsValidCell(pos, Width, Height)) return false;
                var index = Grid.CellToIndex(Width, pos);
                var passability = PassabilityMap[index];
                if (passability < 0 || passability >= FlowSettings.PassabilityLimit) return false;
                queue.Enqueue(new(pos, Costs[index]));
                return true;
            }
        }
        
        [BurstCompile]
        internal struct CollectChunksJob : IJob
        {
            internal UnsafeParallelBlockList NextChunksStream;
            internal NativeList<int4> Chunks;
            
            public void Execute()
            {
                var enumerator = NextChunksStream.GetEnumerator();
                var hashSet = new NativeHashSet<int4>(4, Allocator.Temp);
                while (enumerator.MoveNext())
                {
                    var current = enumerator.GetCurrent<int4>();
                    hashSet.Add(current);
                }
                Chunks.Clear();
                foreach (var int4 in hashSet)
                {
                    Chunks.Add(int4);
                }
            }
        }
    }
}