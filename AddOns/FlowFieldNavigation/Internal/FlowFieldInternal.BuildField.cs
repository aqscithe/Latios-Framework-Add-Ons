using Latios.Psyshock;
using Latios.Transforms;
using Latios.Unsafe;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Entities.UniversalDelegates;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    [BurstCompile]
    internal static partial class FlowFieldInternal
    {
        [BurstCompile]
        internal struct BuildCellsBodiesJob : IJobFor
        {
            [NativeDisableParallelForRestriction]
            internal Field Field;

            public void Execute(int index)
            {
                var position = Field.IndexToWorld(index);
                var rotation = Field.Transform.Value.rotation;
                var halfSize = Field.CellSize.x0y() / 2f;
                Field.CellColliders[index] = new ColliderBody
                {
                    collider = new BoxCollider(float3.zero, halfSize),
                    transform = new TransformQvvs(position, rotation),
                    entity = new Entity { Index = index }
                };
            }
        }

        internal struct ObstaclesProcessor : IFindPairsProcessor
        {
            [NativeDisableParallelForRestriction]
            internal Field Field;

            public void Execute(in FindPairsResult result)
            {
                if (!Physics.DistanceBetween(result.colliderA, result.transformA, result.colliderB, result.transformB, 0.001f, out _))
                    return;

                var index = result.entityB.entity.Index;
                Field.PassabilityMap[index] = -1;
            }
        }

        [BurstCompile]
        internal struct AgentsInfluenceJob : IJobChunk
        {
            [ReadOnly] internal Field Field;
            [ReadOnly] internal FlowFieldAgentsTypeHandles TypeHandles;
            internal NativeParallelMultiHashMap<int, float3> DensityHashMap;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var chunkTransforms = TypeHandles.WorldTransform.Resolve(chunk);
                var chunkVelocities = chunk.GetNativeArray(ref TypeHandles.Velocity);
                var chunkFootprints = chunk.GetNativeArray(ref TypeHandles.AgentFootprint);
                var chunkDensities = chunk.GetNativeArray(ref TypeHandles.AgentDensity);

                var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                while (enumerator.NextEntityIndex(out var i))
                {
                    var position = chunkTransforms[i].position;
                    var footprintSize = chunkFootprints[i].Size;
                    var velocity = chunkVelocities[i].Value;
                    var densityData = chunkDensities[i];
                    
                    if (!Field.TryWorldToFootprint(position, footprintSize, out var footprint)) continue;
                    
                    var minCell = footprint.xy;
                    var maxCell = footprint.zw;
                    var radius = footprintSize / 2f;
            
                    for (var x = minCell.x; x <= maxCell.x; x++)
                    {
                        for (var y = minCell.y; y <= maxCell.y; y++)
                        {
                            var cell = new int2(x, y);
                            if (!Field.IsValidCell(cell)) continue;
                    
                            var cellCenter = Field.CellToWorld(cell);
                    
                            var distance = math.distance(position.xz, cellCenter.xz);
                            if (distance > radius) continue;

                            var normalizedDistance = distance / radius;
                            var weight = densityData.MinWeight + (densityData.MaxWeight - densityData.MinWeight) * 
                                math.pow(1 - normalizedDistance, densityData.Exponent);
                    
                            var index = Field.CellToIndex(cell);
                            DensityHashMap.Add(index, new float3(velocity * weight, weight));
                        }
                    }
                }
            }
        }

        [BurstCompile]
        internal struct AgentsPostProcessJob : IJobFor
        {
            [ReadOnly] internal NativeParallelMultiHashMap<int, float3> DensityHashMap;
            internal NativeArray<float> DensityMap;
            internal NativeArray<float2> MeanVelocityMap;

            public void Execute(int index)
            {
                var densityEnumerator = DensityHashMap.GetValuesForKey(index);
                var count = 0;
                var totalWeight = 0f;
                var totalVelocity = float2.zero;
        
                while (densityEnumerator.MoveNext())
                {
                    var current = densityEnumerator.Current;
                    totalWeight += current.z;
                    totalVelocity += current.xy;
                    count++;
                }

                if (totalWeight <= 0f || count <= 1)
                {
                    DensityMap[index] = 0;
                    MeanVelocityMap[index] = totalVelocity;
                    return;
                }

                DensityMap[index] = math.min(totalWeight, FlowSettings.MaxDensity);
                MeanVelocityMap[index] = totalVelocity / count;
            }
        }
    }
}