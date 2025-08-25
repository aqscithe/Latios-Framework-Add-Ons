using Latios.Psyshock;
using Latios.Transforms;
using Latios.Unsafe;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
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

        internal struct AgentInfluenceData
        {
            public int Index;
            public float2 Velocity;
            public float Weight;
        }

        [BurstCompile]
        internal struct AgentsInfluenceJob : IJobChunk
        {
            [ReadOnly] internal Field Field;
            [ReadOnly] internal FlowFieldAgentsTypeHandles TypeHandles;
            internal UnsafeParallelBlockList Stream;
            
            [NativeSetThreadIndex] int threadIndex;
            
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
                    var radiusSq = radius * radius;
            
                    for (var x = minCell.x; x <= maxCell.x; x++)
                    {
                        for (var y = minCell.y; y <= maxCell.y; y++)
                        {
                            var cell = new int2(x, y);
                            if (!Field.IsValidCell(cell)) continue;
                    
                            var cellCenter = Field.CellToWorld(cell);
                    
                            var distanceSq = math.distancesq(position.xz, cellCenter.xz);
                            if (distanceSq > radiusSq) continue;

                            var normalizedDistance = distanceSq / radiusSq;
                            var t = 1 - normalizedDistance;
                            var weight = math.lerp(densityData.MinWeight, densityData.MaxWeight, t * t);
                    
                            var index = Field.CellToIndex(cell);
                            Stream.Write(new AgentInfluenceData
                            {
                                Index = index,
                                Velocity = velocity,
                                Weight = weight,
                            }, threadIndex);
                        }
                    }
                }
            }
        }

        [BurstCompile]
        internal struct AgentsPostProcessJob : IJob
        {
            [ReadOnly] internal UnsafeParallelBlockList Stream;
            internal NativeArray<float> DensityMap;
            internal NativeArray<float2> MeanVelocityMap;
            internal NativeArray<int> UnitsCountMap;

            public void Execute()
            {
                for (int i = 0; i < DensityMap.Length; i++)
                {
                    DensityMap[i] = 0;
                    MeanVelocityMap[i] = 0;
                    UnitsCountMap[i] = 0;
                }
                
                var enumerator = Stream.GetEnumerator();
                while (enumerator.MoveNext())
                {
                    var current = enumerator.GetCurrentAsRef<AgentInfluenceData>();
                    var index = current.Index;
                    var density = DensityMap[index] + current.Weight;
                    var totalVelocity = MeanVelocityMap[index] + current.Velocity;
                    var totalCount = UnitsCountMap[index] + 1;
                    DensityMap[index] = math.min(density, FlowSettings.MaxDensity);
                    MeanVelocityMap[index] = totalVelocity;
                    UnitsCountMap[index] = totalCount;
                }
            }
        }
    }
}