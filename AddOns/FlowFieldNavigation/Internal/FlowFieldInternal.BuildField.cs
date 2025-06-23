using Latios.Psyshock;
using Latios.Transforms;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
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
            internal NativeParallelMultiHashMap<int, float3>.ParallelWriter DensityHashMap;

            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var chunkTransforms = TypeHandles.WorldTransform.Resolve(chunk);
                var chunkVelocities = chunk.GetNativeArray(ref TypeHandles.Velocity);

                var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                while (enumerator.NextEntityIndex(out var i))
                {
                    var position = chunkTransforms[i].position;
                    if (!Field.TryWorldToFootprint(position, out var footprint, out var interpolation)) continue;
                    var velocity = chunkVelocities[i].Value;
                    
                    var weight00 = (1f - interpolation.x) * (1f - interpolation.y);
                    var weight01 = interpolation.x * (1f - interpolation.y);
                    var weight10 = (1f - interpolation.x) * interpolation.y;
                    var weight11 = interpolation.x * interpolation.y;
                    
                    var i00 = Field.CellToIndex(footprint.xy);
                    DensityHashMap.Add(i00, new(weight00 * velocity, weight00));
                    var i01 = Field.CellToIndex(footprint.zy);
                    DensityHashMap.Add(i01, new(weight01 * velocity, weight01));
                    var i10 = Field.CellToIndex(footprint.xw);
                    DensityHashMap.Add(i10, new(weight10 * velocity, weight10));
                    var i11 = Field.CellToIndex(footprint.zw);
                    DensityHashMap.Add(i11, new(weight11 * velocity, weight11));
                }
            }
        }

        [BurstCompile]
        internal struct AgentsPostProcessJob : IJobFor
        {
            const int MaxUnitsOnCell = 8;
            
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

                if (totalWeight <= 0f)
                {
                    DensityMap[index] = 0;
                    MeanVelocityMap[index] = 0;
                    return;
                }
                var density = math.pow(totalWeight, math.clamp(count, 0, MaxUnitsOnCell));
                DensityMap[index] = density;
                MeanVelocityMap[index] = totalVelocity / totalWeight;
            }
        }
    }
}