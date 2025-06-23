using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace Latios.FlowFieldNavigation
{
    internal static partial class FlowFieldInternal
    {
        [BurstCompile]
        internal struct CalculateAgentsDirectionsJob : IJobChunk
        {
            [ReadOnly] internal Flow Flow;
            [ReadOnly] internal Field Field;
            internal FlowFieldAgentsTypeHandles TypeHandles;
            internal float DeltaTime;
            
            public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
            {
                var chunkTransforms = TypeHandles.WorldTransform.Resolve(chunk);
                var controls = chunk.GetNativeArray(ref TypeHandles.AgentDirection);
                var prevPositions = chunk.GetNativeArray(ref TypeHandles.PrevPosition);
                var velocities = chunk.GetNativeArray(ref TypeHandles.Velocity);
                var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

                while (enumerator.NextEntityIndex(out var i))
                {
                    var position = chunkTransforms[i].position;
                    var prevPosition = prevPositions[i].Value;
                    velocities[i] = new FlowField.Velocity { Value = (position.xz - prevPosition) / DeltaTime };
                    prevPositions[i] = new FlowField.PrevPosition { Value = position.xz };
                    
                    var newPos = GetInterpolatedDirection(position, in Field, in Flow);
                    controls[i] = new FlowField.AgentDirection() { Value = newPos };
                }
            }
        }
        
        // static float2 Rk4Integrate(float2 position, float dt, in Field field, in Flow flow)
        // {
        //     float2 k1 = GetInterpolatedDirection(new float3(position.x, 0, position.y), in field, in flow);
        //     float2 k2 = GetInterpolatedDirection(new float3(position.x + k1.x * dt * 0.5f, 0, position.y + k1.y * dt * 0.5f), in field, in flow);
        //     float2 k3 = GetInterpolatedDirection(new float3(position.x + k2.x * dt * 0.5f, 0, position.y + k2.y * dt * 0.5f), in field, in flow);
        //     float2 k4 = GetInterpolatedDirection(new float3(position.x + k3.x * dt, 0, position.y + k3.y * dt), in field, in flow);
        //     return position + (k1 + 2f * k2 + 2f * k3 + k4) * (dt / 6f);
        // }
        
        static float2 GetInterpolatedDirection(float3 worldPos, in Field field, in Flow flow)
        {
            if (!field.TryWorldToFootprint(worldPos, out var footprint, out var interpolation))
            {
                return float2.zero;
            }

            var index00 = Grid.CellToIndex(field.Width, footprint.xy);
            var index01 = Grid.CellToIndex(field.Width, footprint.zy);
            var index10 = Grid.CellToIndex(field.Width, footprint.xw);
            var index11 = Grid.CellToIndex(field.Width, footprint.zw);

            var d00 = flow.GetDirection(index00);
            var d01 = flow.GetDirection(index01);
            var d10 = flow.GetDirection(index10);
            var d11 = flow.GetDirection(index11);

            var bottom = math.lerp(d00, d01, interpolation.x);
            var top = math.lerp(d10, d11, interpolation.x);
            var direction = math.lerp(bottom, top, interpolation.y);
            direction = math.normalizesafe(direction);

            var v00 = field.GetSpeedFactor(index00);
            var v01 = field.GetSpeedFactor(index01);
            var v10 = field.GetSpeedFactor(index10);
            var v11 = field.GetSpeedFactor(index11);

            var vbottom = math.lerp(v00, v01, interpolation.x);
            var vtop = math.lerp(v10, v11, interpolation.x);
            var velocity = math.lerp(vbottom, vtop, interpolation.y);
            velocity = math.saturate(velocity);
            velocity = math.select(0, velocity, velocity > 0);
            return direction * velocity;
        }
    }
}